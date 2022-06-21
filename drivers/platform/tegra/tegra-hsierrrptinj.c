/*
 * SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file tegra-hsierrrptinj.c
 * @brief <b> HSI Error Report Injection driver</b>
 *
 * This file will register as client driver to support triggering
 * HSI error reporting from CCPLEX to FSI.
 */

/* ==================[Includes]============================================= */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/debugfs.h>
#include <linux/pm.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/mailbox_client.h>
#include "linux/tegra-hsierrrptinj.h"


/* Format of input buffer */
/* IP ID : Instance ID : Error Code : Reporter ID : Error Attribute */
/* "0x0000:0x0000:0x0000:0x0000:0x00000000" */
#define ERR_RPT_LEN 39U

#define NUM_INPUTS 5U

#define WRITE_USER 0200 /* S_IWUSR */

/* Max instance of any registred IP */
#define MAX_INSTANCE 11U

/* Timeout in millisec */
#define TIMEOUT		1000

/* Error code for reporting errors to FSI */
#define HSM_ERROR 0x55

/* EC Index for reporting errors to FSI */
#define EC_INDEX 0xFFFFFFFF

/* Error report frame for reprting errors to FSI */
struct hsm_error_report_frame {
	unsigned int type;
	unsigned int error_code;
	unsigned int reporter_id;
	unsigned int ec_index;
};

/**
 * Note: Any update to IP instances array should be reflected in the macro MAX_INSTANCE.
 */

/**
 * @brief IP Instances
 * EQOS  - 1
 * GPU   - 1
 * I2C   - 10
 * MGBE  - 4
 * PCIE  - 11
 * PSC   - 1
 * QSPI  - 2
 * TSEC  - 1
 * SDMMC - 2
 * DLA   - 2
 */
unsigned int ip_instances[NUM_IPS] = {1, 1, 10, 4, 11, 1, 2, 1, 2, 2};

/* This directory entry will point to `/sys/kernel/debug/tegra_hsierrrptinj`. */
static struct dentry *hsierrrptinj_debugfs_root;

/* This file will point to `/sys/kernel/debug/tegra_hsierrrptinj/hsierrrpt`. */
const char *hsierrrptinj_debugfs_name = "hsierrrpt";

/* This array stores callbacks registered by IP Drivers */
static hsierrrpt_inj ip_driver_cb[NUM_IPS][MAX_INSTANCE] = {{NULL}};

/* Data type for mailbox client and channel details */
struct hsierrrptinj_hsp_sm {
	struct mbox_client client;
	struct mbox_chan *chan;
};

static struct hsierrrptinj_hsp_sm hsierrrptinj_tx;


/* Register Error callbacks from IP Drivers */
int hsierrrpt_reg_cb(hsierrrpt_ipid_t ip_id, unsigned int instance_id, hsierrrpt_inj cb_func)
{
	pr_debug("tegra-hsierrrptinj: Register callback for IP Driver 0x%04x\n", ip_id);

	if (cb_func == NULL) {
		pr_err("tegra-hsierrrptinj: Callback function for 0x%04X invalid\n", ip_id);
		return -EINVAL;
	}

	if (ip_id >= NUM_IPS) {
		pr_err("tegra-hsierrrptinj: Invalid IP ID 0x%04x\n", ip_id);
		return -EINVAL;
	}

	if (instance_id >= ip_instances[ip_id]) {
		pr_err("tegra-hsierrrptinj: Invalid instance 0x%04x\n", instance_id);
		return -EINVAL;
	}

	if (ip_driver_cb[ip_id][instance_id] != NULL) {
		pr_err("tegra-hsierrrptinj: Callback for 0x%04X already registered\n", ip_id);
		return -EINVAL;
	}

	ip_driver_cb[ip_id][instance_id] = cb_func;

	pr_debug("tegra-hsierrrptinj: Successfully registered callback for 0x%04X\n", ip_id);

	return 0;
}
EXPORT_SYMBOL(hsierrrpt_reg_cb);

/* Report errors to FSI */
int hsierrrpt_report_to_fsi(struct epl_error_report_frame err_rpt_frame)
{
	int ret = -EINVAL;
	struct hsm_error_report_frame error_report = {0};

	if (IS_ERR(hsierrrptinj_tx.chan)) {
		pr_err("tegra-hsierrrptinj: Mailbox channel or client not initiated\n");
		return -ENODEV;
	}

	error_report.type = HSM_ERROR;
	error_report.error_code = err_rpt_frame.error_code;
	error_report.reporter_id = err_rpt_frame.reporter_id;
	error_report.ec_index = EC_INDEX;

	ret = mbox_send_message(hsierrrptinj_tx.chan, (void *)&error_report);

	return ret < 0 ? ret : 0;
}

/* Parse user entered data via debugfs interface and trigger IP Driver callback */
static ssize_t hsierrrptinj_inject(struct file *file, const char *buf, size_t lbuf, loff_t *ppos)
{
	struct epl_error_report_frame error_report = {0};
	int count = 0, ret = -EINVAL;
	unsigned long val = 0;
	unsigned int ip_id = 0;
	unsigned int instance_id = 0x0000;
	char ubuf[ERR_RPT_LEN] = {0};
	char *token, *cur = ubuf;
	const char *delim = ":";

	pr_debug("tegra-hsierrrptinj: Inject Error Report\n");
	if (buf == NULL) {
		pr_err("tegra-hsierrrptinj: Invalid null input.\n");
		return -EINVAL;
	}

	if (lbuf != ERR_RPT_LEN) {
		pr_err("tegra-hsierrrptinj: Invalid input length.\n");
		return -EINVAL;
	}

	if (copy_from_user(&ubuf, buf, ERR_RPT_LEN)) {
		pr_err("tegra-hsierrrptinj: Failed to copy from input buffer.\n");
		return -EFAULT;
	}

	pr_debug("tegra-hsierrrptinj: Input Buffer: %s\n", cur);

	/* Extract data and trigger */
	while (count < NUM_INPUTS) {

		token = strsep(&cur, delim);
		if (token == NULL) {
			pr_err("tegra-hsierrrptinj: Failed to obtain token\n");
			return -EFAULT;
		}

		ret = kstrtoul(token, 16, &val);
		if (ret < 0) {
			pr_err("tegra-hsierrrptinj: Parsing failed. Error: %d\n", ret);
			return ret;
		}

		switch (count) {

		case 0: /* IP ID */
			pr_debug("tegra-hsierrrptinj: IP ID: 0x%04lx\n", val);
			ip_id = val;
			count++;
			break;
		case 1: /* Instance ID */
			pr_debug("tegra-hsierrrptinj: Instance ID: 0x%04lx\n", val);
			instance_id = val;
			count++;
			break;
		case 2: /* Error Code */
			pr_debug("tegra-hsierrrptinj: HSI Error ID: 0x%04lx\n", val);
			error_report.error_code = val;
			count++;
			break;
		case 3: /* Reporter ID */
			pr_debug("tegra-hsierrrptinj: Reporter ID: 0x%04lx\n", val);
			error_report.reporter_id = val;
			count++;
			break;
		case 4: /* Error Attribute */
			pr_debug("tegra-hsierrrptinj: Error Attribute: 0x%08lx\n", val);
			error_report.error_attribute = val;
			count++;
			break;
		}

	}

	if (count != NUM_INPUTS) {
		pr_err("tegra-hsierrrptinj: Invalid Input format.\n");
		return -EINVAL;
	}

	if (instance_id >= ip_instances[ip_id]) {
		pr_err("tegra-hsierrrptinj: Invalid instance for IP Driver 0x%04x\n", ip_id);
		return -EINVAL;
	}

	/* Get current timestamp */
	asm volatile("mrs %0, cntvct_el0" : "=r" (error_report.timestamp));

	/* IPs not in the hsierrrpt_ipid_t list normally report HSI errors to the FSI
	 * via their local EC, therefore their controlling drivers do not provide a callback.
	 * Directly send error reports for such IPs to the FSI.
	 */
	if (ip_id >= NUM_IPS) {
		ret = hsierrrpt_report_to_fsi(error_report);
		pr_debug("tegra-hsierrrptinj: Reported error to FSI\n");
		goto done;
	}

	/* Trigger IP driver registered callback. If no callback has been registered,
	 * call the EPD-provided API.
	 *
	 * We want certain logging statements to appear in the kernel log with the
	 * OOTB level configuration. Therefore use pr_err for those statements.
	 */
	if (ip_driver_cb[ip_id][instance_id] != NULL) {
		ret = ip_driver_cb[ip_id][instance_id](instance_id, error_report);
		pr_err("tegra-hsierrrptinj: Triggered registered error report callback\n");
	} else {
		ret = epl_report_error(error_report);
		pr_err("tegra-hsierrrptinj: No registered error report trigger callback found\n");
		pr_err("tegra-hsierrrptinj: Reporting HSI error to FSI directly\n");
	}

done:
	if (ret != 0) {
		pr_err("tegra-hsierrrptinj: Failed to report HSI error to FSI\n");
		pr_err("tegra-hsierrrptinj: Error code: %d", ret);
		/* Error code has been logged.
		 * Change error code in case of timeout error
		 * to prevent re-triggering of .write fops.
		 */
		if (ret == -ETIME) {
			/* Explicitly change error code to -EFAULT */
			ret = -EFAULT;
		}
	} else {
		pr_err("tegra-hsierrrptinj: Successfully reported HSI error to FSI\n");
		/* On success, return the error report length.
		 * This represents the number of bytes successfully written.
		 * Returning 0, would re-trigger .write fops
		 */
		ret = ERR_RPT_LEN;
	}

	pr_err("tegra-hsierrrptinj: IP ID: 0x%04x\n", ip_id);
	pr_err("tegra-hsierrrptinj: Instance: 0x%04x\n", instance_id);
	pr_err("tegra-hsierrrptinj: HSI Error ID: 0x%04x\n", error_report.error_code);
	pr_err("tegra-hsierrrptinj: Timestamp: %u\n", error_report.timestamp);

	return ret;
}

static const struct file_operations hsierrrptinj_fops = {
		.read = NULL,
		.write = hsierrrptinj_inject,
};


static int __maybe_unused hsierrrptinj_suspend(struct device *dev)
{
	pr_debug("tegra-hsierrrptinj: suspend called\n");
	return 0;
}

static int __maybe_unused hsierrrptinj_resume(struct device *dev)
{
	pr_debug("tegra-hsierrrptinj: resume called\n");
	return 0;
}
static SIMPLE_DEV_PM_OPS(hsierrrptinj_pm, hsierrrptinj_suspend, hsierrrptinj_resume);

static const struct of_device_id hsierrrptinj_dt_match[] = {
	{ .compatible = "nvidia,tegra23x-hsierrrptinj", },
	{ }
};
MODULE_DEVICE_TABLE(of, hsierrrptinj_dt_match);

static void tegra_hsp_tx_empty_notify(struct mbox_client *cl, void *data, int empty_value)
{
	pr_debug("tegra-hsierrrptinj: TX empty callback came\n");
}

static int tegra_hsp_mb_init(struct device *dev)
{
	int err;

	hsierrrptinj_tx.client.dev = dev;
	hsierrrptinj_tx.client.tx_block = true;
	hsierrrptinj_tx.client.tx_tout = TIMEOUT;
	hsierrrptinj_tx.client.tx_done = tegra_hsp_tx_empty_notify;

	hsierrrptinj_tx.chan = mbox_request_channel_byname(&hsierrrptinj_tx.client,
							    "hsierrrptinj-tx");
	if (IS_ERR(hsierrrptinj_tx.chan)) {
		err = PTR_ERR(hsierrrptinj_tx.chan);
		pr_err("tegra-hsierrrptinj: Failed to get tx mailbox: %d\n", err);
		return err;
	}

	return 0;
}

static int hsierrrptinj_probe(struct platform_device *pdev)
{
	int ret = -EFAULT;
	struct dentry *dent = 0;
	struct device *dev = &pdev->dev;

	/* Initiate TX Mailbox */
	ret = tegra_hsp_mb_init(dev);
	if (ret != 0) {
		pr_err("tegra-hsierrrptinj: Failed initiating tx mailbox\n");
		goto abort;
	}
	pr_err("tegra-hsierrrptinj: Successfully initiated TX Mailbox\n");

	/* Create a directory 'tegra_hsierrrptinj' under 'sys/kernel/debug'
	 * to hold the set of debug files
	 */
	pr_debug("tegra-hsierrrptinj: Create debugfs directory\n");
	hsierrrptinj_debugfs_root = debugfs_create_dir("tegra_hsierrrptinj", NULL);
	if (IS_ERR_OR_NULL(hsierrrptinj_debugfs_root)) {
		pr_err("tegra-hsierrrptinj: Failed to create debug directory\n");
		ret = -EFAULT;
		goto abort;
	}

	/* Create a debug node 'hsierrrpt' under 'sys/kernel/debug/tegra_hsierrrptinj' */
	pr_debug("tegra-hsierrrptinj: Create debugfs node\n");
	dent = debugfs_create_file(hsierrrptinj_debugfs_name, WRITE_USER,
				   hsierrrptinj_debugfs_root, NULL, &hsierrrptinj_fops);
	if (IS_ERR_OR_NULL(dent)) {
		pr_err("tegra-hsierrrptinj: Failed to create debugfs node\n");
		ret = -EFAULT;
		goto abort;
	}
	pr_err("tegra-hsierrrptinj: Debug node created successfully\n");

	pr_debug("tegra-hsierrrptinj: probe success");
	return 0;

abort:
	pr_err("tegra-hsierrrptinj: Failed to create debug node/directory or setup mailbox.\n");
	return ret;
}

static int hsierrrptinj_remove(struct platform_device *pdev)
{
	/* We must explicitly remove the debugfs entries we created. They are not
	 * automatically removed upon module removal.
	 */
	pr_debug("tegra-hsierrrptinj: Recursively remove directory and node created\n");
	debugfs_remove_recursive(hsierrrptinj_debugfs_root);
	return 0;
}

static struct platform_driver hsierrrptinj = {
	.driver         = {
		.name   = "hsierrrptinj",
		.of_match_table = of_match_ptr(hsierrrptinj_dt_match),
		.pm = pm_ptr(&hsierrrptinj_pm),
	},
	.probe          = hsierrrptinj_probe,
	.remove         = hsierrrptinj_remove,
};
module_platform_driver(hsierrrptinj);

MODULE_DESCRIPTION("tegra: HSI Error Report Injection driver");
MODULE_AUTHOR("Prasun Kumar <prasunk@nvidia.com>");
MODULE_LICENSE("GPL v2");
