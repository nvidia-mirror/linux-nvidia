/*
 * Copyright (c) 2010-2023, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/fs.h>
#include <linux/devfreq.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/export.h>
#include <linux/sort.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/clk/tegra.h>
#include <linux/of.h>
#include <linux/pm_qos.h>
#include <trace/events/nvhost.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#if IS_ENABLED(CONFIG_INTERCONNECT) && IS_ENABLED(CONFIG_TEGRA_T23X_GRHOST)
#include <linux/interconnect.h>
#include <linux/platform/tegra/mc_utils.h>
#endif
#if IS_ENABLED(CONFIG_TEGRA_BWMGR)
#include <linux/platform/tegra/emc_bwmgr.h>
#endif

#include <soc/tegra/tegra-dvfs.h>
#include <linux/clk-provider.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 9, 0)
#include <governor.h>
#endif

#include "dev.h"
#include "debug.h"
#include "chip_support.h"
#include "nvhost_acm.h"
#include "nvhost_scale.h"
#include "host1x/host1x_actmon.h"
#include "platform.h"

static ssize_t nvhost_scale_load_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct nvhost_device_data *pdata = dev_get_drvdata(dev);
	struct nvhost_device_profile *profile = pdata->power_profile;
	u32 busy_time;
	ssize_t res;

	actmon_op().read_avg_norm(profile->actmon[ENGINE_ACTMON],
				&busy_time);
	res = snprintf(buf, PAGE_SIZE, "%u\n", busy_time);

	return res;
}

static DEVICE_ATTR(load, S_IRUGO, nvhost_scale_load_show, NULL);

/*
 * nvhost_scale_make_freq_table(profile)
 *
 * This function initialises the frequency table for the given device profile
 */

static int nvhost_scale_make_freq_table(struct nvhost_device_profile *profile)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(profile->pdev);
	unsigned long max_freq  = clk_round_rate(profile->clk, UINT_MAX);
	unsigned long min_freq  = clk_round_rate(profile->clk, 0);
	unsigned long rate      = clk_round_rate(profile->clk, min_freq + 1);
	unsigned long clk_step  = rate - min_freq;
	size_t cnt = 0;
	size_t num_freqs = 0;

	if (clk_step == 0) {
		num_freqs = 1;
	} else {
		num_freqs = ((max_freq - min_freq) / clk_step) + 1;

		num_freqs = ((max_freq - min_freq) % clk_step) ? num_freqs + 1 : num_freqs;
	}

	/* check if clk scaling is available */
	if (min_freq <= 0 || max_freq <= 0)
		return 0;

	pdata->freq_table = devm_kcalloc(&(profile->pdev->dev),
					num_freqs,
					sizeof(*pdata->freq_table),
					GFP_KERNEL);

	if (!pdata->freq_table)
		return -ENOMEM;

	for (rate = min_freq; cnt < num_freqs && rate <= max_freq;
	     rate += clk_step)
		pdata->freq_table[cnt++] = rate;

	profile->devfreq_profile.freq_table = pdata->freq_table;
	profile->devfreq_profile.max_state  = num_freqs;

	return 0;
}

/*
 * nvhost_scale_target(dev, *freq, flags)
 *
 * This function scales the clock
 */

static int nvhost_scale_target(struct device *dev, unsigned long *freq,
			       u32 flags)
{
	struct nvhost_device_data *pdata = dev_get_drvdata(dev);
	struct nvhost_device_profile *profile = pdata->power_profile;

	*freq = clk_round_rate(profile->clk, *freq);
	if (clk_get_rate(profile->clk) == *freq)
		return 0;

	nvhost_module_set_rate(profile->pdev, pdata->power_manager,
				*freq, 0, NVHOST_CLOCK);
	if (pdata->scaling_post_cb)
		pdata->scaling_post_cb(profile, *freq);

	*freq = clk_get_rate(profile->clk);

	return 0;
}

/*
 * update_load_estimate_actmon(profile)
 *
 * Update load estimate using hardware actmon. The actmon value is normalised
 * based on the time it was asked last time.
 */

static void update_load_estimate_actmon(struct nvhost_device_profile *profile)
{
	ktime_t t;
	unsigned long dt;
	u32 busy_time;

	t = ktime_get();
	dt = ktime_us_delta(t, profile->last_event_time);

	profile->dev_stat.total_time = dt;
	profile->last_event_time = t;
	actmon_op().read_avg_norm(profile->actmon[ENGINE_ACTMON],
				&busy_time);
	profile->dev_stat.busy_time = (busy_time * dt) / 1000;
}

/*
 * nvhost_scale_notify(pdev, busy)
 *
 * Calling this function informs that the device is idling (..or busy). This
 * data is used to estimate the current load
 */

static void nvhost_scale_notify(struct platform_device *pdev, bool busy)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_device_profile *profile = pdata->power_profile;
	struct devfreq *devfreq = pdata->power_manager;

	/* Is the device profile initialised? */
	if (!profile)
		return;

	if (nvhost_debug_trace_actmon) {
		u32 load;

		actmon_op().read_avg_norm(profile->actmon[ENGINE_ACTMON],
					&load);
		if (load)
			trace_nvhost_scale_notify(pdev->name, load, busy);
	}

	/* If defreq is disabled, set the freq to max or min */
	if (!devfreq) {
		unsigned long freq = busy ? UINT_MAX : 0;
		nvhost_scale_target(&pdev->dev, &freq, 0);
		return;
	}

	mutex_lock(&devfreq->lock);
	profile->dev_stat.busy = busy;
#if defined(CONFIG_PM_DEVFREQ)
	update_devfreq(devfreq);
#endif
	mutex_unlock(&devfreq->lock);
}

void nvhost_scale_notify_idle(struct platform_device *pdev)
{
	nvhost_scale_notify(pdev, false);

}

void nvhost_scale_notify_busy(struct platform_device *pdev)
{
	nvhost_scale_notify(pdev, true);
}

/*
 * nvhost_scale_get_dev_status(dev, *stat)
 *
 * This function queries the current device status.
 */

static int nvhost_scale_get_dev_status(struct device *dev,
					      struct devfreq_dev_status *stat)
{
	struct nvhost_device_data *pdata = dev_get_drvdata(dev);
	struct nvhost_device_profile *profile = pdata->power_profile;

	/* Make sure there are correct values for the current frequency */
	profile->dev_stat.current_frequency = clk_get_rate(profile->clk);

	if (profile->actmon[ENGINE_ACTMON])
		update_load_estimate_actmon(profile);

	/* Copy the contents of the current device status */
	*stat = profile->dev_stat;

	/* Finally, clear out the local values */
	profile->dev_stat.total_time = 0;
	profile->dev_stat.busy_time = 0;

	return 0;
}

static int nvhost_scale_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct nvhost_device_data *pdata = dev_get_drvdata(dev);
	struct nvhost_device_profile *profile = pdata->power_profile;

	if (freq)
		*freq = clk_get_rate(profile->clk);

	return 0;
}

/*
 * nvhost_scale_set_low_wmark(dev, threshold)
 *
 * This functions sets the high watermark threshold.
 *
 */

static int nvhost_scale_set_low_wmark(struct device *dev,
				      unsigned int threshold)
{
	struct nvhost_device_data *pdata = dev_get_drvdata(dev);
	struct nvhost_device_profile *profile = pdata->power_profile;

	actmon_op().set_low_wmark(profile->actmon[ENGINE_ACTMON],
			threshold);

	return 0;
}

/*
 * nvhost_scale_set_high_wmark(dev, threshold)
 *
 * This functions sets the high watermark threshold.
 *
 */

static int nvhost_scale_set_high_wmark(struct device *dev,
				       unsigned int threshold)
{
	struct nvhost_device_data *pdata = dev_get_drvdata(dev);
	struct nvhost_device_profile *profile = pdata->power_profile;

	actmon_op().set_high_wmark(profile->actmon[ENGINE_ACTMON],
			threshold);

	return 0;
}

static void unregister_opp(struct platform_device *pdev)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 8, 0)
	dev_pm_opp_remove_all_dynamic(&pdev->dev);
#endif
}

static int register_opp(struct platform_device *pdev)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 8, 0)
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_device_profile *profile = pdata->power_profile;
	unsigned long *freq_table = profile->devfreq_profile.freq_table;
	int max_states = profile->devfreq_profile.max_state;
	int err = 0;

	err |= dev_pm_opp_add(&pdev->dev, freq_table[0], 0);
	err |= dev_pm_opp_add(&pdev->dev, freq_table[max_states-1], 0);
	if (err) {
		nvhost_err(&pdev->dev, "Failed to regsiter opp\n");
		unregister_opp(pdev);
	}

	return err;
#else
	return 0;
#endif
}

static int cmp_dev_freq(const void *_a, const void *_b)
{
	const struct nvhost_scale_emc_mapping *a = _a, *b = _b;

	if (a->dev_freq_khz < b->dev_freq_khz)
		return -1;
	else if (a->dev_freq_khz == b->dev_freq_khz)
		return 0;
	else
		return 1;
}

struct nvhost_scale_emc_mapping*
nvhost_scale_emc_map_dt_init(struct device_node *node)
{
	size_t n_elements;
	size_t entry_size = sizeof(struct nvhost_scale_emc_mapping);
	size_t tb_size;
	int n_entries;
	void *tb;

	n_entries = of_property_count_elems_of_size(node, "dev_emc_map",
						    entry_size);
	if (n_entries < 0)
		return NULL;

	tb_size = entry_size * n_entries;

	/* Append termination entry as the last entry in the table */
	tb = kzalloc(tb_size + entry_size, GFP_KERNEL);
	if (!tb)
		return NULL;

	/* Fill the values in the allocated dev_emc mapping table */
	n_elements = tb_size / sizeof(uint32_t);
	if (of_property_read_u32_array(node, "dev_emc_map", tb, n_elements)) {
		kfree(tb);
		return NULL;
	}

	/* Sort the table in ascending order */
	sort(tb, n_entries, entry_size, cmp_dev_freq, NULL);

	return tb;
}

static ssize_t nvhost_scale_emc_map_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct nvhost_scale_emc_mapping *mapping = file->private_data;
	int counter, size;
	ssize_t ret;
	char *kbuf;

	/* Allocate kernel buffer size */
	kbuf = kmalloc(count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	/* Put the header line */
	size = snprintf(kbuf, count, "(devfreq, emcfreq)\n");
	if (size < 0) {
		ret = -EINVAL;
		goto out;
	}

	/* Fill the dev_emc mapping table information in kernel buffer */
	counter = size;
	while (mapping->dev_freq_khz) {
		size = sprintf(kbuf + counter, "%u %u\n",
					mapping->dev_freq_khz, mapping->emc_freq_khz);

		if (size < 0 || (counter > INT_MAX - size)) {
			ret = -EINVAL;
			goto out;
		}
		counter += size;
		mapping++;
	}

	/* Copy from information from kernel space to user space */
	ret = simple_read_from_buffer(buf, count, ppos, kbuf, counter);

out:
	kfree(kbuf);
	return ret;
}

static const struct file_operations nvhost_scale_emc_map_fops = {
	.open = simple_open,
	.read = nvhost_scale_emc_map_read,
	.llseek = default_llseek,
};

static struct dentry *nvhost_debugfs_create_dev_emc_map(struct dentry *parent,
		struct nvhost_scale_emc_mapping *mapping)
{
	if (!parent || !mapping)
		return NULL;

	return debugfs_create_file("dev_emc_map", S_IRUGO, parent,
		mapping, &nvhost_scale_emc_map_fops);
}

/*
 * nvhost_scale_init(pdev)
 */

void nvhost_scale_init(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_device_profile *profile;
	int err, i;
	struct host1x_actmon *actmon;

	if (pdata->power_profile)
		return;

	profile = kzalloc(sizeof(struct nvhost_device_profile), GFP_KERNEL);
	if (!profile)
		return;
	pdata->power_profile = profile;
	profile->pdev = pdev;
	profile->clk = pdata->clk[0];
	profile->dev_stat.busy = false;
	profile->num_actmons = nvhost_get_host(pdev)->info.nb_actmons;

	/* Create frequency table */
	err = nvhost_scale_make_freq_table(profile);
	if (err || !profile->devfreq_profile.max_state)
		goto err_get_freqs;

	err = nvhost_module_busy(nvhost_get_host(pdev)->dev);
	if (err) {
		nvhost_warn(&pdev->dev, "failed to power on host1x.");
		goto err_module_busy;
	}

	/* Create device scale emc mapping table in debugfs */
	if (pdata->dev_emc_map)
		nvhost_debugfs_create_dev_emc_map(pdata->debugfs,
				pdata->dev_emc_map);

	/* Initialize actmon */
	if (pdata->actmon_enabled) {

		if (device_create_file(&pdev->dev,
		    &dev_attr_load)) {
			nvhost_err(&pdev->dev, "failed to create device file");
			goto err_create_sysfs_entry;
		}

		profile->actmon = kzalloc(profile->num_actmons *
					sizeof(struct host1x_actmon *),
					  GFP_KERNEL);
		if (!profile->actmon) {
			nvhost_err(&pdev->dev,
				   "failed to allocate actmon array");
			goto err_allocate_actmons;
		}

		for (i = 0; i < profile->num_actmons; i++) {
			profile->actmon[i] = kzalloc(
					sizeof(struct host1x_actmon),
					GFP_KERNEL);

			if (!profile->actmon[i]) {
				nvhost_err(&pdev->dev,
					   "failed to allocate actmon struct");
				goto err_allocate_actmon;
			}

			actmon = profile->actmon[i];
			actmon->host = nvhost_get_host(pdev);
			actmon->pdev = pdev;

			actmon->type = i;

			actmon->regs = actmon_op().get_actmon_regs(actmon);
			if (!actmon->regs) {
				nvhost_err(&pdev->dev,
					"can't access actmon regs");
				goto err_get_actmon_regs;
			}

			actmon_op().init(actmon);
			nvhost_actmon_debug_init(actmon, pdata->debugfs);
			actmon_op().deinit(actmon);
		}
	}

	/* initialize devfreq if governor is set and actmon enabled */
	if (pdata->actmon_enabled && pdata->devfreq_governor) {
		struct devfreq *devfreq;
		int error = 0;

		register_opp(pdev);

		profile->devfreq_profile.initial_freq =
			profile->devfreq_profile.freq_table[0];
		profile->devfreq_profile.target = nvhost_scale_target;
		profile->devfreq_profile.get_dev_status =
			nvhost_scale_get_dev_status;
		profile->devfreq_profile.get_cur_freq =
			nvhost_scale_get_cur_freq;
		profile->devfreq_profile.set_low_wmark =
			nvhost_scale_set_low_wmark;
		profile->devfreq_profile.set_high_wmark =
			nvhost_scale_set_high_wmark;

		devfreq = devfreq_add_device(&pdev->dev,
					&profile->devfreq_profile,
					pdata->devfreq_governor, NULL);

		if (IS_ERR(devfreq))
			devfreq = NULL;

		pdata->power_manager = devfreq;
		if (nvhost_module_add_client(pdev, devfreq)) {
			nvhost_err(&pdev->dev,
				"failed to register devfreq as acm client");
		}

		/* create symlink 'devfreq_dev' in nvhost dev. */
		if (devfreq != NULL) {
			error = sysfs_create_link(&pdev->dev.kobj,
				&devfreq->dev.kobj, "devfreq_dev");

			if (error) {
				nvhost_err(&pdev->dev,
					"Failed to create devfreq_dev: %d",
					error);
			}
		}
	}

	nvhost_module_idle(nvhost_get_host(pdev)->dev);

	return;
err_get_actmon_regs:
err_allocate_actmon:
	kfree(profile->actmon);
err_allocate_actmons:
	device_remove_file(&pdev->dev, &dev_attr_load);
err_create_sysfs_entry:
	debugfs_remove(pdata->debugfs);
	nvhost_module_idle(nvhost_get_host(pdev)->dev);
err_module_busy:
err_get_freqs:
	kfree(pdata->power_profile);
	pdata->power_profile = NULL;
}

/*
 * nvhost_scale_deinit(dev)
 *
 * Stop scaling for the given device.
 */

void nvhost_scale_deinit(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_device_profile *profile = pdata->power_profile;

	if (!profile)
		return;

	/* Remove intermediate data of devfreq */
	if (pdata->power_manager) {
		sysfs_remove_link(&pdev->dev.kobj, "devfreq_dev");
		nvhost_module_remove_client(pdev, pdata->power_manager);
		devfreq_remove_device(pdata->power_manager);
		unregister_opp(pdev);
	}

	/* Remove intermediate data of actmon */
	if (pdata->actmon_enabled) {
		debugfs_remove(pdata->debugfs);
		device_remove_file(&pdev->dev, &dev_attr_load);
	}

	/* Remove allocated space */
	kfree(profile->devfreq_profile.freq_table);
	kfree(profile->actmon);
	kfree(profile);

	pdata->power_profile = NULL;
}

void nvhost_scale_actmon_irq(struct platform_device *pdev, int type)
{
	struct nvhost_device_data *engine_pdata =
		platform_get_drvdata(pdev);
	struct devfreq *df = engine_pdata->power_manager;

	devfreq_watermark_event(df, type);
}

/*
 * nvhost_scale_hw_init(dev)
 *
 * Initialize hardware portion of the device
 */

int nvhost_scale_hw_init(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_device_profile *profile = pdata->power_profile;
	int i;

	if (!(profile && profile->actmon))
		return 0;

	/* initialize actmon */
	for (i = 0; i < profile->num_actmons; i++)
		actmon_op().init(profile->actmon[i]);

	/* load engine specific actmon settings */
	if (pdata->mamask_addr)
		host1x_writel(pdev, pdata->mamask_addr,
			      pdata->mamask_val);
	if (pdata->borps_addr)
		host1x_writel(pdev, pdata->borps_addr,
			      pdata->borps_val);

	return 0;
}

/*
 * nvhost_scale_hw_deinit(dev)
 *
 * Deinitialize the hw partition related to scaling
 */

void nvhost_scale_hw_deinit(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_device_profile *profile = pdata->power_profile;
	int i;

	if (profile && profile->actmon) {
		if (pdata->mamask_addr)
			host1x_writel(pdev, pdata->mamask_addr, 0x0);

		if (pdata->borps_addr)
			host1x_writel(pdev, pdata->borps_addr, 0x0);

		for (i = 0; i < profile->num_actmons; i++)
			actmon_op().deinit(profile->actmon[i]);
	}
}

/* activity monitor */
static int actmon_count_norm_show(struct seq_file *s, void *unused)
{
	struct host1x_actmon *actmon = s->private;
	u32 avg;
	int err;

	err = actmon_op().read_count_norm(actmon, &avg);
	if (!err)
		seq_printf(s, "%d\n", avg);
	return err;
}

static int actmon_count_norm_open(struct inode *inode, struct file *file)
{
	return single_open(file, actmon_count_norm_show, inode->i_private);
}

static const struct file_operations actmon_count_norm_fops = {
	.open		= actmon_count_norm_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int actmon_count_show(struct seq_file *s, void *unused)
{
	struct host1x_actmon *actmon = s->private;
	u32 avg;
	int err;

	err = actmon_op().read_count(actmon, &avg);
	if (!err)
		seq_printf(s, "%d\n", avg);
	return err;
}

static int actmon_count_open(struct inode *inode, struct file *file)
{
	return single_open(file, actmon_count_show, inode->i_private);
}

static const struct file_operations actmon_count_fops = {
	.open		= actmon_count_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int actmon_avg_show(struct seq_file *s, void *unused)
{
	struct host1x_actmon *actmon = s->private;
	u32 avg;
	int err;

	err = actmon_op().read_avg(actmon, &avg);
	if (!err)
		seq_printf(s, "%d\n", avg);
	return err;
}

static int actmon_avg_open(struct inode *inode, struct file *file)
{
	return single_open(file, actmon_avg_show, inode->i_private);
}

static const struct file_operations actmon_avg_fops = {
	.open		= actmon_avg_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int actmon_avg_norm_show(struct seq_file *s, void *unused)
{
	struct host1x_actmon *actmon = s->private;
	u32 avg;
	int err;

	err = actmon_op().read_avg_norm(actmon, &avg);
	if (!err)
		seq_printf(s, "%d\n", avg);
	return err;
}

static int actmon_avg_norm_open(struct inode *inode, struct file *file)
{
	return single_open(file, actmon_avg_norm_show, inode->i_private);
}

static const struct file_operations actmon_avg_norm_fops = {
	.open		= actmon_avg_norm_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int actmon_sample_period_norm_show(struct seq_file *s, void *unused)
{
	struct host1x_actmon *actmon = s->private;
	long period = actmon_op().get_sample_period_norm(actmon);
	seq_printf(s, "%ld\n", period);
	return 0;
}

static int actmon_sample_period_norm_open(struct inode *inode,
						struct file *file)
{
	return single_open(file, actmon_sample_period_norm_show,
		inode->i_private);
}

static ssize_t actmon_sample_period_norm_write(struct file *file,
				const char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct host1x_actmon *actmon = s->private;
	char buffer[40];
	unsigned int buf_size;
	unsigned long period;

	if (count >= sizeof(buffer))
		nvhost_warn(NULL, "%s: value too big!" \
			"only first %ld characters will be written",
			__func__, sizeof(buffer) - 1);

	buf_size = min(count, (sizeof(buffer)-1));

	if (copy_from_user(buffer, user_buf, buf_size)) {
		nvhost_err(NULL, "failed to copy from user user_buf=%px",
			   user_buf);
		return -EFAULT;
	}
	buffer[buf_size] = '\0';

	if (kstrtoul(buffer, 10, &period)) {
		nvhost_err(NULL, "failed to convert %s to ul", buffer);
		return -EINVAL;
	}

	actmon_op().set_sample_period_norm(actmon, period);

	return buf_size;
}

static const struct file_operations actmon_sample_period_norm_fops = {
	.open		= actmon_sample_period_norm_open,
	.read		= seq_read,
	.write          = actmon_sample_period_norm_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int actmon_sample_period_show(struct seq_file *s, void *unused)
{
	struct host1x_actmon *actmon = s->private;
	long period = actmon_op().get_sample_period(actmon);
	seq_printf(s, "%ld\n", period);
	return 0;
}

static int actmon_sample_period_open(struct inode *inode, struct file *file)
{
	return single_open(file, actmon_sample_period_show, inode->i_private);
}

static const struct file_operations actmon_sample_period_fops = {
	.open		= actmon_sample_period_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int actmon_k_show(struct seq_file *s, void *unused)
{
	struct host1x_actmon *actmon = s->private;
	long period = actmon_op().get_k(actmon);
	seq_printf(s, "%ld\n", period);
	return 0;
}

static int actmon_k_open(struct inode *inode, struct file *file)
{
	return single_open(file, actmon_k_show, inode->i_private);
}

static ssize_t actmon_k_write(struct file *file,
				const char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct host1x_actmon *actmon = s->private;
	char buffer[40];
	unsigned int buf_size;
	unsigned long k;

	if (count >= sizeof(buffer))
		nvhost_warn(NULL, "%s: value too big!" \
			"only first %ld characters will be written",
			__func__, sizeof(buffer) - 1);

	buf_size = min(count, (sizeof(buffer)-1));

	if (copy_from_user(buffer, user_buf, buf_size)) {
		nvhost_err(NULL,
			   "failed to copy from user user_buf=%px", user_buf);
		return -EFAULT;
	}
	buffer[buf_size] = '\0';

	if (strlen(buffer) > buf_size) {
		nvhost_err(NULL, "buffer too large (>%d)", buf_size);
		return -EFAULT;
	}

	if (kstrtoul(buffer, 10, &k)) {
		nvhost_err(NULL, "failed to convert %s to ul", buffer);
		return -EINVAL;
	}

	actmon_op().set_k(actmon, k);

	return buf_size;
}

static const struct file_operations actmon_k_fops = {
	.open		= actmon_k_open,
	.read		= seq_read,
	.write          = actmon_k_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int actmon_consec_upper_num_show(struct seq_file *s, void *unused)
{
	struct host1x_actmon *actmon = s->private;
	long num = actmon_op().get_consec_upper_num(actmon);

	seq_printf(s, "%ld\n", num);
	return 0;
}

static int actmon_consec_upper_num_open(struct inode *inode, struct file *file)
{
	return single_open(file, actmon_consec_upper_num_show, inode->i_private);
}

static ssize_t actmon_consec_upper_num_write(struct file *file,
				const char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct host1x_actmon *actmon = s->private;
	char buffer[40];
	unsigned int buf_size;
	unsigned long num;

	if (count >= sizeof(buffer))
		nvhost_warn(NULL,
			"%s: value too big! only first %ld characters will be written",
			__func__, sizeof(buffer) - 1);

	buf_size = min(count, (sizeof(buffer)-1));

	if (copy_from_user(buffer, user_buf, buf_size)) {
		nvhost_err(NULL,
			   "failed to copy from user user_buf=%p", user_buf);
		return -EFAULT;
	}
	buffer[buf_size] = '\0';

	if (strlen(buffer) > buf_size) {
		nvhost_err(NULL, "buffer too large (>%d)", buf_size);
		return -EFAULT;
	}

	if (kstrtoul(buffer, 10, &num)) {
		nvhost_err(NULL, "failed to convert %s to ul", buffer);
		return -EINVAL;
	}

	actmon_op().set_consec_upper_num(actmon, num);

	return buf_size;
}

static const struct file_operations actmon_consec_upper_num_fops = {
	.open	= actmon_consec_upper_num_open,
	.read	= seq_read,
	.write	= actmon_consec_upper_num_write,
	.llseek	= seq_lseek,
	.release	= single_release,
};

static int actmon_consec_lower_num_show(struct seq_file *s, void *unused)
{
	struct host1x_actmon *actmon = s->private;
	long num = actmon_op().get_consec_lower_num(actmon);

	seq_printf(s, "%ld\n", num);
	return 0;
}

static int actmon_consec_lower_num_open(struct inode *inode, struct file *file)
{
	return single_open(file, actmon_consec_lower_num_show, inode->i_private);
}

static ssize_t actmon_consec_lower_num_write(struct file *file,
				const char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct host1x_actmon *actmon = s->private;
	char buffer[40];
	unsigned int buf_size;
	unsigned long num;

	if (count >= sizeof(buffer))
		nvhost_warn(NULL,
			"%s: value too big! only first %ld characters will be written",
			__func__, sizeof(buffer) - 1);

	buf_size = min(count, (sizeof(buffer)-1));

	if (copy_from_user(buffer, user_buf, buf_size)) {
		nvhost_err(NULL,
			   "failed to copy from user user_buf=%p", user_buf);
		return -EFAULT;
	}
	buffer[buf_size] = '\0';

	if (strlen(buffer) > buf_size) {
		nvhost_err(NULL, "buffer too large (>%d)", buf_size);
		return -EFAULT;
	}

	if (kstrtoul(buffer, 10, &num)) {
		nvhost_err(NULL, "failed to convert %s to ul", buffer);
		return -EINVAL;
	}

	actmon_op().set_consec_lower_num(actmon, num);

	return buf_size;
}

static const struct file_operations actmon_consec_lower_num_fops = {
	.open	= actmon_consec_lower_num_open,
	.read	= seq_read,
	.write	= actmon_consec_lower_num_write,
	.llseek	= seq_lseek,
	.release	= single_release,
};

void nvhost_actmon_debug_init(struct host1x_actmon *actmon,
				     struct dentry *de)
{
	if (!actmon)
		return;

	debugfs_create_file("actmon_k", 0644, de,
			actmon, &actmon_k_fops);
	debugfs_create_file("actmon_sample_period", 0444, de,
			actmon, &actmon_sample_period_fops);
	debugfs_create_file("actmon_sample_period_norm", 0644, de,
			actmon, &actmon_sample_period_norm_fops);
	debugfs_create_file("actmon_avg_norm", 0444, de,
			actmon, &actmon_avg_norm_fops);
	debugfs_create_file("actmon_avg", 0444, de,
			actmon, &actmon_avg_fops);
	debugfs_create_file("actmon_count", 0444, de,
			actmon, &actmon_count_fops);
	debugfs_create_file("actmon_count_norm", 0444, de,
			actmon, &actmon_count_norm_fops);
	debugfs_create_file("actmon_consec_upper_num", 0644, de,
			actmon, &actmon_consec_upper_num_fops);
	debugfs_create_file("actmon_consec_lower_num", 0644, de,
			actmon, &actmon_consec_lower_num_fops);
	/* additional hardware specific debugfs nodes */
	if (actmon_op().debug_init)
		actmon_op().debug_init(actmon, de);

}

static unsigned long
nvhost_scale_to_emc_freq(unsigned long dev_freq, struct nvhost_scale_emc_mapping *mapping)
{
	uint32_t emc_freq = mapping->emc_freq_khz;

	while (mapping->dev_freq_khz) {
		if (dev_freq < mapping->dev_freq_khz)
			break;
		emc_freq = mapping->emc_freq_khz;
		mapping++;
	}

	return emc_freq;
}

void nvhost_scale_callback(struct nvhost_device_profile *profile, unsigned long freq)
{
	unsigned long emc_freq_khz, dev_freq_khz;
	struct nvhost_device_data *pdata = platform_get_drvdata(profile->pdev);
	struct nvhost_scale_emc_mapping *mapping = pdata->dev_emc_map;

#if IS_ENABLED(CONFIG_INTERCONNECT) && IS_ENABLED(CONFIG_TEGRA_T23X_GRHOST)
	/* Scale EMC frequency through ICC framework */
	if (pdata->icc_path_handle && mapping && nvhost_is_234()) {
		unsigned long emc_floor_kbps;

		dev_freq_khz = freq / 1000;
		emc_freq_khz = nvhost_scale_to_emc_freq(dev_freq_khz, mapping);
		emc_floor_kbps = emc_freq_to_bw(emc_freq_khz);
		if (emc_floor_kbps > U32_MAX)
			dev_warn(&profile->pdev->dev,
				 "Type casting out of range!\n");
		else
			icc_set_bw(pdata->icc_path_handle, 0, emc_floor_kbps);
	}
#endif

#if IS_ENABLED(CONFIG_TEGRA_BWMGR)
	/* Scale EMC frequency through bandwidth manager */
	if (pdata->bwmgr_handle && mapping && nvhost_is_194()) {
		dev_freq_khz = freq / 1000;
		emc_freq_khz = nvhost_scale_to_emc_freq(dev_freq_khz, mapping);
		tegra_bwmgr_set_emc(pdata->bwmgr_handle, emc_freq_khz*1000,
				    TEGRA_BWMGR_SET_EMC_FLOOR);
	}
#endif

}
