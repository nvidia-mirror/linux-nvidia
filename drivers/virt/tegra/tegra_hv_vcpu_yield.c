// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES. All rights reserved.

#define pr_fmt(fmt) "%s:%s(): " fmt, KBUILD_MODNAME, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/tick.h>
#include <linux/preempt.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <uapi/linux/tegra_hv_vcpu_yield_ioctl.h>

#define MAX_YIELD_VM_COUNT 10
#define MAX_VCPU_YIELD_TIMEOUT_US 100000

#define DRV_NAME	"tegra_hv_vcpu_yield"

struct vcpu_yield_dev {
	int minor;
	dev_t dev;
	struct cdev cdev;
	struct device *device;
	struct mutex mutex_lock;
	u32 ivc;
	int vcpu;
	int low_prio_vmid;
	unsigned int timeout_us;
	struct hrtimer yield_timer;
	char name[32];
	bool char_is_open;
	bool yield_in_progress;
};

struct vcpu_yield_plat_dev {
	struct vcpu_yield_dev *vcpu_yield_dev_list;
	dev_t vcpu_yield_dev;
	struct class *vcpu_yield_class;
	int vmid_count;
};

static enum hrtimer_restart timer_callback_func(struct hrtimer *hrt)
{
	return HRTIMER_NORESTART;
}

static long vcpu_yield_func(void *data)
{
	ktime_t timeout;
	struct vcpu_yield_dev *vcpu_yield = (struct vcpu_yield_dev *)data;

	timeout = ktime_set(0, (NSEC_PER_USEC * vcpu_yield->timeout_us));

	while (timeout > 0) {

		preempt_disable();
		stop_critical_timings();
		local_irq_disable();

		hrtimer_start(&vcpu_yield->yield_timer, timeout,
			HRTIMER_MODE_REL_PINNED);

		asm volatile("wfi\n");

		timeout = hrtimer_get_remaining(&vcpu_yield->yield_timer);

		hrtimer_cancel(&vcpu_yield->yield_timer);

		local_irq_enable();
		start_critical_timings();
		preempt_enable();
	}

	return 0;
}

static int tegra_hv_vcpu_yield_open(struct inode *inode, struct file *filp)
{
	struct cdev *cdev = inode->i_cdev;
	struct vcpu_yield_dev *data =
		container_of(cdev, struct vcpu_yield_dev, cdev);
	int ret = 0;

	mutex_lock(&data->mutex_lock);
	if (data->char_is_open) {
		mutex_unlock(&data->mutex_lock);
		ret = -EBUSY;
		goto out;
	}

	data->char_is_open = true;
	filp->private_data = data;

	mutex_unlock(&data->mutex_lock);

	hrtimer_init(&data->yield_timer, CLOCK_MONOTONIC,
		HRTIMER_MODE_REL_PINNED);
	data->yield_timer.function = &timer_callback_func;

out:
	return ret;
}

static int tegra_hv_vcpu_yield_release(struct inode *inode, struct file *filp)
{
	struct cdev *cdev = inode->i_cdev;
	struct vcpu_yield_dev *data =
		container_of(cdev, struct vcpu_yield_dev, cdev);

	mutex_lock(&data->mutex_lock);
	data->char_is_open = false;
	filp->private_data = NULL;

	mutex_unlock(&data->mutex_lock);
	return 0;
}

static long tegra_hv_vcpu_yield_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
{
	int ret = 0;
	struct vcpu_yield_dev *data =
		(struct vcpu_yield_dev *)filp->private_data;
	struct vcpu_yield_start_ctl yield_start_ctl_data;

	switch (cmd) {
	case VCPU_YIELD_START_IOCTL:

		mutex_lock(&data->mutex_lock);

		if (data->yield_in_progress == true) {
			mutex_unlock(&data->mutex_lock);
			ret = -EBUSY;
			goto out;
		}

		data->yield_in_progress = true;
		mutex_unlock(&data->mutex_lock);

		ret = copy_from_user(&yield_start_ctl_data,
				(void __user *)arg,
				sizeof(struct vcpu_yield_start_ctl));
		if (ret) {
			pr_err("Failed to copy_from_user :%d\n", ret);
		} else {

			data->timeout_us = yield_start_ctl_data.timeout_us;

			/* Restrict to a Max VCPU Yield Timeout per invocation */
			if (data->timeout_us > MAX_VCPU_YIELD_TIMEOUT_US)
				data->timeout_us = MAX_VCPU_YIELD_TIMEOUT_US;

			ret = work_on_cpu_safe(data->vcpu, vcpu_yield_func,
					(void *)data);
			if (ret)
				pr_err("work_on_cpu_safe Failed :%d\n", ret);
		}

		mutex_lock(&data->mutex_lock);
		data->yield_in_progress = false;
		mutex_unlock(&data->mutex_lock);

		break;

	default:
		pr_err("invalid ioctl command\n");
		ret = -EINVAL;
	}


out:
	return ret;
}

const struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = tegra_hv_vcpu_yield_open,
	.release = tegra_hv_vcpu_yield_release,
	.unlocked_ioctl = tegra_hv_vcpu_yield_ioctl,
};

static int tegra_hv_vcpu_yield_remove(struct platform_device *pdev)
{
	uint32_t i;
	struct vcpu_yield_plat_dev *vcpu_yield_pdev;
	struct vcpu_yield_dev *vcpu_yield, *vcpu_yield_dev_list;
	struct class *vcpu_yield_class;

	vcpu_yield_pdev = (struct vcpu_yield_plat_dev *)dev_get_drvdata(&pdev->dev);
	if (vcpu_yield_pdev) {
		vcpu_yield_dev_list = vcpu_yield_pdev->vcpu_yield_dev_list;
		vcpu_yield_class = vcpu_yield_pdev->vcpu_yield_class;
	}

	if (vcpu_yield_dev_list) {
		for (i = 0; i < vcpu_yield_pdev->vmid_count; i++) {
			vcpu_yield = &vcpu_yield_dev_list[i];

			if (vcpu_yield->device) {
				cdev_del(&vcpu_yield->cdev);
				device_del(vcpu_yield->device);
			}
			device_destroy(vcpu_yield_class, vcpu_yield->dev);
		}
		kfree(vcpu_yield_dev_list);
	}

	if (!IS_ERR_OR_NULL(vcpu_yield_class))
		class_destroy(vcpu_yield_class);

	if (vcpu_yield_pdev->vcpu_yield_dev) {
		unregister_chrdev_region(vcpu_yield_pdev->vcpu_yield_dev,
			vcpu_yield_pdev->vmid_count);
		vcpu_yield_pdev->vcpu_yield_dev = 0;
	}

	kfree(vcpu_yield_pdev);
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static int tegra_hv_vcpu_yield_probe(struct platform_device *pdev)
{
	int result = 0;
	int major = 0;
	int ivc_count = 0;
	int vcpu_count = 0;
	int vmid_count = 0;
	int i = 0;
	struct vcpu_yield_dev *vcpu_yield, *vcpu_yield_dev_list;
	struct vcpu_yield_plat_dev *vcpu_yield_pdev;
	struct device_node *np = pdev->dev.of_node;
	int *vmid_list = NULL;
	int *ivc_list = NULL;
	int *vcpu_list = NULL;
	struct class *vcpu_yield_class;

	vmid_list = kcalloc(MAX_YIELD_VM_COUNT, sizeof(int), GFP_KERNEL);
	if (vmid_list == NULL) {
		pr_err("kcalloc failed for vmid_list\n");
		result = -ENOMEM;
		goto out;
	}

	ivc_list = kcalloc(MAX_YIELD_VM_COUNT, sizeof(int), GFP_KERNEL);
	if (ivc_list == NULL) {
		pr_err("kcalloc failed for ivc_list\n");
		result = -ENOMEM;
		goto out;
	}

	vcpu_list = kcalloc(MAX_YIELD_VM_COUNT, sizeof(int), GFP_KERNEL);
	if (vcpu_list == NULL) {
		pr_err("kcalloc failed for vcpu_list\n");
		result = -ENOMEM;
		goto out;
	}

	vmid_count = of_property_read_variable_u32_array(np, "low_prio_vmid",
			vmid_list, 1, MAX_YIELD_VM_COUNT);
	if (vmid_count < 0) {
		pr_err("low_prio_vmid not specified in device tree node\n");
		result = -EINVAL;
		goto out;
	}

	ivc_count = of_property_read_variable_u32_array(np, "ivc", ivc_list, 1,
			MAX_YIELD_VM_COUNT);
	if (ivc_count < 0) {
		pr_err("ivc not specified in device tree node\n");
		result = -EINVAL;
		goto out;
	} else if (vmid_count != ivc_count) {
		pr_err("ivc count not matching low prio vmid count\n");
		result = -EINVAL;
		goto out;
	}

	vcpu_count = of_property_read_variable_u32_array(np, "yield_vcpu",
			vcpu_list, 1, MAX_YIELD_VM_COUNT);
	if (vcpu_count < 0) {
		pr_err("yield_vcpu not specified in device tree node\n");
		result = -EINVAL;
		goto out;
	} else if (vcpu_count != vmid_count) {
		pr_err("yield_vcpu count not matching low prio vmid count\n");
		result = -EINVAL;
		goto out;
	}

	vcpu_yield_pdev = kzalloc(sizeof(*vcpu_yield_pdev), GFP_KERNEL);
	if (!vcpu_yield_pdev) {
		pr_err("failed to allocate vcpu_yield_pdev");
		result = -ENOMEM;
		goto out;
	}

	vcpu_yield_dev_list = kcalloc(vmid_count, sizeof(*vcpu_yield_dev_list),
				GFP_KERNEL);
	if (!vcpu_yield_dev_list) {
		pr_err("failed to allocate vcpu_yield_dev_list");
		result = -ENOMEM;
		goto out;
	}

	vcpu_yield_pdev->vmid_count = vmid_count;
	vcpu_yield_pdev->vcpu_yield_dev_list = vcpu_yield_dev_list;

	dev_set_drvdata(&pdev->dev, (void *)vcpu_yield_pdev);

	/* allocate the chardev range based on number of low prio vm */
	result = alloc_chrdev_region(&vcpu_yield_pdev->vcpu_yield_dev, 0,
		vmid_count, "tegra_hv_vcpu_yield");
	if (result < 0) {
		pr_err("alloc_chrdev_region() failed\n");
		goto out;
	}

	major = MAJOR(vcpu_yield_pdev->vcpu_yield_dev);
	vcpu_yield_class =
		class_create(THIS_MODULE, "tegra_hv_vcpu_yield");
	if (IS_ERR(vcpu_yield_class)) {
		pr_err("failed to create ivc class: %ld\n", PTR_ERR(vcpu_yield_class));
		result = PTR_ERR(vcpu_yield_class);
		goto out;
	}

	vcpu_yield_pdev->vcpu_yield_class = vcpu_yield_class;

	for (i = 0; i < vmid_count; i++) {
		vcpu_yield = &vcpu_yield_dev_list[i];
		vcpu_yield->dev = MKDEV(major, vmid_list[i]);
		vcpu_yield->low_prio_vmid = vmid_list[i];
		vcpu_yield->ivc = ivc_list[i];
		vcpu_yield->vcpu = vcpu_list[i];
		cdev_init(&vcpu_yield->cdev, &fops);
		result = snprintf(vcpu_yield->name, sizeof(vcpu_yield->name) - 1,
				"tegra_hv_vcpu_yield_vm%d", vmid_list[i]);
		if (result < 0) {
			pr_err("snprintf() failed\n");
			goto out;
		}
		result = cdev_add(&vcpu_yield->cdev, vcpu_yield->dev, 1);
		if (result) {
			pr_err("%s: Failed adding cdev to subsystem retval:%d\n", __func__, result);
			goto out;
		} else {
			vcpu_yield->device = device_create(vcpu_yield_class,
						&pdev->dev, vcpu_yield->dev,
						vcpu_yield, vcpu_yield->name);
			if (IS_ERR(vcpu_yield->device)) {
				pr_err("device_create() failed for %s\n", vcpu_yield->name);
				result = PTR_ERR(vcpu_yield->device);
				goto out;
			}

		}

		mutex_init(&vcpu_yield->mutex_lock);
	}

out:
	kfree(vmid_list);
	kfree(ivc_list);
	kfree(vcpu_list);

	if (result)
		tegra_hv_vcpu_yield_remove(pdev);

	return result;
}


static const struct of_device_id tegra_hv_vcpu_yield_match[] = {
	{ .compatible = "nvidia,tegra-hv-vcpu-yield", },
	{},
};
MODULE_DEVICE_TABLE(of, tegra_hv_vcpu_yield_match);

static struct platform_driver tegra_hv_vcpu_yield_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_hv_vcpu_yield_match),
	},
	.probe = tegra_hv_vcpu_yield_probe,
	.remove = tegra_hv_vcpu_yield_remove,
};
module_platform_driver(tegra_hv_vcpu_yield_driver);


MODULE_DESCRIPTION("Timed VCPU Yield driver");
MODULE_AUTHOR("Suresh Venkatachalam <skathirampat@nvidia.com>");
MODULE_LICENSE("GPL v2");
