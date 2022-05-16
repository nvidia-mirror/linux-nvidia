/*
 * Copyright (c) 2009-2022, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/io.h>
#include <linux/syscore_ops.h>
#include <linux/suspend.h>
#include <linux/serial_reg.h>
#include <linux/tegra-pm.h>
#include <linux/of_address.h>
#include <linux/ioport.h>

static u64 resume_time;
static u64 resume_entry_time;
static u64 suspend_time;
static u64 suspend_entry_time;

void tegra_log_suspend_entry_time(void)
{
        suspend_entry_time = arch_timer_read_counter();
}
EXPORT_SYMBOL(tegra_log_suspend_entry_time);

void tegra_log_resume_time(void)
{
	u32 timer_rate = arch_timer_get_rate()/1000;

        resume_time = arch_timer_read_counter() - resume_entry_time;
	resume_time = resume_time / timer_rate;
}
EXPORT_SYMBOL(tegra_log_resume_time);

static void tegra_log_suspend_time(void)
{
	u32 timer_rate = arch_timer_get_rate()/1000;

        suspend_time = arch_timer_read_counter() - suspend_entry_time;
	suspend_time = suspend_time / timer_rate;
}

static int tegra_sc7_timestamp_suspend(void)
{
	tegra_log_suspend_time();
	pr_info("Entered SC7\n");

	return 0;
}

static void tegra_sc7_timestamp_resume(void)
{
	resume_entry_time = arch_timer_read_counter();
	pr_info("Exited SC7\n");
}

static struct syscore_ops tegra_sc7_timestamp_syscore_ops = {
	.suspend = tegra_sc7_timestamp_suspend,
	.resume = tegra_sc7_timestamp_resume,
};

static ssize_t resume_time_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%llums\n", resume_time);
}

static struct kobj_attribute resume_time_attribute =
	__ATTR(resume_time, 0444, resume_time_show, NULL);

static ssize_t suspend_time_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%llums\n", suspend_time);
}

static struct kobj_attribute suspend_time_attribute =
	__ATTR(suspend_time, 0444, suspend_time_show, NULL);

static struct kobject *suspend_kobj;

static int __init suspend_resume_time_init(void)
{
	suspend_kobj = kobject_create_and_add("suspend", power_kobj);
	if (suspend_kobj) {
		if (sysfs_create_file(suspend_kobj,
					&resume_time_attribute.attr))
			pr_err("%s: sysfs_create_file resume_time failed!\n",
								__func__);
		if (sysfs_create_file(suspend_kobj,
					&suspend_time_attribute.attr))
			pr_err("%s: sysfs_create_file suspend_time failed!\n",
								__func__);
	}

	register_syscore_ops(&tegra_sc7_timestamp_syscore_ops);

	return 0;
}
late_initcall(suspend_resume_time_init);
