/*
 * Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef TEGRA_HWPM_TIMERS_H
#define TEGRA_HWPM_TIMERS_H

#include <tegra_hwpm_types.h>

#ifdef __KERNEL__
#include <os/linux/timers.h>
#else
int tegra_hwpm_timeout_init_impl(struct tegra_hwpm *hwpm,
	struct tegra_hwpm_timeout *timeout, u32 retries)
{
	return -EINVAL;
}

int tegra_hwpm_timeout_expired_impl(struct tegra_hwpm *hwpm,
	struct tegra_hwpm_timeout *timeout)
{
	return -EINVAL;
}

void tegra_hwpm_msleep_impl(unsigned int msecs)
{
	return -EINVAL;
}
#endif

#define tegra_hwpm_timeout_init(hwpm, timeout, retries)	\
	tegra_hwpm_timeout_init_impl(hwpm, timeout, retries)
#define tegra_hwpm_timeout_expired(hwpm, timeout)	\
	tegra_hwpm_timeout_expired_impl(hwpm, timeout)
#define tegra_hwpm_msleep(msecs)	\
	tegra_hwpm_msleep_impl(msecs)

#endif /* TEGRA_HWPM_TIMERS_H */
