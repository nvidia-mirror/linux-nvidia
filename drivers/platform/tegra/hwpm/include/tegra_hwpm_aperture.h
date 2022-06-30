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

#ifndef TEGRA_HWPM_APERTURE_H
#define TEGRA_HWPM_APERTURE_H

#ifdef __KERNEL__
#include <os/linux/aperture_utils.h>
#else
int tegra_hwpm_perfmon_reserve_impl(struct tegra_soc_hwpm *hwpm,
	struct hwpm_ip_inst *ip_inst, struct hwpm_ip_aperture *perfmon)
{
	return -EINVAL;
}
int tegra_hwpm_perfmux_reserve_impl(struct tegra_soc_hwpm *hwpm,
	struct hwpm_ip_inst *ip_inst, struct hwpm_ip_aperture *perfmux)
{
	return -EINVAL;
}
int tegra_hwpm_perfmon_release_impl(struct tegra_soc_hwpm *hwpm,
	struct hwpm_ip_aperture *perfmon)
{
	return -EINVAL;
}
int tegra_hwpm_perfmux_release_impl(struct tegra_soc_hwpm *hwpm,
	struct hwpm_ip_aperture *perfmux)
{
	return -EINVAL;
}
#endif

#define tegra_hwpm_perfmon_reserve(hwpm, ip_inst, perfmon)	\
	tegra_hwpm_perfmon_reserve_impl(hwpm, ip_inst, perfmon)
#define tegra_hwpm_perfmux_reserve(hwpm, ip_inst, perfmux)	\
	tegra_hwpm_perfmux_reserve_impl(hwpm, ip_inst, perfmux)
#define tegra_hwpm_perfmon_release(hwpm, perfmon)		\
	tegra_hwpm_perfmon_release_impl(hwpm, perfmon)
#define tegra_hwpm_perfmux_release(hwpm, perfmux)		\
	tegra_hwpm_perfmux_release_impl(hwpm, perfmux)

#endif /* TEGRA_HWPM_APERTURE_H */
