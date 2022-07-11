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

#ifndef TEGRA_HWPM_SOC_H
#define TEGRA_HWPM_SOC_H

#ifdef __KERNEL__
#include <os/linux/soc_utils.h>
#else
bool tegra_hwpm_is_platform_simulation_impl(void)
{
	return false;
}

bool tegra_hwpm_is_platform_vsp_impl(void)
{
	return false;
}

bool tegra_hwpm_is_platform_silicon_impl(void)
{
	return true;
}

bool tegra_hwpm_is_hypervisor_mode_impl(void)
{
	return false;
}

int tegra_hwpm_fuse_readl_impl(struct tegra_soc_hwpm *hwpm,
	u64 reg_offset, u32 *val)
{
	return -EINVAL;
}

int tegra_hwpm_fuse_readl_prod_mode_impl(struct tegra_soc_hwpm *hwpm, u32 *val)
{
	return -EINVAL;
}

#endif

#define tegra_hwpm_is_platform_simulation()	\
	tegra_hwpm_is_platform_simulation_impl()

#define tegra_hwpm_is_platform_vsp()		\
	tegra_hwpm_is_platform_vsp_impl()

#define tegra_hwpm_is_platform_silicon()	\
	tegra_hwpm_is_platform_silicon_impl()

#define tegra_hwpm_is_hypervisor_mode()		\
	tegra_hwpm_is_hypervisor_mode_impl()

#define tegra_hwpm_fuse_readl(hwpm, reg_offset, val)	\
	tegra_hwpm_fuse_readl_impl(hwpm, reg_offset, val)

#define tegra_hwpm_fuse_readl_prod_mode(hwpm, val)	\
	tegra_hwpm_fuse_readl_prod_mode_impl(hwpm, val)

#endif /* TEGRA_HWPM_SOC_H */
