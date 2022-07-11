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

#ifndef TEGRA_HWPM_OS_LINUX_SOC_UTILS_H
#define TEGRA_HWPM_OS_LINUX_SOC_UTILS_H

struct tegra_soc_hwpm;

bool tegra_hwpm_is_platform_simulation_impl(void);
bool tegra_hwpm_is_platform_vsp_impl(void);
bool tegra_hwpm_is_platform_silicon_impl(void);
bool tegra_hwpm_is_hypervisor_mode_impl(void);
int tegra_hwpm_fuse_readl_impl(struct tegra_soc_hwpm *hwpm,
	u64 reg_offset, u32 *val);
int tegra_hwpm_fuse_readl_prod_mode_impl(struct tegra_soc_hwpm *hwpm, u32 *val);

#endif /* TEGRA_HWPM_OS_LINUX_SOC_UTILS_H */
