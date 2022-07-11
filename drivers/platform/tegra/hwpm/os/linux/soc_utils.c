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

#include <soc/tegra/fuse.h>

#include <tegra_hwpm.h>
#include <tegra_hwpm_soc.h>

bool tegra_hwpm_is_platform_simulation_impl(void)
{
	return tegra_platform_is_vdk();
}

bool tegra_hwpm_is_platform_vsp_impl(void)
{
	return tegra_platform_is_vsp();
}

bool tegra_hwpm_is_platform_silicon_impl(void)
{
	return tegra_platform_is_silicon();
}

bool tegra_hwpm_is_hypervisor_mode_impl(void)
{
	return is_tegra_hypervisor_mode();
}

int tegra_hwpm_fuse_readl_impl(struct tegra_soc_hwpm *hwpm,
	u64 reg_offset, u32 *val)
{
	u32 fuse_val = 0U;
	int err = 0;

	err = tegra_fuse_readl(reg_offset, &fuse_val);
	if (err != 0) {
		return err;
	}

	*val = fuse_val;
	return 0;
}

int tegra_hwpm_fuse_readl_prod_mode_impl(struct tegra_soc_hwpm *hwpm, u32 *val)
{
	return tegra_hwpm_fuse_readl(hwpm, TEGRA_FUSE_PRODUCTION_MODE, val);
}
