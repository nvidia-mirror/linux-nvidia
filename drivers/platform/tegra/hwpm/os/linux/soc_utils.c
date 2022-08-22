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

#include <linux/of.h>
#include <soc/tegra/fuse-helper.h>

#include <tegra_hwpm.h>
#include <tegra_hwpm_soc.h>

#if defined(CONFIG_TEGRA_HWPM_OOT)
#if defined(CONFIG_TEGRA_NEXT1_HWPM)
#include <os/linux/next1_soc_utils.h>
#endif
#endif

u32 tegra_hwpm_get_chip_id_impl(void)
{
#if defined(CONFIG_TEGRA_HWPM_OOT)
	if (of_machine_is_compatible("nvidia,tegra234")) {
		return 0x23U;
	}
#ifdef CONFIG_TEGRA_NEXT1_HWPM
		return tegra_hwpm_next1_get_chip_id_impl();
#else
		return 0x0U;
#endif /* CONFIG_TEGRA_NEXT1_HWPM */
#else
	return (u32)tegra_get_chip_id();
#endif /* CONFIG_TEGRA_HWPM_OOT */
}

u32 tegra_hwpm_get_major_rev_impl(void)
{
#if defined(CONFIG_TEGRA_HWPM_OOT)
	if (of_machine_is_compatible("nvidia,tegra234")) {
		return 0x4U;
	}
#ifdef CONFIG_TEGRA_NEXT1_HWPM
		return tegra_hwpm_next1_get_major_rev_impl();
#else
		return 0x0U;
#endif /* CONFIG_TEGRA_NEXT1_HWPM */
#else
	return (u32)tegra_get_major_rev();
#endif
}

u32 tegra_hwpm_chip_get_revision_impl(void)
{
#if defined(CONFIG_TEGRA_HWPM_OOT)
	return 0x0U;
#else
	return (u32)tegra_chip_get_revision();
#endif
}

u32 tegra_hwpm_get_platform_impl(void)
{
#if defined(CONFIG_TEGRA_HWPM_OOT)
	if (of_machine_is_compatible("nvidia,tegra234")) {
		return PLAT_SI;
	}
#ifdef CONFIG_TEGRA_NEXT1_HWPM
		return tegra_hwpm_next1_get_platform_impl();
#else
		return PLAT_INVALID;
#endif /* CONFIG_TEGRA_NEXT1_HWPM */
#else
	return (u32)tegra_get_platform();
#endif
}

bool tegra_hwpm_is_platform_silicon_impl(void)
{
#if defined(CONFIG_TEGRA_HWPM_OOT)
	return tegra_platform_is_silicon();
#else
	return tegra_platform_is_silicon();
#endif
}

bool tegra_hwpm_is_platform_simulation_impl(void)
{
#if defined(CONFIG_TEGRA_HWPM_OOT)
	return tegra_hwpm_get_platform() == PLAT_PRE_SI_VDK;
#else
	return tegra_platform_is_vdk();
#endif
}

bool tegra_hwpm_is_platform_vsp_impl(void)
{
#if defined(CONFIG_TEGRA_HWPM_OOT)
	return tegra_hwpm_get_platform() == PLAT_PRE_SI_VSP;
#else
	return tegra_platform_is_vsp();
#endif
}

bool tegra_hwpm_is_hypervisor_mode_impl(void)
{
#if defined(CONFIG_TEGRA_HWPM_OOT)
	return false;
#else
	return is_tegra_hypervisor_mode();
#endif
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
