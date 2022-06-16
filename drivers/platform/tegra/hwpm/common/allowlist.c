/*
 * Copyright (c) 2021-2022, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <tegra_hwpm.h>
#include <tegra_hwpm_log.h>
#include <tegra_hwpm_kmem.h>
#include <tegra_hwpm_common.h>
#include <tegra_hwpm_mem_mgmt.h>
#include <tegra_hwpm_static_analysis.h>

int tegra_hwpm_get_allowlist_size(struct tegra_soc_hwpm *hwpm)
{
	int ret = 0;

	hwpm->alist_map->full_alist_size = 0ULL;

	tegra_hwpm_fn(hwpm, " ");

	ret = tegra_hwpm_func_all_ip(hwpm, NULL, TEGRA_HWPM_GET_ALIST_SIZE);
	if (ret != 0) {
		tegra_hwpm_err(hwpm, "get_alist_size failed");
		return ret;
	}

	return 0;
}

int tegra_hwpm_combine_alist(struct tegra_soc_hwpm *hwpm, u64 *alist)
{
	struct tegra_hwpm_func_args func_args;
	int err = 0;

	tegra_hwpm_fn(hwpm, " ");

	func_args.alist = alist;
	func_args.full_alist_idx = 0ULL;

	err = tegra_hwpm_func_all_ip(hwpm, &func_args,
		TEGRA_HWPM_COMBINE_ALIST);
	if (err != 0) {
		tegra_hwpm_err(hwpm, "combine alist failed");
		return err;
	}

	/* Check size of full alist with hwpm->alist_map->full_alist_size*/
	if (func_args.full_alist_idx != hwpm->alist_map->full_alist_size) {
		tegra_hwpm_err(hwpm, "full_alist_size 0x%llx doesn't match "
			"max full_alist_idx 0x%llx",
			hwpm->alist_map->full_alist_size,
			func_args.full_alist_idx);
		err = -EINVAL;
	}

	return err;
}
