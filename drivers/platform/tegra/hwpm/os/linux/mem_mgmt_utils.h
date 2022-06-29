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

#ifndef TEGRA_HWPM_OS_LINUX_MEM_MGMT_UTILS_H
#define TEGRA_HWPM_OS_LINUX_MEM_MGMT_UTILS_H

struct tegra_soc_hwpm;
struct tegra_soc_hwpm_alloc_pma_stream;
struct tegra_soc_hwpm_update_get_put;

int tegra_hwpm_map_stream_buffer(struct tegra_soc_hwpm *hwpm,
	struct tegra_soc_hwpm_alloc_pma_stream *alloc_pma_stream);
int tegra_hwpm_clear_mem_pipeline(struct tegra_soc_hwpm *hwpm);
int tegra_hwpm_update_mem_bytes(struct tegra_soc_hwpm *hwpm,
	struct tegra_soc_hwpm_update_get_put *update_get_put);
int tegra_hwpm_map_update_allowlist(struct tegra_soc_hwpm *hwpm,
	void *ioctl_struct);

#endif /* TEGRA_HWPM_OS_LINUX_MEM_MGMT_UTILS_H */
