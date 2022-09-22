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

#ifndef TEGRA_HWPM_COMMON_H
#define TEGRA_HWPM_COMMON_H

#include <tegra_hwpm_types.h>

enum tegra_hwpm_funcs;
enum hwpm_aperture_type;
enum tegra_hwpm_element_type;
struct tegra_hwpm_func_args;
struct tegra_hwpm_ip_ops;
struct tegra_soc_hwpm;
struct hwpm_ip;
struct hwpm_ip_inst;
struct hwpm_ip_aperture;

int tegra_hwpm_init_sw_components(struct tegra_soc_hwpm *hwpm,
	u32 chip_id, u32 chip_id_rev);

int tegra_hwpm_func_all_ip(struct tegra_soc_hwpm *hwpm,
	struct tegra_hwpm_func_args *func_args,
	enum tegra_hwpm_funcs iia_func);
int tegra_hwpm_func_single_ip(struct tegra_soc_hwpm *hwpm,
	struct tegra_hwpm_func_args *func_args,
	enum tegra_hwpm_funcs iia_func, u32 ip_idx);

bool tegra_hwpm_aperture_for_address(struct tegra_soc_hwpm *hwpm,
	enum tegra_hwpm_funcs iia_func,
	u64 find_addr, u32 *ip_idx, u32 *inst_idx, u32 *element_idx,
	enum tegra_hwpm_element_type *element_type);

int tegra_hwpm_perfmux_disable(struct tegra_soc_hwpm *hwpm,
	struct hwpm_ip_aperture *perfmux);
int tegra_hwpm_reserve_rtr(struct tegra_soc_hwpm *hwpm);
int tegra_hwpm_release_rtr(struct tegra_soc_hwpm *hwpm);
int tegra_hwpm_reserve_resource(struct tegra_soc_hwpm *hwpm, u32 resource);
int tegra_hwpm_release_resources(struct tegra_soc_hwpm *hwpm);
int tegra_hwpm_bind_resources(struct tegra_soc_hwpm *hwpm);

int tegra_hwpm_element_reserve(struct tegra_soc_hwpm *hwpm,
	struct hwpm_ip_inst *ip_inst, struct hwpm_ip_aperture *perfmon);
int tegra_hwpm_element_release(struct tegra_soc_hwpm *hwpm,
	struct hwpm_ip_aperture *perfmon);

int tegra_hwpm_set_fs_info_ip_ops(struct tegra_soc_hwpm *hwpm,
	struct tegra_hwpm_ip_ops *ip_ops,
	u64 base_address, u32 ip_idx, bool available);
int tegra_hwpm_get_fs_info(struct tegra_soc_hwpm *hwpm,
	u32 ip_enum, u64 *fs_mask, u8 *ip_status);
int tegra_hwpm_get_resource_info(struct tegra_soc_hwpm *hwpm,
	u32 resource_enum, u8 *status);
int tegra_hwpm_finalize_chip_info(struct tegra_soc_hwpm *hwpm);
int tegra_hwpm_ip_handle_power_mgmt(struct tegra_soc_hwpm *hwpm,
	struct hwpm_ip_inst *ip_inst, bool disable);

int tegra_hwpm_alloc_alist_map(struct tegra_soc_hwpm *hwpm);
int tegra_hwpm_get_allowlist_size(struct tegra_soc_hwpm *hwpm);
int tegra_hwpm_combine_alist(struct tegra_soc_hwpm *hwpm, u64 *alist);
size_t tegra_hwpm_get_alist_buf_size(struct tegra_soc_hwpm *hwpm);
int tegra_hwpm_zero_alist_regs(struct tegra_soc_hwpm *hwpm,
	struct hwpm_ip_inst *ip_inst, struct hwpm_ip_aperture *aperture);
int tegra_hwpm_copy_alist(struct tegra_soc_hwpm *hwpm,
	struct hwpm_ip_aperture *aperture, u64 *full_alist,
	u64 *full_alist_idx);
bool tegra_hwpm_check_alist(struct tegra_soc_hwpm *hwpm,
	struct hwpm_ip_aperture *aperture, u64 phys_addr);

int tegra_hwpm_setup_hw(struct tegra_soc_hwpm *hwpm);
int tegra_hwpm_setup_sw(struct tegra_soc_hwpm *hwpm);
int tegra_hwpm_disable_triggers(struct tegra_soc_hwpm *hwpm);
int tegra_hwpm_release_hw(struct tegra_soc_hwpm *hwpm);
void tegra_hwpm_release_sw_setup(struct tegra_soc_hwpm *hwpm);

#endif /* TEGRA_HWPM_COMMON_H */
