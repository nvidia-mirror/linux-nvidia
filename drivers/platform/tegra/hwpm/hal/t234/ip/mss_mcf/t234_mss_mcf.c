/*
 * Copyright (c) 2021-2023, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "t234_mss_mcf.h"

#include <tegra_hwpm.h>
#include <hal/t234/t234_regops_allowlist.h>
#include <hal/t234/t234_perfmon_device_index.h>
#include <hal/t234/hw/t234_addr_map_soc_hwpm.h>

static struct hwpm_ip_aperture t234_mss_mcf_inst0_perfmon_element_static_array[
	T234_HWPM_IP_MSS_MCF_NUM_PERFMON_PER_INST] = {
	{
		.element_type = HWPM_ELEMENT_PERFMON,
		.element_index_mask = BIT(0),
		.element_index = 0U,
		.dt_mmio = NULL,
		.name = "perfmon_mssmcfclient0",
		.device_index = T234_MSSMCFCLIENT0_PERFMON_DEVICE_NODE_INDEX,
		.start_abs_pa = addr_map_rpg_pm_mcf0_base_r(),
		.end_abs_pa = addr_map_rpg_pm_mcf0_limit_r(),
		.start_pa = addr_map_rpg_pm_mcf0_base_r(),
		.end_pa = addr_map_rpg_pm_mcf0_limit_r(),
		.base_pa = addr_map_rpg_pm_base_r(),
		.alist = t234_perfmon_alist,
		.alist_size = ARRAY_SIZE(t234_perfmon_alist),
		.fake_registers = NULL,
	},
	{
		.element_type = HWPM_ELEMENT_PERFMON,
		.element_index_mask = BIT(0),
		.element_index = 1U,
		.dt_mmio = NULL,
		.name = "perfmon_mssmcfmem0",
		.device_index = T234_MSSMCFMEM0_PERFMON_DEVICE_NODE_INDEX,
		.start_abs_pa = addr_map_rpg_pm_mcf1_base_r(),
		.end_abs_pa = addr_map_rpg_pm_mcf1_limit_r(),
		.start_pa = addr_map_rpg_pm_mcf1_base_r(),
		.end_pa = addr_map_rpg_pm_mcf1_limit_r(),
		.base_pa = addr_map_rpg_pm_base_r(),
		.alist = t234_perfmon_alist,
		.alist_size = ARRAY_SIZE(t234_perfmon_alist),
		.fake_registers = NULL,
	},
	{
		.element_type = HWPM_ELEMENT_PERFMON,
		.element_index_mask = BIT(0),
		.element_index = 2U,
		.dt_mmio = NULL,
		.name = "perfmon_mssmcfmem1",
		.device_index = T234_MSSMCFMEM1_PERFMON_DEVICE_NODE_INDEX,
		.start_abs_pa = addr_map_rpg_pm_mcf2_base_r(),
		.end_abs_pa = addr_map_rpg_pm_mcf2_limit_r(),
		.start_pa = addr_map_rpg_pm_mcf2_base_r(),
		.end_pa = addr_map_rpg_pm_mcf2_limit_r(),
		.base_pa = addr_map_rpg_pm_base_r(),
		.alist = t234_perfmon_alist,
		.alist_size = ARRAY_SIZE(t234_perfmon_alist),
		.fake_registers = NULL,
	},
};

static struct hwpm_ip_aperture t234_mss_mcf_inst0_perfmux_element_static_array[
	T234_HWPM_IP_MSS_MCF_NUM_PERFMUX_PER_INST] = {
	{
		.element_type = IP_ELEMENT_PERFMUX,
		.element_index_mask = BIT(0),
		.element_index = 1U,
		.dt_mmio = NULL,
		.name = {'\0'},
		.start_abs_pa = addr_map_mc0_base_r(),
		.end_abs_pa = addr_map_mc0_limit_r(),
		.start_pa = addr_map_mc0_base_r(),
		.end_pa = addr_map_mc0_limit_r(),
		.base_pa = 0ULL,
		.alist = t234_mc0to1_mss_mcf_alist,
		.alist_size = ARRAY_SIZE(t234_mc0to1_mss_mcf_alist),
		.fake_registers = NULL,
	},
	{
		.element_type = IP_ELEMENT_PERFMUX,
		.element_index_mask = BIT(1),
		.element_index = 2U,
		.dt_mmio = NULL,
		.name = {'\0'},
		.start_abs_pa = addr_map_mc1_base_r(),
		.end_abs_pa = addr_map_mc1_limit_r(),
		.start_pa = addr_map_mc1_base_r(),
		.end_pa = addr_map_mc1_limit_r(),
		.base_pa = 0ULL,
		.alist = t234_mc0to1_mss_mcf_alist,
		.alist_size = ARRAY_SIZE(t234_mc0to1_mss_mcf_alist),
		.fake_registers = NULL,
	},
	{
		.element_type = IP_ELEMENT_PERFMUX,
		.element_index_mask = BIT(2),
		.element_index = 3U,
		.dt_mmio = NULL,
		.name = {'\0'},
		.start_abs_pa = addr_map_mc2_base_r(),
		.end_abs_pa = addr_map_mc2_limit_r(),
		.start_pa = addr_map_mc2_base_r(),
		.end_pa = addr_map_mc2_limit_r(),
		.base_pa = 0ULL,
		.alist = t234_mc2to7_mss_mcf_alist,
		.alist_size = ARRAY_SIZE(t234_mc2to7_mss_mcf_alist),
		.fake_registers = NULL,
	},
	{
		.element_type = IP_ELEMENT_PERFMUX,
		.element_index_mask = BIT(3),
		.element_index = 4U,
		.dt_mmio = NULL,
		.name = {'\0'},
		.start_abs_pa = addr_map_mc3_base_r(),
		.end_abs_pa = addr_map_mc3_limit_r(),
		.start_pa = addr_map_mc3_base_r(),
		.end_pa = addr_map_mc3_limit_r(),
		.base_pa = 0ULL,
		.alist = t234_mc2to7_mss_mcf_alist,
		.alist_size = ARRAY_SIZE(t234_mc2to7_mss_mcf_alist),
		.fake_registers = NULL,
	},
	{
		.element_type = IP_ELEMENT_PERFMUX,
		.element_index_mask = BIT(4),
		.element_index = 5U,
		.dt_mmio = NULL,
		.name = {'\0'},
		.start_abs_pa = addr_map_mc4_base_r(),
		.end_abs_pa = addr_map_mc4_limit_r(),
		.start_pa = addr_map_mc4_base_r(),
		.end_pa = addr_map_mc4_limit_r(),
		.base_pa = 0ULL,
		.alist = t234_mc2to7_mss_mcf_alist,
		.alist_size = ARRAY_SIZE(t234_mc2to7_mss_mcf_alist),
		.fake_registers = NULL,
	},
	{
		.element_type = IP_ELEMENT_PERFMUX,
		.element_index_mask = BIT(5),
		.element_index = 6U,
		.dt_mmio = NULL,
		.name = {'\0'},
		.start_abs_pa = addr_map_mc5_base_r(),
		.end_abs_pa = addr_map_mc5_limit_r(),
		.start_pa = addr_map_mc5_base_r(),
		.end_pa = addr_map_mc5_limit_r(),
		.base_pa = 0ULL,
		.alist = t234_mc2to7_mss_mcf_alist,
		.alist_size = ARRAY_SIZE(t234_mc2to7_mss_mcf_alist),
		.fake_registers = NULL,
	},
	{
		.element_type = IP_ELEMENT_PERFMUX,
		.element_index_mask = BIT(6),
		.element_index = 7U,
		.dt_mmio = NULL,
		.name = {'\0'},
		.start_abs_pa = addr_map_mc6_base_r(),
		.end_abs_pa = addr_map_mc6_limit_r(),
		.start_pa = addr_map_mc6_base_r(),
		.end_pa = addr_map_mc6_limit_r(),
		.base_pa = 0ULL,
		.alist = t234_mc2to7_mss_mcf_alist,
		.alist_size = ARRAY_SIZE(t234_mc2to7_mss_mcf_alist),
		.fake_registers = NULL,
	},
	{
		.element_type = IP_ELEMENT_PERFMUX,
		.element_index_mask = BIT(7),
		.element_index = 8U,
		.dt_mmio = NULL,
		.name = {'\0'},
		.start_abs_pa = addr_map_mc7_base_r(),
		.end_abs_pa = addr_map_mc7_limit_r(),
		.start_pa = addr_map_mc7_base_r(),
		.end_pa = addr_map_mc7_limit_r(),
		.base_pa = 0ULL,
		.alist = t234_mc2to7_mss_mcf_alist,
		.alist_size = ARRAY_SIZE(t234_mc2to7_mss_mcf_alist),
		.fake_registers = NULL,
	},
};

static struct hwpm_ip_aperture t234_mss_mcf_inst0_broadcast_element_static_array[
	T234_HWPM_IP_MSS_MCF_NUM_BROADCAST_PER_INST] = {
	{
		.element_type = IP_ELEMENT_BROADCAST,
		.element_index_mask = BIT(0),
		.element_index = 0U,
		.dt_mmio = NULL,
		.name = {'\0'},
		.start_abs_pa = addr_map_mcb_base_r(),
		.end_abs_pa = addr_map_mcb_limit_r(),
		.start_pa = addr_map_mcb_base_r(),
		.end_pa = addr_map_mcb_limit_r(),
		.base_pa = 0ULL,
		.alist = t234_mcb_mss_mcf_alist,
		.alist_size = ARRAY_SIZE(t234_mcb_mss_mcf_alist),
		.fake_registers = NULL,
	},
};

/* IP instance array */
static struct hwpm_ip_inst t234_mss_mcf_inst_static_array[
	T234_HWPM_IP_MSS_MCF_NUM_INSTANCES] = {
	{
		.hw_inst_mask = BIT(0),
		.num_core_elements_per_inst =
			T234_HWPM_IP_MSS_MCF_NUM_CORE_ELEMENT_PER_INST,
		.element_info = {
			/*
			 * Instance info corresponding to
			 * TEGRA_HWPM_APERTURE_TYPE_PERFMUX
			 */
			{
				.num_element_per_inst =
					T234_HWPM_IP_MSS_MCF_NUM_PERFMUX_PER_INST,
				.element_static_array =
					t234_mss_mcf_inst0_perfmux_element_static_array,
				/* NOTE: range should be in ascending order */
				.range_start = addr_map_mc4_base_r(),
				.range_end = addr_map_mc3_limit_r(),
				.element_stride = addr_map_mc4_limit_r() -
					addr_map_mc4_base_r() + 1ULL,
				.element_slots = 0U,
				.element_arr = NULL,
			},
			/*
			 * Instance info corresponding to
			 * TEGRA_HWPM_APERTURE_TYPE_BROADCAST
			 */
			{
				.num_element_per_inst =
					T234_HWPM_IP_MSS_MCF_NUM_BROADCAST_PER_INST,
				.element_static_array =
					t234_mss_mcf_inst0_broadcast_element_static_array,
				.range_start = addr_map_mcb_base_r(),
				.range_end = addr_map_mcb_limit_r(),
				.element_stride = addr_map_mcb_limit_r() -
					addr_map_mcb_base_r() + 1ULL,
				.element_slots = 0U,
				.element_arr = NULL,
			},
			/*
			 * Instance info corresponding to
			 * TEGRA_HWPM_APERTURE_TYPE_PERFMON
			 */
			{
				.num_element_per_inst =
					T234_HWPM_IP_MSS_MCF_NUM_PERFMON_PER_INST,
				.element_static_array =
					t234_mss_mcf_inst0_perfmon_element_static_array,
				.range_start = addr_map_rpg_pm_mcf0_base_r(),
				.range_end = addr_map_rpg_pm_mcf2_limit_r(),
				.element_stride = addr_map_rpg_pm_mcf0_limit_r() -
					addr_map_rpg_pm_mcf0_base_r() + 1ULL,
				.element_slots = 0U,
				.element_arr = NULL,
			},
		},

		.ip_ops = {
			.ip_dev = NULL,
			.hwpm_ip_pm = NULL,
			.hwpm_ip_reg_op = NULL,
		},

		.element_fs_mask = 0U,
	},
};

/* IP structure */
struct hwpm_ip t234_hwpm_ip_mss_mcf = {
	.num_instances = T234_HWPM_IP_MSS_MCF_NUM_INSTANCES,
	.ip_inst_static_array = t234_mss_mcf_inst_static_array,

	.inst_aperture_info = {
		/*
		 * Instance info corresponding to
		 * TEGRA_HWPM_APERTURE_TYPE_PERFMUX
		 */
		{
			/* NOTE: range should be in ascending order */
			.range_start = addr_map_mc4_base_r(),
			.range_end = addr_map_mc3_limit_r(),
			.inst_stride = addr_map_mc3_limit_r() -
				addr_map_mc4_base_r() + 1ULL,
			.inst_slots = 0U,
			.inst_arr = NULL,
		},
		/*
		 * Instance info corresponding to
		 * TEGRA_HWPM_APERTURE_TYPE_BROADCAST
		 */
		{
			.range_start = addr_map_mcb_base_r(),
			.range_end = addr_map_mcb_limit_r(),
			.inst_stride = addr_map_mcb_limit_r() -
				addr_map_mcb_base_r() + 1ULL,
			.inst_slots = 0U,
			.inst_arr = NULL,
		},
		/*
		 * Instance info corresponding to
		 * TEGRA_HWPM_APERTURE_TYPE_PERFMON
		 */
		{
			.range_start = addr_map_rpg_pm_mcf0_base_r(),
			.range_end = addr_map_rpg_pm_mcf2_limit_r(),
			.inst_stride = addr_map_rpg_pm_mcf2_limit_r() -
				addr_map_rpg_pm_mcf0_base_r() + 1ULL,
			.inst_slots = 0U,
			.inst_arr = NULL,
		},
	},

	.dependent_fuse_mask = TEGRA_HWPM_FUSE_HWPM_GLOBAL_DISABLE_MASK,
	.override_enable = false,
	.inst_fs_mask = 0U,
	.resource_status = TEGRA_HWPM_RESOURCE_STATUS_INVALID,
	.reserved = false,
};
