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

#include "t234_nvdla.h"

#include <tegra_hwpm.h>
#include <hal/t234/t234_regops_allowlist.h>
#include <hal/t234/hw/t234_addr_map_soc_hwpm.h>
#include <hal/t234/t234_perfmon_device_index.h>

static struct hwpm_ip_aperture t234_nvdla_inst0_perfmon_element_static_array[
	T234_HWPM_IP_NVDLA_NUM_PERFMON_PER_INST] = {
	{
		.element_type = HWPM_ELEMENT_PERFMON,
		.element_index_mask = BIT(0),
		.element_index = 0U,
		.dt_mmio = NULL,
		.name = "perfmon_nvdlab0",
		.device_index = T234_NVDLAB0_PERFMON_DEVICE_NODE_INDEX,
		.start_abs_pa = addr_map_rpg_pm_nvdla0_base_r(),
		.end_abs_pa = addr_map_rpg_pm_nvdla0_limit_r(),
		.start_pa = addr_map_rpg_pm_nvdla0_base_r(),
		.end_pa = addr_map_rpg_pm_nvdla0_limit_r(),
		.base_pa = addr_map_rpg_pm_base_r(),
		.alist = t234_perfmon_alist,
		.alist_size = ARRAY_SIZE(t234_perfmon_alist),
		.fake_registers = NULL,
	},
};

static struct hwpm_ip_aperture t234_nvdla_inst1_perfmon_element_static_array[
	T234_HWPM_IP_NVDLA_NUM_PERFMON_PER_INST] = {
	{
		.element_type = HWPM_ELEMENT_PERFMON,
		.element_index_mask = BIT(0),
		.element_index = 0U,
		.dt_mmio = NULL,
		.name = "perfmon_nvdlab1",
		.device_index = T234_NVDLAB1_PERFMON_DEVICE_NODE_INDEX,
		.start_abs_pa = addr_map_rpg_pm_nvdla1_base_r(),
		.end_abs_pa = addr_map_rpg_pm_nvdla1_limit_r(),
		.start_pa = addr_map_rpg_pm_nvdla1_base_r(),
		.end_pa = addr_map_rpg_pm_nvdla1_limit_r(),
		.base_pa = addr_map_rpg_pm_base_r(),
		.alist = t234_perfmon_alist,
		.alist_size = ARRAY_SIZE(t234_perfmon_alist),
		.fake_registers = NULL,
	},
};

static struct hwpm_ip_aperture t234_nvdla_inst0_perfmux_element_static_array[
	T234_HWPM_IP_NVDLA_NUM_PERFMUX_PER_INST] = {
	{
		.element_type = IP_ELEMENT_PERFMUX,
		.element_index_mask = BIT(0),
		.element_index = 0U,
		.dt_mmio = NULL,
		.name = {'\0'},
		.start_abs_pa = addr_map_nvdla0_base_r(),
		.end_abs_pa = addr_map_nvdla0_limit_r(),
		.start_pa = addr_map_nvdla0_base_r(),
		.end_pa = addr_map_nvdla0_limit_r(),
		.base_pa = 0ULL,
		.alist = t234_nvdla_alist,
		.alist_size = ARRAY_SIZE(t234_nvdla_alist),
		.fake_registers = NULL,
	},
};

static struct hwpm_ip_aperture t234_nvdla_inst1_perfmux_element_static_array[
	T234_HWPM_IP_NVDLA_NUM_PERFMUX_PER_INST] = {
	{
		.element_type = IP_ELEMENT_PERFMUX,
		.element_index_mask = BIT(0),
		.element_index = 0U,
		.dt_mmio = NULL,
		.name = {'\0'},
		.start_abs_pa = addr_map_nvdla1_base_r(),
		.end_abs_pa = addr_map_nvdla1_limit_r(),
		.start_pa = addr_map_nvdla1_base_r(),
		.end_pa = addr_map_nvdla1_limit_r(),
		.base_pa = 0ULL,
		.alist = t234_nvdla_alist,
		.alist_size = ARRAY_SIZE(t234_nvdla_alist),
		.fake_registers = NULL,
	},
};

/* IP instance array */
static struct hwpm_ip_inst t234_nvdla_inst_static_array[
	T234_HWPM_IP_NVDLA_NUM_INSTANCES] = {
	{
		.hw_inst_mask = BIT(0),
		.num_core_elements_per_inst =
			T234_HWPM_IP_NVDLA_NUM_CORE_ELEMENT_PER_INST,
		.element_info = {
			/*
			 * Instance info corresponding to
			 * TEGRA_HWPM_APERTURE_TYPE_PERFMUX
			 */
			{
				.num_element_per_inst =
					T234_HWPM_IP_NVDLA_NUM_PERFMUX_PER_INST,
				.element_static_array =
					t234_nvdla_inst0_perfmux_element_static_array,
				.range_start = addr_map_nvdla0_base_r(),
				.range_end = addr_map_nvdla0_limit_r(),
				.element_stride = addr_map_nvdla0_limit_r() -
					addr_map_nvdla0_base_r() + 1ULL,
				.element_slots = 0U,
				.element_arr = NULL,
			},
			/*
			 * Instance info corresponding to
			 * TEGRA_HWPM_APERTURE_TYPE_BROADCAST
			 */
			{
				.num_element_per_inst =
					T234_HWPM_IP_NVDLA_NUM_BROADCAST_PER_INST,
				.element_static_array = NULL,
				.range_start = 0ULL,
				.range_end = 0ULL,
				.element_stride = 0ULL,
				.element_slots = 0U,
				.element_arr = NULL,
			},
			/*
			 * Instance info corresponding to
			 * TEGRA_HWPM_APERTURE_TYPE_PERFMON
			 */
			{
				.num_element_per_inst =
					T234_HWPM_IP_NVDLA_NUM_PERFMON_PER_INST,
				.element_static_array =
					t234_nvdla_inst0_perfmon_element_static_array,
				.range_start = addr_map_rpg_pm_nvdla0_base_r(),
				.range_end = addr_map_rpg_pm_nvdla0_limit_r(),
				.element_stride = addr_map_rpg_pm_nvdla0_limit_r() -
					addr_map_rpg_pm_nvdla0_base_r() + 1ULL,
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
	{
		.hw_inst_mask = BIT(1),
		.num_core_elements_per_inst =
			T234_HWPM_IP_NVDLA_NUM_CORE_ELEMENT_PER_INST,
		.element_info = {
			/*
			 * Instance info corresponding to
			 * TEGRA_HWPM_APERTURE_TYPE_PERFMUX
			 */
			{
				.num_element_per_inst =
					T234_HWPM_IP_NVDLA_NUM_PERFMUX_PER_INST,
				.element_static_array =
					t234_nvdla_inst1_perfmux_element_static_array,
				.range_start = addr_map_nvdla1_base_r(),
				.range_end = addr_map_nvdla1_limit_r(),
				.element_stride = addr_map_nvdla1_limit_r() -
					addr_map_nvdla1_base_r() + 1ULL,
				.element_slots = 0U,
				.element_arr = NULL,
			},
			/*
			 * Instance info corresponding to
			 * TEGRA_HWPM_APERTURE_TYPE_BROADCAST
			 */
			{
				.num_element_per_inst =
					T234_HWPM_IP_NVDLA_NUM_BROADCAST_PER_INST,
				.element_static_array = NULL,
				.range_start = 0ULL,
				.range_end = 0ULL,
				.element_stride = 0ULL,
				.element_slots = 0U,
				.element_arr = NULL,
			},
			/*
			 * Instance info corresponding to
			 * TEGRA_HWPM_APERTURE_TYPE_PERFMON
			 */
			{
				.num_element_per_inst =
					T234_HWPM_IP_NVDLA_NUM_PERFMON_PER_INST,
				.element_static_array =
					t234_nvdla_inst1_perfmon_element_static_array,
				.range_start = addr_map_rpg_pm_nvdla1_base_r(),
				.range_end = addr_map_rpg_pm_nvdla1_limit_r(),
				.element_stride = addr_map_rpg_pm_nvdla1_limit_r() -
					addr_map_rpg_pm_nvdla1_base_r() + 1ULL,
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
struct hwpm_ip t234_hwpm_ip_nvdla = {
	.num_instances = T234_HWPM_IP_NVDLA_NUM_INSTANCES,
	.ip_inst_static_array = t234_nvdla_inst_static_array,

	.inst_aperture_info = {
		/*
		 * Instance info corresponding to
		 * TEGRA_HWPM_APERTURE_TYPE_PERFMUX
		 */
		{
			.range_start = addr_map_nvdla0_base_r(),
			.range_end = addr_map_nvdla1_limit_r(),
			.inst_stride = addr_map_nvdla0_limit_r() -
				addr_map_nvdla0_base_r() + 1ULL,
			.inst_slots = 0U,
			.inst_arr = NULL,
		},
		/*
		 * Instance info corresponding to
		 * TEGRA_HWPM_APERTURE_TYPE_BROADCAST
		 */
		{
			.range_start = 0ULL,
			.range_end = 0ULL,
			.inst_stride = 0ULL,
			.inst_slots = 0U,
			.inst_arr = NULL,
		},
		/*
		 * Instance info corresponding to
		 * TEGRA_HWPM_APERTURE_TYPE_PERFMON
		 */
		{
			.range_start = addr_map_rpg_pm_nvdla0_base_r(),
			.range_end = addr_map_rpg_pm_nvdla1_limit_r(),
			.inst_stride = addr_map_rpg_pm_nvdla0_limit_r() -
				addr_map_rpg_pm_nvdla0_base_r() + 1ULL,
			.inst_slots = 0U,
			.inst_arr = NULL,
		},
	},

	.dependent_fuse_mask = TEGRA_HWPM_FUSE_SECURITY_MODE_MASK |
		TEGRA_HWPM_FUSE_HWPM_GLOBAL_DISABLE_MASK,
	.override_enable = false,
	.inst_fs_mask = 0U,
	.resource_status = TEGRA_HWPM_RESOURCE_STATUS_INVALID,
	.reserved = false,
};
