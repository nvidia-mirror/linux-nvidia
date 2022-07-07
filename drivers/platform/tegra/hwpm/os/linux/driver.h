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

#ifndef TEGRA_HWPM_OS_LINUX_DRIVER_H
#define TEGRA_HWPM_OS_LINUX_DRIVER_H

struct hwpm_ip_register_list {
	struct tegra_soc_hwpm_ip_ops ip_ops;
	struct hwpm_ip_register_list *next;
};
extern struct hwpm_ip_register_list *ip_register_list_head;

#endif /* TEGRA_HWPM_OS_LINUX_DRIVER_H */
