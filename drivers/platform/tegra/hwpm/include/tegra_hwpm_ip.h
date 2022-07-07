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

#ifndef TEGRA_HWPM_IP_H
#define TEGRA_HWPM_IP_H

#ifdef __KERNEL__
#include <os/linux/ip_utils.h>
#else
int tegra_hwpm_complete_ip_register_impl(struct tegra_soc_hwpm *hwpm)
{
	return -EINVAL;
}
#endif

#define tegra_hwpm_complete_ip_register(hwpm)	\
	tegra_hwpm_complete_ip_register_impl(hwpm)

#endif /* TEGRA_HWPM_IP_H */
