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

#ifndef TEGRA_HWPM_OS_LINUX_LOG_H
#define TEGRA_HWPM_OS_LINUX_LOG_H

struct tegra_soc_hwpm;

void tegra_hwpm_err_impl(struct tegra_soc_hwpm *hwpm,
	const char *func, int line, const char *fmt, ...);
void tegra_hwpm_dbg_impl(struct tegra_soc_hwpm *hwpm,
	u32 dbg_mask, const char *func, int line, const char *fmt, ...);

#endif /* TEGRA_HWPM_OS_LINUX_LOG_H */
