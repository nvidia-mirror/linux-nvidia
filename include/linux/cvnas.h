/*
 * include/linux/cvnas.h
 *
 * Tegra cvnas driver
 *
 * Copyright (c) 2018-2022, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#ifndef __LINUX_CVNAS_H
#define __LINUX_CVNAS_H

#include <linux/types.h>

#if defined(CONFIG_TEGRA_CVNAS) || defined(CVNAS_MODULE)
int nvcvnas_busy(void);
int nvcvnas_busy_no_rpm(void);
int nvcvnas_idle(void);
int nvcvnas_idle_no_rpm(void);
int is_nvcvnas_probed(void);
phys_addr_t nvcvnas_get_cvsram_base(void);
size_t nvcvnas_get_cvsram_size(void);
int is_nvcvnas_clk_enabled(void);
#else
static inline int nvcvnas_busy(void) { return 0; }
static inline int nvcvnas_busy_no_rpm(void) { return 0; }
static inline int nvcvnas_idle(void) { return 0; }
static inline int nvcvnas_idle_no_rpm(void) { return 0; }
static inline int is_nvcvnas_probed(void) { return 0; }
static inline phys_addr_t nvcvnas_get_cvsram_base(void) { return 0; }
static inline size_t nvcvnas_get_cvsram_size(void) { return 0; }
static inline int is_nvcvnas_clk_enabled(void) { return 0; }
#endif /* CONFIG_TEGRA_CVNAS || CVNAS_MODULE */

#endif /* __LINUX_CVNAS_H */
