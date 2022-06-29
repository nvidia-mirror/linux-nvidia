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

#ifndef TEGRA_HWPM_KMEM_H
#define TEGRA_HWPM_KMEM_H

#ifdef __KERNEL__
#include <os/linux/kmem.h>
#else
void *tegra_hwpm_kzalloc_impl(struct tegra_soc_hwpm *hwpm, size_t size)
{
	return NULL;
}

void *tegra_hwpm_kcalloc_impl(struct tegra_soc_hwpm *hwpm, u32 num, size_t size)
{
	return NULL;
}

void tegra_hwpm_kfree_impl(struct tegra_soc_hwpm *hwpm, void *addr)
{
	return;
}
#endif

#define tegra_hwpm_kzalloc(hwpm, size)			\
	tegra_hwpm_kzalloc_impl(hwpm, size)
#define tegra_hwpm_kcalloc(hwpm, num, size)		\
	tegra_hwpm_kcalloc_impl(hwpm, num, size)

#define tegra_hwpm_kfree(hwpm, addr)			\
	tegra_hwpm_kfree_impl(hwpm, addr)


#endif /* TEGRA_HWPM_KMEM_H */
