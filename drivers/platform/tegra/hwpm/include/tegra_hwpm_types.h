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

#ifndef TEGRA_HWPM_TYPES_H
#define TEGRA_HWPM_TYPES_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/err.h>
#include <linux/bug.h>
#include <linux/bits.h>
#if defined(CONFIG_TEGRA_HWPM_OOT)
#include <linux/bitmap.h>
#include <linux/limits.h>
#include <linux/kernel.h>
#endif
#endif

#endif /* TEGRA_HWPM_TYPES_H */
