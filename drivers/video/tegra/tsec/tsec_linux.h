/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *
 * Tegra TSEC Module Support
 *
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef TSEC_LINUX_H
#define TSEC_LINUX_H

#include <linux/types.h>                   /* for types like u8, u32 etc */
#include <linux/platform_device.h>         /* for platform_device */
#include <linux/of_platform.h>             /* for of_match_device etc */
#include <linux/slab.h>                    /* for kzalloc */
#include <linux/delay.h>                   /* for udelay */
#include <linux/clk.h>                     /* for clk_prepare_enable */
#include <linux/reset.h>                   /* for reset_control_reset */
#include <linux/iommu.h>                   /* for dev_iommu_fwspec_get */
#include <linux/iopoll.h>                  /* for readl_poll_timeout */
#include <linux/dma-mapping.h>             /* for dma_map_page_attrs */
#include <linux/pm.h>                      /* for dev_pm_ops */
#include <linux/version.h>                 /* for KERNEL_VERSION */
#include <linux/interrupt.h>               /* for enable_irq */
#include <linux/firmware.h>                /* for request_firmware */
#include <asm/cacheflush.h>                /* for __flush_dcache_area */
#if (KERNEL_VERSION(5, 14, 0) <= LINUX_VERSION_CODE)
#include <soc/tegra/mc.h>                  /* for tegra_mc_get_carveout_info */
#else
#include <linux/platform/tegra/tegra_mc.h> /* for mc_get_carveout_info */
#endif

#endif /* TSEC_LINUX_H */
