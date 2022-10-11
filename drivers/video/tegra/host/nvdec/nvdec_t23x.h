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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __NVHOST_NVDEC_T23x_H__
#define __NVHOST_NVDEC_T23x_H__

#include <linux/nvhost.h>

#define MTHD_ADDR_ACTMON_WEIGHT 0xC9U
#define MTHD_ADDR_ACTMON_ACTIVE_MASK 0xCAU
#define MTHD_ADDR_ACTMON_ACTIVE_BORPS 0xCBU

int nvhost_nvdec_finalize_poweron_t23x(struct platform_device *dev);
int nvhost_nvdec_prepare_poweroff_t23x(struct platform_device *dev);

#endif //__NVHOST_NVDEC_T23x_H__
