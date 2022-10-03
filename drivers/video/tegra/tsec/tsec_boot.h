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

#ifndef TSEC_BOOT_H
#define TSEC_BOOT_H

#define RISCV_IDLE_TIMEOUT_DEFAULT    100000  /* 100 milliseconds */
#define RISCV_IDLE_TIMEOUT_LONG       2000000 /* 2 seconds */
#define RISCV_IDLE_CHECK_PERIOD       10      /* 10 usec */
#define RISCV_IDLE_CHECK_PERIOD_LONG  1000    /* 1 milliseconds */

/* Image descriptor format */
struct RM_RISCV_UCODE_DESC {
	/*
	 * Version 1
	 * Version 2
	 * Vesrion 3 = for Partition boot
	 * Vesrion 4 = for eb riscv boot
	 */
	u32  version; /* structure version */
	u32  bootloaderOffset;
	u32  bootloaderSize;
	u32  bootloaderParamOffset;
	u32  bootloaderParamSize;
	u32  riscvElfOffset;
	u32  riscvElfSize;
	u32  appVersion; /* Changelist number associated with the image */
	/*
	 * Manifest contains information about Monitor and it is
	 * input to BR
	 */
	u32  manifestOffset;
	u32  manifestSize;
	/*
	 * Monitor Data offset within RISCV image and size
	 */
	u32  monitorDataOffset;
	u32  monitorDataSize;
	/*
	 * Monitor Code offset withtin RISCV image and size
	 */
	u32  monitorCodeOffset;
	u32  monitorCodeSize;
	u32  bIsMonitorEnabled;
	/*
	 * Swbrom Code offset within RISCV image and size
	 */
	u32  swbromCodeOffset;
	u32  swbromCodeSize;
	/*
	 * Swbrom Data offset within RISCV image and size
	 */
	u32  swbromDataOffset;
	u32  swbromDataSize;
};

struct riscv_image_desc {
	u32 manifest_offset;
	u32 manifest_size;
	u32 data_offset;
	u32 data_size;
	u32 code_offset;
	u32 code_size;
};

struct riscv_data {
	bool valid;
	struct riscv_image_desc desc;
	dma_addr_t backdoor_img_iova;
	u32 *backdoor_img_va;
	size_t backdoor_img_size;
};

int tsec_kickoff_boot(struct platform_device *pdev);
int tsec_finalize_poweron(struct platform_device *dev);
int tsec_prepare_poweroff(struct platform_device *dev);

#endif /* TSEC_BOOT_H */
