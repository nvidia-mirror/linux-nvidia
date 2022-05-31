/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Header file for Tegra FDEDEV
 *
 * Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES.  All rights reserved.
 */

#ifndef __UAPI_TEGRA_FDEDEV_H
#define __UAPI_TEGRA_FDEDEV_H

#include <asm-generic/ioctl.h>

enum tegra_fde_op_mode {
	BLOCK_MODE,
	FRAME_MODE,
};

struct tegra_fde_req {
	__u64 input_size;
	__u64 __user *output_size;
	enum tegra_fde_op_mode op_mode;
	int fd;
	int fd1;
	__u8 __user *input;
	__u8 __user *output;
};


#define TEGRA_IOCTL_GET_FDE	\
		_IOWR(0x98, 111, struct tegra_fde_req)

#endif
