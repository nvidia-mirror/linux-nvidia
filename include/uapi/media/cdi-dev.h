/*
 * Copyright (c) 2020-2022, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __UAPI_CDI_DEV_H__
#define __UAPI_CDI_DEV_H__

#include <linux/ioctl.h>
#include <linux/types.h>

#define CDI_DEV_PKG_FLAG_WR	1

#define CDI_DEV_IOCTL_RW	          _IOW('o', 1, struct cdi_dev_package)
#define CDI_DEV_IOCTL_GET_PWR_INFO    _IOW('o', 2, struct cdi_dev_pwr_ctrl_info)

#define DES_PWR_NVCCP    0U
#define DES_PWR_GPIO     1U
#define DES_PWR_NO_PWR   0xFFU
#define CAM_PWR_NVCCP    0U
#define CAM_PWR_MAX20087 1U
#define CAM_PWR_NO_PWR   0xFFU

#define MAX_POWER_LINKS_PER_BLOCK (4U)

struct __attribute__ ((__packed__)) cdi_dev_pwr_ctrl_info {
	__s8 cam_pwr_method;
	__s8 cam_pwr_i2c_addr;
	__u8 cam_pwr_links[MAX_POWER_LINKS_PER_BLOCK];
};

struct __attribute__ ((__packed__)) cdi_dev_package {
	__u16 offset;
	__u16 offset_len;
	__u32 size;
	__u32 flags;
	unsigned long buffer;
};

#ifdef __KERNEL__
#ifdef CONFIG_COMPAT
#define CDI_DEV_IOCTL_RW32	_IOW('o', 1, struct cdi_dev_package32)

struct __attribute__ ((__packed__)) cdi_dev_package32 {
	__u16 offset;
	__u16 offset_len;
	__u32 size;
	__u32 flags;
	__u32 buffer;
};
#endif /* CONFIG_COMPAT */
#endif /* __KERNEL__ */

#endif  /* __UAPI_CDI_DEV_H__ */
