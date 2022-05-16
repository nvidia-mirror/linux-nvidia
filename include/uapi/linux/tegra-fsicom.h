/* SPDX-License-Identifier: (GPL-2.0 WITH Linux-syscall-note)
 *
 * Copyright (c) 2021-2022, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 */

#ifndef _UAPI_TEGRA_FSICOM_H_
#define _UAPI_TEGRA_FSICOM_H_

#include <linux/ioctl.h>

struct rw_data {
	uint32_t handle;
	uint64_t pa;
	uint64_t iova;
};

/* signal value */
#define SIG_FSI_DAEMON 44

/* ioctl call macros */
#define NVMAP_SMMU_MAP    _IOWR('q', 1, struct rw_data *)
#define NVMAP_SMMU_UNMAP  _IOWR('q', 2, struct rw_data *)
#define TEGRA_HSP_WRITE   _IOWR('q', 3, struct rw_data *)
#define TEGRA_SIGNAL_REG  _IOWR('q', 4, struct rw_data *)

#endif	/* _UAPI_TEGRA_FSICOM_H_ */
