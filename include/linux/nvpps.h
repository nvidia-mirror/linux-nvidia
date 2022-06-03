/*
 * Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __NVPPS_H__
#define __NVPPS_H__

/*
 * Get PTP time
 * Clients may call the API every and anytime PTP time is needed.
 * If PTP time source is not registered, returns -EINVAL
 */
int nvpps_get_ptp_ts(void *ts);

#endif /* __NVPPS_H__ */
