/*
 * drivers/video/tegra/host/nvhost_ktime.h
 *
 * Tegra Graphics Host Interrupt Management
 *
 * Copyright (c) 2022, NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_KTIME_H
#define __NVHOST_KTIME_H

#include <linux/version.h>

enum nvhost_clock_id {
	NVHOST_CLOCK_UNKNOWN = 0,
	NVHOST_CLOCK_MONOTONIC,
	NVHOST_CLOCK_PTP
};

struct nvhost_timespec {
	struct timespec64 ts;
	enum nvhost_clock_id clock;
};

#include <linux/nvpps.h>
#define nvhost_ktime_get_ts(nvts)	\
do {					\
	u64 time_ns;			\
	int err = nvpps_get_ptp_ts(&time_ns);		\
	if (err) {					\
		ktime_get_ts64(&(nvts)->ts);			\
		(nvts)->clock = NVHOST_CLOCK_MONOTONIC;	\
	} else {						\
		(nvts)->ts = ns_to_timespec64(time_ns);		\
		(nvts)->clock = NVHOST_CLOCK_PTP;		\
	}							\
} while (0)

#endif
