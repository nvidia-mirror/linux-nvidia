/* SPDX-License-Identifier: GPL-2.0-only */
/*
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

#ifndef TSEC_PLAT_H
#define TSEC_PLAT_H

#define LVL_INFO (1)
#define LVL_DBG  (2)
#define LVL_WARN (3)
#define LVL_ERR  (4)

#ifdef __KERNEL__

#include <linux/types.h>
#include <linux/device.h>
#include <linux/platform_device.h>

extern struct platform_device *tsec;

#define EXPORT_SYMBOL_COMMS(sym) \
	EXPORT_SYMBOL(sym)

#define TSEC_EINVAL EINVAL
#define TSEC_ENODEV ENODEV

#define plat_print(level, fmt, ...) \
do { \
	if (level == LVL_INFO) \
		dev_info(&tsec->dev, fmt, ##__VA_ARGS__); \
	else if (level == LVL_DBG) \
		dev_dbg(&tsec->dev, fmt, ##__VA_ARGS__); \
	else if (level == LVL_WARN) \
		dev_warn(&tsec->dev, fmt, ##__VA_ARGS__); \
	else if (level == LVL_ERR) \
		dev_err(&tsec->dev, fmt, ##__VA_ARGS__); \
} while (0)

#elif __DCE_KERNEL__

// Functions to be implemented by DCE

#else

// Platform not supported

#endif

typedef void (*work_cb_t)(void *);

/* @brief: API to write a register r with the value specified by v.
 *
 * usage:  Writes a register r with a value specified by v.
 *
 * params[in]:	r register address to write to
 *              v value to write
 */
void plat_tsec_reg_write(u32 r, u32 v);

/* @brief: API to Read a register specified by address r.
 *
 * usage:  Reads a register specified by address r.
 *
 * params[in]:	r register to read from
 *
 * params[out]:	value that is read
 */
uint32_t plat_tsec_reg_read(u32 r);

/* @brief: Adds a delay of usec micro-seconds.
 *
 * usage:  Add a delay
 *
 * params[in]:	usec delay specified in micro-seconds.
 */
void plat_udelay(u64 usec);

/* @brief: The Tsec comms unit needs a comms mutex for its internal
 * synchronization. Tsec driver provides this mutex. This is an API
 * to acquire the mutex.
 *
 * usage:  Called to acquire mutex provided by Tsec driver.
 */
void plat_acquire_comms_mutex(void);

/* @brief: API to release the mutex provided by TSec driver.
 *
 * usage:  Called to release the mutex acquired by
 * plat_acquire_comms_mutex.
 */
void plat_release_comms_mutex(void);

/* @brief: A generic API to queue a work item. This work item
 * will later be scheduled in work queue's thread/task context.
 *
 * usage:  Used for queueing a work item.
 *
 * params[in]:	cb  callback
 *              ctx context
 */
void plat_queue_work(work_cb_t cb, void *ctx);

#endif
