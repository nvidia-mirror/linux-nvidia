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

#ifndef TSEC_COMMS_H
#define TSEC_COMMS_H

typedef void (*callback_func_t)(void *, void *);

/* -------- Tsec driver internal functions to be called by platform dependent code --------- */

/* @brief: Initialises IPC CO and reserves pages on the same.
 *
 * usage: To be called when tsec driver is initialised. Must be
 * called  before any other API is used from the comms lib.
 *
 * params[in]: ipc_co_va       carveout base virtual address
 *             ipc_co_va_size  carveout address space size
 */
void tsec_comms_init(u64 ipc_co_va, u64 ipc_co_va_size);

/* @brief: This function will drain all the messages
 * from the tsec queue. It is called when interrupt is
 * received from TSec. It should be called in threaded
 * context and not interrupt context.
 *
 * usage: To be called when interrupt is received from TSec.
 *
 * params[in]: invoke_cb indicates whether to invoke callback or not.
 */
void tsec_drain_msg(bool invoke_cb);

/* -------- END -------- */

/* -------- Exported functions which are invoked from DisplayRM. -------- */

/* @brief: Sets callback for init message
 *
 * usage: Called for setting callback for init msg
 *
 * params[in]:  cb_func function to be called after init msg is
 *                      received
 *              cb_ctx  pointer to callback context
 *
 * params[out]: return value(0 for success).
 */
int nvhost_tsec_set_init_cb(callback_func_t cb_func, void *cb_ctx);

/* @brief: Send the command upon receiving it by putting it into the
 * tsec queue. Also sets appropriate callback to be called when
 * response arrives.
 *
 * usage: Called when sending a command to tsec.
 *
 * params[in]: cmd      pointer to the memory containing the command
 *             queue_id Id of the queue being used.
 *             cb_func  callback function tobe registered
 *             cb_ctx   pointer to context of the callback function.
 *
 * params[out]: return value(0 for success)
 */
int nvhost_tsec_send_cmd(void *cmd, u32 queue_id,
	callback_func_t cb_func, void *cb_ctx);

/* @brief: Retrieves a page from the carevout memory
 *
 * usage: Called to get a particular page from the carveout.
 *
 * params[in]: page_number  page number
 *             gscco_offset pointer to the offset memory
 */
void *nvhost_tsec_get_gscco_page(u32 page_number, u32 *gscco_offset);

/* -------- END -------- */

#endif
