// SPDX-License-Identifier: GPL-2.0-only
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

#include "tsec_plat.h"
#include "tsec_reg_comms.h"
#include "tsec_comms.h"
#include "rm_flcn_cmds_comms.h"

#define TSEC_POLL_TIME_MS 2000
#define TSEC_TAIL_POLL_TIME 50
#define TSEC_MSG_QUEUE_PORT 0
#define TSEC_EMEM_START 0x1000000
#define TSEC_EMEM_SIZE 0x2000
#define TSEC_CMD_EMEM_SIZE 4
#define DO_IPC_OVER_GSC_CO (1)
/*
 * Uncomment below when RM is running on CCPLEX and GSC has been
 * configured via BCT files to allow access via CCPLEX to GSC-CO
 */
// #define TSEC_RM_ON_DCE (1)

#define MAX_MSG_SIZE (128)

static bool s_init_msg_rcvd;
static u8 s_init_tsec_msg[MAX_MSG_SIZE];

#ifdef DO_IPC_OVER_GSC_CO
static u64 s_ipc_gscco_base;
static u64 s_ipc_gscco_size;
static u64 s_ipc_gscco_page_base;
static u64 s_ipc_gscco_page_size;
static u64 s_ipc_gscco_page_count;
struct TSEC_BOOT_INFO {
	u32 bootFlag;
};
#endif

/* Struct to register client callback function */
struct callback_t {
	callback_func_t cb_func;
	void            *cb_ctx;
};
static struct callback_t s_callbacks[RM_GSP_UNIT_END];

/* gspQueueCmdValidate */
static int validate_cmd(struct RM_FLCN_QUEUE_HDR *flcn_cmd_hdr,  u32 queue_id)
{
	if (flcn_cmd_hdr == NULL)
		return -TSEC_EINVAL;

	if ((flcn_cmd_hdr->size < RM_FLCN_QUEUE_HDR_SIZE) ||
		(queue_id != RM_DPU_CMDQ_LOG_ID) ||
		(flcn_cmd_hdr->unitId >= RM_GSP_UNIT_END)) {
		return -TSEC_EINVAL;
	}

	return 0;
}

#ifdef DO_IPC_OVER_GSC_CO

static int ipc_gscco_txfr(u32 offset, u8 *buff, u32 size, bool read)
{
	u8 *gscCo;
	u32 idx;

	if (offset < TSEC_EMEM_START) {
		plat_print(LVL_ERR, "Invalid Offset %x less than TSEC_EMEM_START\n", offset);
		return -TSEC_EINVAL;
	}

	offset -= TSEC_EMEM_START;

	if (!s_ipc_gscco_base || !s_ipc_gscco_size) {
		plat_print(LVL_ERR, "Invalid IPC GSC-CO address/size\n");
		return -TSEC_EINVAL;
	}
	if (!buff || !size) {
		plat_print(LVL_ERR, "Invalid client buf/size\n");
		return -TSEC_EINVAL;
	}
	if (offset  > s_ipc_gscco_size || ((offset + size) > s_ipc_gscco_size)) {
		plat_print(LVL_ERR, "Client buf beyond IPC GSC-CO limits\n");
		return -TSEC_EINVAL;
	}

	gscCo = (u8 *)(s_ipc_gscco_base + offset);
	if (read) {
		for (idx = 0; idx < size; idx++)
			buff[idx] = gscCo[idx];
	} else {
		for (idx = 0; idx < size; idx++)
			gscCo[idx] = buff[idx];
	}

	return 0;
}

#else

static int emem_transfer(
	u32 dmem_addr, u8 *buff, u32 size, u8 port, bool copy_from)
{
	u32     num_words;
	u32     num_bytes;
	u32    *pData = (u32 *)buff;
	u32     reg32;
	u32     i;
	u32     ememc_offset = tsec_ememc_r(port);
	u32     ememd_offset = tsec_ememd_r(port);
	u32     emem_start = TSEC_EMEM_START;
	u32     emem_end = TSEC_EMEM_START + TSEC_EMEM_SIZE;

	if (!size || (port >= TSEC_CMD_EMEM_SIZE))
		return -TSEC_EINVAL;

	if ((dmem_addr < emem_start) || ((dmem_addr + size) > emem_end)) {
		plat_print(LVL_ERR, "CMD: FAILED: copy must be in EMEM aperature [0x%x, 0x%x)\n",
			emem_start, emem_end);
		return -TSEC_EINVAL;
	}

	dmem_addr -= emem_start;

	num_words = size >> 2;
	num_bytes = size & 0x3; /* MASK_BITS(2); */

	/* (DRF_SHIFTMASK(NV_PGSP_EMEMC_OFFS) |
	 * DRF_SHIFTMASK(NV_PGSP_EMEMC_BLK));
	 */
	reg32 = dmem_addr & 0x00007ffc;

	if (copy_from) {
		/* PSEC_EMEMC EMEMC_AINCR enable
		 * indicate auto increment on read
		 */
		reg32 = reg32 | 0x02000000;
	} else {
		/* PSEC_EMEMC EMEMC_AINCW enable
		 * mark auto-increment on write
		 */
		reg32 = reg32 | 0x01000000;
	}

	plat_tsec_reg_write(ememc_offset, reg32);

	for (i = 0; i < num_words; i++) {
		if (copy_from)
			pData[i] = plat_tsec_reg_read(ememd_offset);
		else
			plat_tsec_reg_write(ememd_offset, pData[i]);
	}

	/* Check if there are leftover bytes to copy */
	if (num_bytes > 0) {
		u32 bytes_copied = num_words << 2;

		/* Read the contents first. If we're copying to the EMEM,
		 * we've set autoincrement on write,
		 * so reading does not modify the pointer.
		 * We can, thus, do a read/modify/write without needing
		 * to worry about the pointer having moved forward.
		 * There is no special explanation needed
		 * if we're copying from the EMEM since this is the last
		 * access to HW in that case.
		 */
		reg32 = plat_tsec_reg_read(ememd_offset);
		if (copy_from) {
			for (i = 0; i < num_bytes; i++)
				buff[bytes_copied + i] = ((u8 *)&reg32)[i];
		} else {
			for (i = 0; i < num_bytes; i++)
				((u8 *)&reg32)[i] = buff[bytes_copied + i];

			plat_tsec_reg_write(ememd_offset, reg32);
		}
	}

	return 0;
}

#endif

static int ipc_write(u32 head, u8 *pSrc, u32 num_bytes, u32 port)
{
#ifdef DO_IPC_OVER_GSC_CO
	return ipc_gscco_txfr(head, pSrc, num_bytes, false);
#else
	return emem_transfer(head, pSrc, num_bytes, port, false);
#endif
}

#ifdef TSEC_RM_ON_DCE
static int ipc_read(u32 tail, u8 *pdst, u32 num_bytes, u32 port)
{
#ifdef DO_IPC_OVER_GSC_CO
	return ipc_gscco_txfr(tail, pdst, num_bytes, true);
#else
	return emem_transfer(tail, pdst, num_bytes, port, true);
#endif
}
#endif

#ifdef DO_IPC_OVER_GSC_CO
#define TSEC_BOOT_POLL_TIME_US     (100000)
#define TSEC_BOOT_POLL_INTERVAL_US (50)
#define TSEC_BOOT_POLL_COUNT       (TSEC_BOOT_POLL_TIME_US / TSEC_BOOT_POLL_INTERVAL_US)
#define TSEC_BOOT_FLAG_MAGIC       (0xA5A5A5A5)
#endif

#ifdef DO_IPC_OVER_GSC_CO
static u32 tsec_get_boot_flag(void)
{
	struct TSEC_BOOT_INFO *bootInfo = (struct TSEC_BOOT_INFO *)(s_ipc_gscco_base);

	if (!s_ipc_gscco_base || !s_ipc_gscco_size) {
		plat_print(LVL_ERR, "%s: Invalid GSC-CO address/size\n", __func__);
		return 0;
	} else {
		return bootInfo->bootFlag;
	}
}

#ifdef TSEC_RM_ON_DCE
static void tsec_reset_boot_flag(void)
{
	struct TSEC_BOOT_INFO *bootInfo = (struct TSEC_BOOT_INFO *)(s_ipc_gscco_base);

	if (!s_ipc_gscco_base || !s_ipc_gscco_size)
		plat_print(LVL_ERR, "%s: Invalid GSC-CO address/size\n", __func__);
	else
		bootInfo->bootFlag = 0;
}
#endif
#endif

static void invoke_init_cb(void *unused)
{
	callback_func_t cb_func;
	void *cb_ctx;

	plat_acquire_comms_mutex();
	cb_func = s_callbacks[RM_GSP_UNIT_INIT].cb_func;
	cb_ctx  = s_callbacks[RM_GSP_UNIT_INIT].cb_ctx;
	s_callbacks[RM_GSP_UNIT_INIT].cb_func = NULL;
	s_callbacks[RM_GSP_UNIT_INIT].cb_ctx  = NULL;
	plat_release_comms_mutex();

	if (cb_func)
		cb_func(cb_ctx, (void *)s_init_tsec_msg);
}

void tsec_drain_msg(bool invoke_cb)
{
#ifndef TSEC_RM_ON_DCE
	return;
#else
	int i;
	u32 tail = 0;
	u32 head = 0;
	u32 queue_id = 0;
	u32 msgq_head_base;
	u32 msgq_tail_base;
	u32 msgq_head_stride;
	u32 msgq_tail_stride;
	static u32 msgq_start;
	struct RM_FLCN_QUEUE_HDR *gsp_hdr;
	struct RM_GSP_INIT_MSG_GSP_INIT *gsp_init_msg;
	struct RM_FLCN_QUEUE_HDR *init_gsp_hdr;
	struct RM_GSP_INIT_MSG_GSP_INIT *init_gsp_init_msg;
	callback_func_t cb_func;
	void *cb_ctx;
	u8 tsec_msg[MAX_MSG_SIZE];

	msgq_head_base = tsec_msgq_head_r(TSEC_MSG_QUEUE_PORT);
	msgq_tail_base = tsec_msgq_tail_r(TSEC_MSG_QUEUE_PORT);

	msgq_head_stride = tsec_msgq_head_r(1) - tsec_msgq_head_r(0);
	msgq_tail_stride = tsec_msgq_tail_r(1) - tsec_msgq_tail_r(0);

	gsp_hdr =  (struct RM_FLCN_QUEUE_HDR *)(tsec_msg);
	gsp_init_msg = (struct RM_GSP_INIT_MSG_GSP_INIT *) (tsec_msg + RM_FLCN_QUEUE_HDR_SIZE);

	init_gsp_hdr = (struct RM_FLCN_QUEUE_HDR *)(s_init_tsec_msg);
	init_gsp_init_msg = (struct RM_GSP_INIT_MSG_GSP_INIT *)
		(s_init_tsec_msg + RM_FLCN_QUEUE_HDR_SIZE);

	for (i = 0; !msgq_start && i < TSEC_POLL_TIME_MS; i++) {
		msgq_start = plat_tsec_reg_read((msgq_tail_base +
						 (msgq_tail_stride * queue_id)));
		if (!msgq_start)
			plat_udelay(TSEC_TAIL_POLL_TIME);
	}

	if (!msgq_start)
		plat_print(LVL_WARN, "msgq_start=0x%x\n", msgq_start);

	for (i = 0; i < TSEC_POLL_TIME_MS; i++) {
		tail = plat_tsec_reg_read((msgq_tail_base +
					   (msgq_tail_stride * queue_id)));
		head = plat_tsec_reg_read((msgq_head_base +
					   (msgq_head_stride * queue_id)));
		if (tail != head)
			break;
		plat_udelay(TSEC_TAIL_POLL_TIME);
	}

	if (head == 0 || tail == 0) {
		plat_print(LVL_ERR, "Err: Invalid MSGQ head=0x%x, tail=0x%x\n",
			head, tail);
		goto EXIT;
	}

	if (tail == head) {
		plat_print(LVL_DBG, "Empty MSGQ tail(0x%x): 0x%x head(0x%x): 0x%x\n",
			(msgq_tail_base + (msgq_tail_stride * queue_id)),
			tail,
			(msgq_head_base + (msgq_head_stride * queue_id)),
			head);
		goto EXIT;
	}

	while (tail != head) {

		/* read header */
		ipc_read(tail, tsec_msg,
			RM_FLCN_QUEUE_HDR_SIZE, 0);
		plat_print(LVL_ERR, "seqNumId=%d\n", gsp_hdr->seqNumId);
		/* copy msg body */
		if (gsp_hdr->size > RM_FLCN_QUEUE_HDR_SIZE) {
			ipc_read(tail + RM_FLCN_QUEUE_HDR_SIZE, tsec_msg + RM_FLCN_QUEUE_HDR_SIZE,
				gsp_hdr->size - RM_FLCN_QUEUE_HDR_SIZE, 0);
		}

		if (gsp_hdr->unitId == RM_GSP_UNIT_INIT) {
			plat_print(LVL_DBG, "MSGQ: %s(%d) init msg\n",
				__func__, __LINE__);
			if (gsp_init_msg->numQueues < 2) {
				plat_print(LVL_ERR, "MSGQ: Initing less queues than expected %d\n",
					gsp_init_msg->numQueues);
				goto FAIL;
			}
#ifdef DO_IPC_OVER_GSC_CO
			/* DCE can access the GSC-CO hence can poll for the Tsec
			 * booted flag and also reset it
			 */
			for (i = 0; i < TSEC_BOOT_POLL_COUNT; i++) {
				if (tsec_get_boot_flag() == TSEC_BOOT_FLAG_MAGIC)
					break;
				plat_udelay(TSEC_BOOT_POLL_INTERVAL_US);
			}
			if (i >= TSEC_BOOT_POLL_COUNT) {
				plat_print(LVL_ERR, "Tsec GSC-CO Boot Flag not set\n");
				goto FAIL;
			} else {
				tsec_reset_boot_flag();
				plat_print(LVL_DBG, "Tsec GSC-CO Boot Flag reset done\n");
			}
#endif
			memcpy(init_gsp_hdr, gsp_hdr, RM_FLCN_QUEUE_HDR_SIZE);
			memcpy(init_gsp_init_msg, gsp_init_msg,
				gsp_hdr->size - RM_FLCN_QUEUE_HDR_SIZE);

			/* Invoke the callback and clear it */
			plat_acquire_comms_mutex();
			s_init_msg_rcvd = true;
			if (invoke_cb) {
				cb_func = s_callbacks[gsp_hdr->unitId].cb_func;
				cb_ctx  = s_callbacks[gsp_hdr->unitId].cb_ctx;
				s_callbacks[gsp_hdr->unitId].cb_func = NULL;
				s_callbacks[gsp_hdr->unitId].cb_ctx = NULL;
			}
			plat_release_comms_mutex();
			if (cb_func && invoke_cb)
				cb_func(cb_ctx, (void *)tsec_msg);
		} else if (gsp_hdr->unitId < RM_GSP_UNIT_END) {
			plat_print(LVL_DBG, "Msg received from unit 0x%x\n", gsp_hdr->unitId);

			if (gsp_hdr->unitId == RM_GSP_UNIT_HDCP22WIRED) {
				plat_print(LVL_DBG, "MSGQ: %s(%d) RM_GSP_UNIT_HDCP22WIRED\n",
					__func__, __LINE__);
			} else if (gsp_hdr->unitId == RM_GSP_UNIT_REWIND) {
				tail = msgq_start;
				plat_tsec_reg_write((msgq_tail_base +
						     (msgq_tail_stride * queue_id)), tail);
				head = plat_tsec_reg_read(
					(msgq_head_base + (msgq_head_stride * queue_id)));
				plat_print(LVL_DBG, "MSGQ tail rewinded\n");
				continue;
			} else {
				plat_print(LVL_DBG, "MSGQ: %s(%d) what msg could it be 0x%x?\n",
					__func__, __LINE__, gsp_hdr->unitId);
			}

			/* Invoke the callback and clear it */
			if (invoke_cb) {
				plat_acquire_comms_mutex();
				cb_func = s_callbacks[gsp_hdr->unitId].cb_func;
				cb_ctx  = s_callbacks[gsp_hdr->unitId].cb_ctx;
				s_callbacks[gsp_hdr->unitId].cb_func = NULL;
				s_callbacks[gsp_hdr->unitId].cb_ctx = NULL;
				plat_release_comms_mutex();
				if (cb_func)
					cb_func(cb_ctx, (void *)tsec_msg);
			}
		} else {
			plat_print(LVL_ERR, "MSGQ: %s(%d) invalid msg uintId 0x%x?\n",
				__func__, __LINE__, gsp_hdr->unitId);
		}

FAIL:
		tail += ALIGN(gsp_hdr->size, 4);
		head = plat_tsec_reg_read(
			(msgq_head_base + (msgq_head_stride * queue_id)));
		plat_tsec_reg_write(
			(msgq_tail_base + (msgq_tail_stride * queue_id)), tail);
	}

EXIT:
	return;

#endif /* TSEC_RM_ON_DCE */
}

#ifdef DO_IPC_OVER_GSC_CO
void tsec_comms_init(u64 ipc_co_va, u64 ipc_co_va_size)
{
	/* Set IPC CO Info before enabling Msg Interrupts from TSEC to CCPLEX */
	s_ipc_gscco_base = ipc_co_va;
	s_ipc_gscco_size = ipc_co_va_size;

	s_ipc_gscco_page_size = (64 * 1024);

	/* First Page Reserved */
	if (s_ipc_gscco_size > s_ipc_gscco_page_size) {
		s_ipc_gscco_page_count = (s_ipc_gscco_size -
			s_ipc_gscco_page_size) / s_ipc_gscco_page_size;
	} else {
		s_ipc_gscco_page_count = 0;
	}
	s_ipc_gscco_page_base = s_ipc_gscco_page_count ?
		s_ipc_gscco_base + s_ipc_gscco_page_size : 0;
}
#endif

void *nvhost_tsec_get_gscco_page(u32 page_number, u32 *gscco_offset)
{
#ifdef DO_IPC_OVER_GSC_CO
	u8 *page_va;

	if (!s_ipc_gscco_page_base || (page_number >= s_ipc_gscco_page_count)) {
		plat_print(LVL_ERR,
			"%s: No reserved memory for Page %d\n",
			__func__, page_number);
		return NULL;
	}

	page_va = (u8 *)s_ipc_gscco_page_base;
	page_va += (page_number * s_ipc_gscco_page_size);
	if (gscco_offset) {
		*gscco_offset =
			(u32)((s_ipc_gscco_page_base - s_ipc_gscco_base) +
			(page_number * s_ipc_gscco_page_size));
	}
	return page_va;
#else
	plat_print(LVL_ERR, "%s: IPC over GSC-CO not enabled\n", __func__);
	return NULL;
#endif
}
EXPORT_SYMBOL_COMMS(nvhost_tsec_get_gscco_page);

/*
 * cmd - Falcon command
 * queue_id - ID of queue (usually 0)
 * callback_func - callback func to caller on command completion
 *
 */
int nvhost_tsec_send_cmd(void *cmd, u32 queue_id,
	callback_func_t cb_func, void *cb_ctx)
{
	int i;
	int placeholder;
	u32 head;
	u32 tail;
	u8  cmd_size;
	u32 cmd_size_aligned;
	u32 cmdq_head_base;
	u32 cmdq_tail_base;
	u32 cmdq_head_stride;
	u32 cmdq_tail_stride;
	u32 cmdq_size = 0x80;
	static u32 cmdq_start;
	struct RM_FLCN_QUEUE_HDR *flcn_cmd_hdr;
	struct RM_FLCN_QUEUE_HDR hdr;

	if (!s_init_msg_rcvd) {
		plat_print(LVL_ERR, "TSEC RISCV hasn't booted successfully\n");
		return -TSEC_ENODEV;
	}

	cmdq_head_base = tsec_cmdq_head_r(0);
	cmdq_head_stride = tsec_cmdq_head_r(1) - tsec_cmdq_head_r(0);
	cmdq_tail_base = tsec_cmdq_tail_r(0);
	cmdq_tail_stride = tsec_cmdq_tail_r(1) - tsec_cmdq_tail_r(0);

	for (i = 0; !cmdq_start && i < TSEC_POLL_TIME_MS; i++) {
		cmdq_start = plat_tsec_reg_read((cmdq_tail_base +
						(queue_id * cmdq_tail_stride)));
		if (!cmdq_start)
			plat_udelay(TSEC_TAIL_POLL_TIME);
	}

	if (!cmdq_start) {
		plat_print(LVL_WARN, "cmdq_start=0x%x\n", cmdq_start);
		return -TSEC_ENODEV;
	}

	if (validate_cmd(cmd, queue_id != 0)) {
		plat_print(LVL_DBG, "CMD: %s: %d Invalid command\n",
			__func__, __LINE__);
		return -TSEC_EINVAL;
	}

	flcn_cmd_hdr = (struct RM_FLCN_QUEUE_HDR *)cmd;
	plat_acquire_comms_mutex();
	if (s_callbacks[flcn_cmd_hdr->unitId].cb_func) {
		plat_release_comms_mutex();
		plat_print(LVL_ERR, "CMD: %s: %d More than 1 outstanding cmd for unit 0x%x\n",
			__func__, __LINE__, flcn_cmd_hdr->unitId);
		return -TSEC_EINVAL;
	}
	plat_release_comms_mutex();
	cmd_size = flcn_cmd_hdr->size;
	placeholder = ALIGN(cmd_size, 4);
	if (placeholder < 0) {
		plat_print(LVL_ERR, "Alignment found to be negative\n");
		return -TSEC_EINVAL;
	}
	cmd_size_aligned = (unsigned int) placeholder;
	head = plat_tsec_reg_read(
		(cmdq_head_base + (queue_id * cmdq_head_stride)));

check_space:
	tail = plat_tsec_reg_read((cmdq_tail_base +
				   (queue_id * cmdq_tail_stride)));
	if (head < cmdq_start || tail < cmdq_start)
		plat_print(LVL_ERR, "***** head/tail invalid, h=0x%x,t=0x%x\n", head, tail);
	if (UINT_MAX - head < cmd_size_aligned) {
		pr_err("addition of head and offset wraps\n");
		return -EINVAL;
	}
	if (tail > head) {
		if ((head + cmd_size_aligned) < tail)
			goto enqueue;
		plat_udelay(TSEC_TAIL_POLL_TIME);
		goto check_space;
	} else {

		if ((head + cmd_size_aligned) < (cmdq_start + cmdq_size)) {
			goto enqueue;
		} else {
			if ((cmdq_start + cmd_size_aligned) < tail) {
				goto rewind;
			} else {
				plat_udelay(TSEC_TAIL_POLL_TIME);
				goto check_space;
			}
		}
	}

rewind:
	hdr.unitId = RM_GSP_UNIT_REWIND;
	hdr.size = RM_FLCN_QUEUE_HDR_SIZE;
	hdr.ctrlFlags = 0;
	hdr.seqNumId = 0;
	if (ipc_write(head, (u8 *)&hdr, hdr.size, 0))
		return -TSEC_EINVAL;
	head = cmdq_start;
	plat_tsec_reg_write((cmdq_head_base +
			     (queue_id * cmdq_head_stride)), head);
	plat_print(LVL_DBG, "CMDQ: rewind h=%x,t=%x\n", head, tail);

enqueue:
	plat_acquire_comms_mutex();
	s_callbacks[flcn_cmd_hdr->unitId].cb_func = cb_func;
	s_callbacks[flcn_cmd_hdr->unitId].cb_ctx  = cb_ctx;
	plat_release_comms_mutex();
	if (ipc_write(head, (u8 *)cmd, cmd_size, 0)) {
		plat_acquire_comms_mutex();
		s_callbacks[flcn_cmd_hdr->unitId].cb_func = NULL;
		s_callbacks[flcn_cmd_hdr->unitId].cb_ctx  = NULL;
		plat_release_comms_mutex();
		return -TSEC_EINVAL;
	}
	head += cmd_size_aligned;
	plat_tsec_reg_write((cmdq_head_base +
		(queue_id * cmdq_head_stride)), head);

	plat_print(LVL_DBG, "Cmd sent to unit 0x%x\n", flcn_cmd_hdr->unitId);

	return 0;
}
EXPORT_SYMBOL_COMMS(nvhost_tsec_send_cmd);

int nvhost_tsec_set_init_cb(callback_func_t cb_func, void *cb_ctx)
{
	int err = 0;

	plat_acquire_comms_mutex();

	if (s_callbacks[RM_GSP_UNIT_INIT].cb_func) {
		plat_print(LVL_ERR, "%s: %d: INIT unit cb_func already set\n",
			__func__, __LINE__);
		err = -TSEC_EINVAL;
		goto FAIL;
	}
	if (!cb_func) {
		plat_print(LVL_ERR, "%s: %d: Init CallBack NULL\n",
			__func__, __LINE__);
		err = -TSEC_EINVAL;
		goto FAIL;
	}

	s_callbacks[RM_GSP_UNIT_INIT].cb_func = cb_func;
	s_callbacks[RM_GSP_UNIT_INIT].cb_ctx = cb_ctx;

	if (s_init_msg_rcvd) {
		plat_print(LVL_DBG, "Init msg already received invoking callback\n");
		plat_queue_work(invoke_init_cb, NULL);
	}
#ifdef DO_IPC_OVER_GSC_CO
	else if (tsec_get_boot_flag() == TSEC_BOOT_FLAG_MAGIC) {
		plat_print(LVL_DBG, "Doorbell missed tsec booted first, invoke init callback\n");
		/* Interrupt missed as tsec booted first
		 * Explicitly call drain_msg
		 */
		plat_release_comms_mutex();
		tsec_drain_msg(false);
		plat_acquire_comms_mutex();
		/* Init message is drained now, hence queue the work item to invoke init callback*/
		plat_queue_work(invoke_init_cb, NULL);
	}
#endif

FAIL:
	plat_release_comms_mutex();
	return err;
}
EXPORT_SYMBOL_COMMS(nvhost_tsec_set_init_cb);
