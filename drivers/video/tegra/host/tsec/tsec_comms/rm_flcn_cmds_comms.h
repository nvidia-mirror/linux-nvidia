/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * RM FLCN CMDS COMMS
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

#ifndef RM_FLCN_CMDS_COMMS_H
#define RM_FLCN_CMDS_COMMS_H

struct RM_FLCN_QUEUE_HDR {
	u8      unitId;
	u8      size;
	u8      ctrlFlags;
	u8      seqNumId;
};

#define RM_FLCN_QUEUE_HDR_SIZE          sizeof(struct RM_FLCN_QUEUE_HDR)

#define RM_DPU_LOG_QUEUE_NUM    2
#define RM_GSP_UNIT_REWIND      (0x00)
#define RM_GSP_UNIT_INIT        (0x02)
#define RM_GSP_UNIT_HDCP22WIRED (0x06)
#define RM_GSP_UNIT_END         (0x11)

#define RM_DPU_CMDQ_LOG_ID 0

struct RM_GSP_INIT_MSG_GSP_INIT {
	u8      msgType;
	u8      numQueues;
	u16     osDebugEntryPoint;
	struct {
		u32     queueOffset;
		u16     queueSize;
		u8      queuePhyId;
		u8      queueLogId;
	} qInfo[RM_DPU_LOG_QUEUE_NUM];
	u32     rsvd1;
	u8      rsvd2;
	u8      status;
};

#endif
