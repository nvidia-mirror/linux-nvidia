/*
 * Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
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
/*
 * Function naming determines intended use:
 *
 *     <x>_r(void) : Returns the offset for register <x>.
 *
 *     <x>_o(void) : Returns the offset for element <x>.
 *
 *     <x>_w(void) : Returns the word offset for word (4 byte) element <x>.
 *
 *     <x>_<y>_s(void) : Returns size of field <y> of register <x> in bits.
 *
 *     <x>_<y>_f(u32 v) : Returns a value based on 'v' which has been shifted
 *         and masked to place it at field <y> of register <x>.  This value
 *         can be |'d with others to produce a full register value for
 *         register <x>.
 *
 *     <x>_<y>_m(void) : Returns a mask for field <y> of register <x>.  This
 *         value can be ~'d and then &'d to clear the value of field <y> for
 *         register <x>.
 *
 *     <x>_<y>_<z>_f(void) : Returns the constant value <z> after being shifted
 *         to place it at field <y> of register <x>.  This value can be |'d
 *         with others to produce a full register value for <x>.
 *
 *     <x>_<y>_v(u32 r) : Returns the value of field <y> from a full register
 *         <x> value 'r' after being shifted to place its LSB at bit 0.
 *         This value is suitable for direct comparison with other unshifted
 *         values appropriate for use in field <y> of register <x>.
 *
 *     <x>_<y>_<z>_v(void) : Returns the constant value for <z> defined for
 *         field <y> of register <x>.  This value is suitable for direct
 *         comparison with unshifted values appropriate for use in field <y>
 *         of register <x>.
 */

#ifndef HW_TSEC_T23X_H
#define HW_TSEC_T23X_H

#include "tsec_comms/tsec_reg_comms.h"

static inline u32 tsec_riscv_bcr_ctrl_r(void)
{
	/* NV_PSEC_RISCV_BCR_CTRL */
	return 0x2668;
}
static inline u32 tsec_riscv_bcr_ctrl_core_select_riscv_f(void)
{
	return 0x10;
}
static inline u32 tsec_riscv_bcr_dmaaddr_pkcparam_lo_r(void)
{
	/* NV_PSEC_RISCV_BCR_DMAADDR_PKCPARAM_LO */
	return 0x2670;
}
static inline u32 tsec_riscv_bcr_dmaaddr_pkcparam_hi_r(void)
{
	/* NV_PSEC_RISCV_BCR_DMAADDR_PKCPARAM_HI */
	return 0x2674;
}
static inline u32 tsec_riscv_bcr_dmaaddr_fmccode_lo_r(void)
{
	/* NV_PSEC_RISCV_BCR_DMAADDR_FMCCODE_LO */
	return 0x2678;
}
static inline u32 tsec_riscv_bcr_dmaaddr_fmccode_hi_r(void)
{
	/* NV_PSEC_RISCV_BCR_DMAADDR_FMCCODE_HI */
	return 0x267c;
}
static inline u32 tsec_riscv_bcr_dmaaddr_fmcdata_lo_r(void)
{
	/* NV_PSEC_RISCV_BCR_DMAADDR_FMCDATA_LO */
	return 0x2680;
}
static inline u32 tsec_riscv_bcr_dmaaddr_fmcdata_hi_r(void)
{
	/* NV_PSEC_RISCV_BCR_DMAADDR_FMCDATA_HI */
	return 0x2684;
}
static inline u32 tsec_riscv_bcr_dmacfg_r(void)
{
	/* NV_PSEC_RISCV_BCR_DMACFG */
	return 0x266c;
}
static inline u32 tsec_riscv_bcr_dmacfg_target_local_fb_f(void)
{
	return 0x0;
}
static inline u32 tsec_riscv_bcr_dmacfg_lock_locked_f(void)
{
	return 0x80000000;
}
static inline u32 tsec_riscv_bcr_dmacfg_sec_r(void)
{
	/* NV_PSEC_RISCV_BCR_DMACFG_SEC */
	return 0x2694;
}
static inline u32 tsec_riscv_bcr_dmacfg_sec_gscid_f(u32 v)
{
	return (v & 0x1f) << 16;
}
static inline u32 tsec_falcon_mailbox0_r(void)
{
	/* NV_PSEC_FALCON_MAILBOX0 */
	return 0x1040;
}
static inline u32 tsec_falcon_mailbox1_r(void)
{
	/* NV_PSEC_FALCON_MAILBOX1 */
	return 0x1044;
}
static inline u32 tsec_riscv_cpuctl_r(void)
{
	/* NV_PSEC_RISCV_CPUCTL */
	return 0x2388;
}
static inline u32 tsec_riscv_cpuctl_startcpu_true_f(void)
{
	return 0x1;
}
static inline u32 tsec_riscv_cpuctl_active_stat_v(u32 r)
{
	return (r >> 7) & 0x1;
}
static inline u32 tsec_riscv_cpuctl_active_stat_active_v(void)
{
	return 0x00000001;
}
static inline u32 tsec_riscv_br_retcode_r(void)
{
	/* NV_PSEC_RISCV_BR_RETCODE */
	return 0x265c;
}
static inline u32 tsec_riscv_br_retcode_result_v(u32 r)
{
	return (r >> 0) & 0x3;
}
static inline u32 tsec_riscv_br_retcode_result_pass_v(void)
{
	return 0x00000003;
}
static inline u32 tsec_riscv_irqmset_r(void)
{
	return 0x00002520;
}
static inline u32 tsec_riscv_irqmset_wdtmr_set_f(void)
{
	return 0x2;
}
static inline u32 tsec_riscv_irqmset_halt_set_f(void)
{
	return 0x10;
}
static inline u32 tsec_riscv_irqmset_exterr_set_f(void)
{
	return 0x20;
}
static inline u32 tsec_riscv_irqmset_swgen0_set_f(void)
{
	return 0x40;
}
static inline u32 tsec_riscv_irqmset_swgen1_set_f(void)
{
	return 0x80;
}
static inline u32 tsec_riscv_irqmset_ext_f(u32 v)
{
	return (v & 0xff) << 8;
}
static inline u32 tsec_hwcfg_r(void)
{
	return 0x00001abc;
}
static inline u32 tsec_hwcfg1_r(void)
{
	return 0x0000112c;
}
#endif
