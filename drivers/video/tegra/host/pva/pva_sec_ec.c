// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023, NVIDIA Corporation.  All rights reserved.
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

#include <linux/nvhost.h>
#include "pva_regs.h"
#include "pva.h"

static u32 pva_get_sec_ec_addrs(u32 index)
{
	u32 sec_ec_miss_addrs[] = {
		sec_ec_errslice0_missionerr_enable_r(),
		sec_ec_errslice0_latenterr_enable_r(),
		sec_ec_errslice1_missionerr_enable_r(),
		sec_ec_errslice1_latenterr_enable_r(),
		sec_ec_errslice2_missionerr_enable_r(),
		sec_ec_errslice2_latenterr_enable_r(),
		sec_ec_errslice3_missionerr_enable_r(),
		sec_ec_errslice3_latenterr_enable_r()
	};

	return sec_ec_miss_addrs[index];
};

void pva_disable_ec_err_reporting(struct pva *pva)
{

	u32 n_regs = (pva->version != PVA_HW_GEN1) ? 8 : 4;
	u32 i;

	/* save current state */
	for (i = 0; i < n_regs; i++)
		pva->ec_state[i] = host1x_readl(pva->pdev,
						pva_get_sec_ec_addrs(i));

	/* disable reporting */
	for (i = 0; i < n_regs; i++)
		host1x_writel(pva->pdev, pva_get_sec_ec_addrs(i), 0);
}

void pva_enable_ec_err_reporting(struct pva *pva)
{

	u32 n_regs = (pva->version != PVA_HW_GEN1) ? 8 : 4;
	u32 i;

	/* enable reporting */
	for (i = 0; i < n_regs; i++)
		host1x_writel(pva->pdev,
			      pva_get_sec_ec_addrs(i),
			      pva->ec_state[i]);
}
