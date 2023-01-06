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

#ifndef _PVA_SEC_EC_H_
#define _PVA_SEC_EC_H_

void pva_disable_ec_err_reporting(struct pva *pva);
void pva_enable_ec_err_reporting(struct pva *pva);
#endif
