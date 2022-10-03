/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *
 * Tegra TSEC Module Support
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

#ifndef TSEC_H
#define TSEC_H

/*
 * TSEC Device Data Structure
 */

#define TSEC_CLK_NAME        "tsec"
#define TSEC_CLK_INDEX       (0)
#define EFUSE_CLK_NAME       "efuse"
#define EFUSE_CLK_INDEX      (1)
#define TSEC_PKA_CLK_NAME    "tsec_pka"
#define TSEC_PKA_CLK_INDEX   (2)
#define TSEC_NUM_OF_CLKS     (3)

struct tsec_device_data {
	void __iomem *reg_aperture;
	struct device_dma_parameters dma_parms;

	int irq;
	/* spin lock for module irq */
	spinlock_t mirq_lock;

	/* If module is powered on */
	bool power_on;

	struct clk *clk[TSEC_NUM_OF_CLKS];
	long rate[TSEC_NUM_OF_CLKS];
	 /* private platform data */
	void *private_data;
	/* owner platform_device */
	struct platform_device *pdev;

	/* reset control for this device */
	struct reset_control *reset_control;

	/* store the risc-v info */
	void *riscv_data;
	/* name of riscv descriptor binary */
	char *riscv_desc_bin;
	/* name of riscv image binary */
	char *riscv_image_bin;
};

/*
 * TSEC Register Access APIs
 */

void tsec_writel(struct tsec_device_data *pdata, u32 r, u32 v);
u32 tsec_readl(struct tsec_device_data *pdata, u32 r);


/*
 * TSEC power on/off APIs
 */

int tsec_poweron(struct device *dev);
int tsec_poweroff(struct device *dev);

#endif /* TSEC_H */
