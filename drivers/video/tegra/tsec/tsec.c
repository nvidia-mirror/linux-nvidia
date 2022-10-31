// SPDX-License-Identifier: GPL-2.0-only
/*
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

#include "tsec_linux.h"
#include "tsec.h"
#include "tsec_boot.h"
#include "tsec_regs.h"

/*
 * TSEC Device Data
 */

struct tsec_device_data t23x_tsec_data = {
	.rate = {192000000, 0, 204000000},
	.riscv_desc_bin		= "tegra23x/nvhost_tsec_desc.fw",
	.riscv_image_bin	= "tegra23x/nvhost_tsec_riscv.fw",
};

struct tsec_device_data t239_tsec_data = {
	.rate = {192000000, 0, 204000000},
	.riscv_desc_bin		= "tegra239/nvhost_tsec_desc.fw",
	.riscv_image_bin	= "tegra239/nvhost_tsec_riscv.fw",
};

/*
 * TSEC Register Access APIs
 */

void tsec_writel(struct tsec_device_data *pdata, u32 r, u32 v)
{
	void __iomem *addr = pdata->reg_aperture + r;

	writel(v, addr);
}

u32 tsec_readl(struct tsec_device_data *pdata, u32 r)
{
	void __iomem *addr = pdata->reg_aperture + r;

	return readl(addr);
}

/*
 * TSEC helpers for clock, reset and register initialisation
 */

static int tsec_enable_clks(struct tsec_device_data *pdata)
{
	int err = 0, index = 0;

	for (index = 0; index < TSEC_NUM_OF_CLKS; index++) {
		err = clk_prepare_enable(pdata->clk[index]);
		if (err) {
			err = -EINVAL;
			goto out;
		}
	}

out:
	return err;
}

static void tsec_disable_clk(struct tsec_device_data *pdata)
{
	int index = 0;

	for (index = 0; index < TSEC_NUM_OF_CLKS; index++)
		clk_disable_unprepare(pdata->clk[index]);
}

static void tsec_deassert_reset(struct tsec_device_data *pdata)
{
	reset_control_acquire(pdata->reset_control);
	reset_control_reset(pdata->reset_control);
	reset_control_release(pdata->reset_control);
}

static void tsec_set_streamid_regs(struct device *dev,
	struct tsec_device_data *pdata)
{
	struct iommu_fwspec *fwspec;
	int streamid;

	/* Get the StreamID value */
	fwspec = dev_iommu_fwspec_get(dev);
	if (fwspec && fwspec->num_ids)
		streamid = fwspec->ids[0] & 0xffff;
	else
		streamid = 0x7F; /* bypass hwid */

	/* Update the StreamID value */
	tsec_writel(pdata, tsec_thi_streamid0_r(), streamid);
	tsec_writel(pdata, tsec_thi_streamid1_r(), streamid);
}

static void tsec_set_cg_regs(struct tsec_device_data *pdata)
{
	tsec_writel(pdata, tsec_priv_blocker_ctrl_cg1_r(), 0x0);
	tsec_writel(pdata, tsec_riscv_cg_r(), 0x3);
}

/*
 * TSEC Power Management Operations
 */

int tsec_poweron(struct device *dev)
{
	struct tsec_device_data *pdata;
	int err = 0, tsec_clks_enabled = 0;

	pdata = dev_get_drvdata(dev);

	err = tsec_enable_clks(pdata);
	if (err) {
		dev_err(dev, "Cannot enable tsec clocks %d\n", err);
		goto out;
	}
	tsec_clks_enabled = 1;

	tsec_deassert_reset(pdata);
	tsec_set_cg_regs(pdata);
	tsec_set_streamid_regs(dev, pdata);

	err = tsec_finalize_poweron(to_platform_device(dev));
	/* Failed to start the device */
	if (err) {
		dev_err(dev, "tsec_finalize_poweron error %d\n", err);
		goto out;
	}

	pdata->power_on = true;

out:
	if (err && tsec_clks_enabled)
		tsec_disable_clk(pdata);
	return err;
}

int tsec_poweroff(struct device *dev)
{
	struct tsec_device_data *pdata;

	pdata = dev_get_drvdata(dev);

	if (pdata->power_on)
		tsec_prepare_poweroff(to_platform_device(dev));

	tsec_disable_clk(pdata);

	pdata->power_on = false;

	return 0;
}

static int tsec_module_suspend(struct device *dev)
{
	return tsec_poweroff(dev);
}

static int tsec_module_resume(struct device *dev)
{
	return tsec_poweron(dev);
}

/*
 * TSEC Probe/Remove and Module Init
 */

static int tsec_module_init(struct platform_device *dev)
{
	struct tsec_device_data *pdata = platform_get_drvdata(dev);
	struct resource *res = NULL;
	void __iomem *regs = NULL;

	/* Initialize dma parameters */
	dma_set_mask_and_coherent(&dev->dev, DMA_BIT_MASK(39));
	dev->dev.dma_parms = &pdata->dma_parms;
	dma_set_max_seg_size(&dev->dev, UINT_MAX);

	/* Get register aperture */
	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	regs = devm_ioremap_resource(&dev->dev, res);
	if (IS_ERR(regs)) {
		int err = PTR_ERR(regs);

		dev_err(&dev->dev, "failed to get register memory %d\n", err);
		return err;
	}
	pdata->reg_aperture = regs;

	/* Get interrupt */
	pdata->irq = platform_get_irq(dev, 0);
	if (pdata->irq < 0) {
		dev_err(&dev->dev, "failed to get irq %d\n", -pdata->irq);
		return -ENXIO;
	}

	/* get TSEC_CLK and enable it */
	pdata->clk[TSEC_CLK_INDEX] = devm_clk_get(&dev->dev, TSEC_CLK_NAME);
	if (IS_ERR(pdata->clk[TSEC_CLK_INDEX])) {
		dev_err(&dev->dev, "failed to get %s clk", TSEC_CLK_NAME);
		return -ENXIO;
	}
	clk_set_rate(pdata->clk[TSEC_CLK_INDEX],
		clk_round_rate(pdata->clk[TSEC_CLK_INDEX],
		pdata->rate[TSEC_CLK_INDEX]));
	clk_prepare_enable(pdata->clk[TSEC_CLK_INDEX]);

	/* get EFUSE_CLK and enable it */
	pdata->clk[EFUSE_CLK_INDEX] = devm_clk_get(&dev->dev, EFUSE_CLK_NAME);
	if (IS_ERR(pdata->clk[EFUSE_CLK_INDEX])) {
		dev_err(&dev->dev, "failed to get %s clk", EFUSE_CLK_NAME);
		clk_disable_unprepare(pdata->clk[TSEC_CLK_INDEX]);
		return -ENXIO;
	}
	clk_set_rate(pdata->clk[EFUSE_CLK_INDEX],
		clk_round_rate(pdata->clk[EFUSE_CLK_INDEX],
		pdata->rate[EFUSE_CLK_INDEX]));
	clk_prepare_enable(pdata->clk[EFUSE_CLK_INDEX]);

	/* get TSEC_PKA_CLK and enable it */
	pdata->clk[TSEC_PKA_CLK_INDEX] = devm_clk_get(&dev->dev, TSEC_PKA_CLK_NAME);
	if (IS_ERR(pdata->clk[TSEC_PKA_CLK_INDEX])) {
		dev_err(&dev->dev, "failed to get %s clk", TSEC_PKA_CLK_NAME);
		clk_disable_unprepare(pdata->clk[EFUSE_CLK_INDEX]);
		clk_disable_unprepare(pdata->clk[TSEC_CLK_INDEX]);
		return -ENXIO;
	}
	clk_set_rate(pdata->clk[TSEC_PKA_CLK_INDEX],
		clk_round_rate(pdata->clk[TSEC_PKA_CLK_INDEX],
		pdata->rate[TSEC_PKA_CLK_INDEX]));
	clk_prepare_enable(pdata->clk[TSEC_PKA_CLK_INDEX]);

	/* get reset_control and reset the module */
	pdata->reset_control = devm_reset_control_get_exclusive_released(
					&dev->dev, NULL);
	if (IS_ERR(pdata->reset_control))
		pdata->reset_control = NULL;
	tsec_deassert_reset(pdata);

	/* disable the clocks after resetting the module */
	tsec_disable_clk(pdata);

	return 0;
}


const struct dev_pm_ops tsec_module_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tsec_module_suspend, tsec_module_resume)
};

static const struct of_device_id tsec_of_match[] = {
	{ .compatible = "nvidia,tegra234-tsec",
		.data = (struct tsec_device_data *)&t23x_tsec_data },
	{ .compatible = "nvidia,tegra239-tsec",
		.data = (struct nvhost_device_data *)&t239_tsec_data },
	{ },
};

static int tsec_probe(struct platform_device *dev)
{
	int err;
	struct tsec_device_data *pdata = NULL;

	/* Get device platform data */
	if (dev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tsec_of_match, &dev->dev);
		if (match)
			pdata = (struct tsec_device_data *)match->data;
	} else {
		pdata = (struct tsec_device_data *)dev->dev.platform_data;
	}
	pdata->pdev = dev;
	platform_set_drvdata(dev, pdata);

	err = tsec_module_init(dev);
	if (err) {
		dev_err(&dev->dev, "error %d in tsec_module_init\n", err);
		return err;
	}

	return tsec_kickoff_boot(dev);
}

static int tsec_remove(struct platform_device *dev)
{
	return tsec_poweroff(&dev->dev);
}

struct platform_driver tsec_driver = {
	.probe = tsec_probe,
	.remove = tsec_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "tsec",
		.pm = &tsec_module_pm_ops,
		.of_match_table = tsec_of_match,
	}
};

module_platform_driver(tsec_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nikesh Oswal <noswal@nvidia.com>");
MODULE_DEVICE_TABLE(of, tsec_of_match);
MODULE_DESCRIPTION("TSEC Driver");
