/*
 * Copyright (c) 2021-2022, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/reset.h>
#include <linux/clk.h>
#include <linux/dma-buf.h>
#include <linux/debugfs.h>
#include <soc/tegra/fuse.h>

#include <tegra_hwpm.h>
#include <tegra_hwpm_ip.h>
#include <tegra_hwpm_log.h>
#include <tegra_hwpm_soc.h>
#include <tegra_hwpm_kmem.h>
#include <tegra_hwpm_common.h>
#include <os/linux/debugfs.h>
#include <os/linux/driver.h>

static const struct of_device_id tegra_soc_hwpm_of_match[] = {
	{
		.compatible     = "nvidia,t234-soc-hwpm",
	},
#ifdef CONFIG_TEGRA_NEXT1_HWPM
#include <os/linux/next1_driver.h>
#endif
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_soc_hwpm_of_match);

static char *tegra_hwpm_get_devnode(struct device *dev, umode_t *mode)
{
	if (!mode) {
		return NULL;
	}

	/* Allow root:debug ownership */
	*mode = 0660;

	return NULL;
}

static bool tegra_hwpm_read_support_soc_tools_prop(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	bool allow_node = of_property_read_bool(np, "support-soc-tools");

	if (!allow_node) {
		tegra_hwpm_err(NULL, "support-soc-tools is absent");
	}

	return allow_node;
}

static int tegra_hwpm_init_chip_info(struct tegra_hwpm_os_linux *hwpm_linux)
{
	hwpm_linux->device_info.chip = tegra_get_chip_id();
	hwpm_linux->device_info.chip_revision = tegra_get_major_rev();
	hwpm_linux->device_info.revision = tegra_chip_get_revision();
	hwpm_linux->device_info.platform = tegra_get_platform();

	return 0;
}

static int tegra_hwpm_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = NULL;
	struct tegra_hwpm_os_linux *hwpm_linux = NULL;
	struct tegra_soc_hwpm *hwpm = NULL;

	if (!pdev) {
		tegra_hwpm_err(NULL, "Invalid platform device");
		ret = -ENODEV;
		goto fail;
	}

	if (!tegra_hwpm_read_support_soc_tools_prop(pdev)) {
		tegra_hwpm_err(NULL, "SOC HWPM not supported in this config");
		ret = -ENODEV;
		goto fail;
	}

	hwpm_linux = kzalloc(sizeof(struct tegra_hwpm_os_linux), GFP_KERNEL);
	if (!hwpm_linux) {
		tegra_hwpm_err(NULL,
			"Couldn't allocate memory for linux hwpm struct");
		ret = -ENOMEM;
		goto fail;
	}
	hwpm_linux->pdev = pdev;
	hwpm_linux->dev = &pdev->dev;
	hwpm_linux->np = pdev->dev.of_node;
	hwpm_linux->class.owner = THIS_MODULE;
	hwpm_linux->class.name = TEGRA_SOC_HWPM_MODULE_NAME;

	hwpm = &hwpm_linux->hwpm;

	/* Create device node */
	ret = class_register(&hwpm_linux->class);
	if (ret) {
		tegra_hwpm_err(hwpm, "Failed to register class");
		goto class_register;
	}

	/* Set devnode to retrieve device permissions */
	hwpm_linux->class.devnode = tegra_hwpm_get_devnode;

	ret = alloc_chrdev_region(&hwpm_linux->dev_t, 0, 1,
		dev_name(hwpm_linux->dev));
	if (ret) {
		tegra_hwpm_err(hwpm, "Failed to allocate device region");
		goto alloc_chrdev_region;
	}

	cdev_init(&hwpm_linux->cdev, &tegra_hwpm_ops);
	hwpm_linux->cdev.owner = THIS_MODULE;

	ret = cdev_add(&hwpm_linux->cdev, hwpm_linux->dev_t, 1);
	if (ret) {
		tegra_hwpm_err(hwpm, "Failed to add cdev");
		goto cdev_add;
	}

	dev = device_create(&hwpm_linux->class, NULL,
		hwpm_linux->dev_t, NULL, TEGRA_SOC_HWPM_MODULE_NAME);
	if (IS_ERR(dev)) {
		tegra_hwpm_err(hwpm, "Failed to create device");
		ret = PTR_ERR(dev);
		goto device_create;
	}

	(void) dma_set_mask_and_coherent(hwpm_linux->dev, DMA_BIT_MASK(39));

	if (tegra_hwpm_is_platform_silicon()) {
		hwpm_linux->la_clk = devm_clk_get(hwpm_linux->dev, "la");
		if (IS_ERR(hwpm_linux->la_clk)) {
			tegra_hwpm_err(hwpm, "Missing la clock");
			ret = PTR_ERR(hwpm_linux->la_clk);
			goto clock_reset_fail;
		}

		hwpm_linux->la_parent_clk =
			devm_clk_get(hwpm_linux->dev, "parent");
		if (IS_ERR(hwpm_linux->la_parent_clk)) {
			tegra_hwpm_err(hwpm, "Missing la parent clk");
			ret = PTR_ERR(hwpm_linux->la_parent_clk);
			goto clock_reset_fail;
		}

		hwpm_linux->la_rst =
			devm_reset_control_get(hwpm_linux->dev, "la");
		if (IS_ERR(hwpm_linux->la_rst)) {
			tegra_hwpm_err(hwpm, "Missing la reset");
			ret = PTR_ERR(hwpm_linux->la_rst);
			goto clock_reset_fail;
		}

		hwpm_linux->hwpm_rst =
			devm_reset_control_get(hwpm_linux->dev, "hwpm");
		if (IS_ERR(hwpm_linux->hwpm_rst)) {
			tegra_hwpm_err(hwpm, "Missing hwpm reset");
			ret = PTR_ERR(hwpm_linux->hwpm_rst);
			goto clock_reset_fail;
		}
	}

	tegra_hwpm_debugfs_init(hwpm_linux);

	ret = tegra_hwpm_init_chip_info(hwpm_linux);
	if (ret != 0) {
		tegra_hwpm_err(hwpm, "Failed to initialize current chip info");
		goto init_chip_info_fail;
	}

	ret = tegra_hwpm_init_sw_components(hwpm, hwpm_linux->device_info.chip,
		hwpm_linux->device_info.chip_revision);
	if (ret != 0) {
		tegra_hwpm_err(hwpm, "Failed to init sw components");
		goto init_sw_components_fail;
	}

	/*
	 * Currently VDK doesn't have a fmodel for SOC HWPM. Therefore, we
	 * enable fake registers on VDK for minimal testing.
	 */
	if (tegra_hwpm_is_platform_simulation())
		hwpm->fake_registers_enabled = true;
	else
		hwpm->fake_registers_enabled = false;

	platform_set_drvdata(pdev, hwpm);
	tegra_soc_hwpm_pdev = pdev;

	tegra_hwpm_dbg(hwpm, hwpm_info, "Probe successful!");
	goto success;

init_chip_info_fail:
init_sw_components_fail:
	if (tegra_hwpm_is_platform_silicon()) {
		if (hwpm_linux->la_clk)
			devm_clk_put(hwpm_linux->dev, hwpm_linux->la_clk);
		if (hwpm_linux->la_parent_clk)
			devm_clk_put(hwpm_linux->dev,
				hwpm_linux->la_parent_clk);
		if (hwpm_linux->la_rst)
			reset_control_assert(hwpm_linux->la_rst);
		if (hwpm_linux->hwpm_rst)
			reset_control_assert(hwpm_linux->hwpm_rst);
	}
clock_reset_fail:
	device_destroy(&hwpm_linux->class, hwpm_linux->dev_t);
device_create:
	cdev_del(&hwpm_linux->cdev);
cdev_add:
	unregister_chrdev_region(hwpm_linux->dev_t, 1);
alloc_chrdev_region:
	class_unregister(&hwpm_linux->class);
class_register:
	tegra_hwpm_kfree(NULL, hwpm_linux);
fail:
	tegra_hwpm_err(NULL, "Probe failed!");
success:
	return ret;
}

static int tegra_hwpm_remove(struct platform_device *pdev)
{
	struct tegra_hwpm_os_linux *hwpm_linux = NULL;
	struct tegra_soc_hwpm *hwpm = NULL;

	if (!pdev) {
		tegra_hwpm_err(hwpm, "Invalid platform device");
		return -ENODEV;
	}

	hwpm = platform_get_drvdata(pdev);
	if (!hwpm) {
		tegra_hwpm_err(hwpm, "Invalid hwpm struct");
		return -ENODEV;
	}

	hwpm_linux = tegra_hwpm_os_linux_from_hwpm(hwpm);
	if (!hwpm_linux) {
		tegra_hwpm_err(NULL, "Invalid hwpm_linux struct");
		return -ENODEV;
	}

	if (tegra_hwpm_is_platform_silicon()) {
		if (hwpm_linux->la_clk)
			devm_clk_put(hwpm_linux->dev, hwpm_linux->la_clk);
		if (hwpm_linux->la_parent_clk)
			devm_clk_put(hwpm_linux->dev, hwpm_linux->la_parent_clk);
		if (hwpm_linux->la_rst)
			reset_control_assert(hwpm_linux->la_rst);
		if (hwpm_linux->hwpm_rst)
			reset_control_assert(hwpm_linux->hwpm_rst);
	}

	tegra_hwpm_release_ip_register_node(hwpm);
	tegra_hwpm_release_sw_setup(hwpm);
	tegra_hwpm_debugfs_deinit(hwpm_linux);

	device_destroy(&hwpm_linux->class, hwpm_linux->dev_t);
	cdev_del(&hwpm_linux->cdev);
	unregister_chrdev_region(hwpm_linux->dev_t, 1);
	class_unregister(&hwpm_linux->class);

	tegra_hwpm_kfree(NULL, hwpm_linux);
	tegra_soc_hwpm_pdev = NULL;

	return 0;
}

static struct platform_driver tegra_soc_hwpm_pdrv = {
	.probe		= tegra_hwpm_probe,
	.remove		= tegra_hwpm_remove,
	.driver		= {
		.name	= TEGRA_SOC_HWPM_MODULE_NAME,
		.of_match_table = of_match_ptr(tegra_soc_hwpm_of_match),
	},
};

static int __init tegra_hwpm_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&tegra_soc_hwpm_pdrv);
	if (ret < 0)
		tegra_hwpm_err(NULL, "Platform driver register failed");

	return ret;
}

static void __exit tegra_hwpm_exit(void)
{
	platform_driver_unregister(&tegra_soc_hwpm_pdrv);
}

postcore_initcall(tegra_hwpm_init);
module_exit(tegra_hwpm_exit);

MODULE_ALIAS(TEGRA_SOC_HWPM_MODULE_NAME);
MODULE_DESCRIPTION("Tegra SOC HWPM Driver");
MODULE_LICENSE("GPL v2");
