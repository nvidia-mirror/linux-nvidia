/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 */

#include <linux/version.h>
#include <linux/sys_soc.h>
#include <soc/tegra/fuse.h>

#define FUSE_SKU_INFO	0x10
#define FUSE_BEGIN	0x100

static inline u32 tegra_get_sku_id(void)
{
	if (!tegra_sku_info.sku_id)
		tegra_fuse_readl(FUSE_SKU_INFO, &tegra_sku_info.sku_id);

	return tegra_sku_info.sku_id;
}

#ifdef CONFIG_TEGRA_FUSE_UPSTREAM

static inline bool tegra_platform_is_silicon(void)
{
	const struct soc_device_attribute tegra_soc_attrs[] = {
		{ .revision = "Silicon*" },
		{/* sentinel */}
	};

	if (soc_device_match(tegra_soc_attrs))
		return true;

	return false;
}

static inline bool tegra_platform_is_sim(void)
{
	return !tegra_platform_is_silicon();
}

static inline bool tegra_platform_is_fpga(void)
{
	const struct soc_device_attribute tegra_soc_attrs[] = {
		{ .revision = "*FPGA" },
		{/* sentinel */}
	};

	if (soc_device_match(tegra_soc_attrs))
		return true;

	return false;
}

static inline bool tegra_platform_is_vdk(void)
{
	const struct soc_device_attribute tegra_soc_attrs[] = {
		{ .revision = "VDK" },
		{/* sentinel */}
	};

	if (soc_device_match(tegra_soc_attrs))
		return true;

	return false;
}

/*
 * tegra_fuse_control_read() - allows reading fuse offsets < 0x100.
 * @offset: Offset to be read.
 * @value: Pointer to an unsigned integer where the value is stored.
 *
 * Function tegra_fuse_control_read() allows reading fuse_offsets < 0x100
 * by using the already upstreamed tegra_fuse_readl() function.
 *
 * Return: Returns a negative integer in case of error, 0 in case
 * of success.
 */
static inline int tegra_fuse_control_read(unsigned long offset, u32 *value)
{
	/*
	 * Allow reading offsets between 0x0 - 0xff.
	 * For offsets > 0xff, use tegra_fuse_readl instead.
	 */
	if (WARN_ON(offset > 0xff))
		return -EINVAL;

	/*
	 * This will overflow the offset value, which is safe as
	 * tegra_fuse_readl() would again add 0x100 to it.
	 */
	offset -= FUSE_BEGIN;

	return tegra_fuse_readl(offset, value);
}
#endif /* CONFIG_TEGRA_FUSE_UPSTREAM */
