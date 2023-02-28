// SPDX-License-Identifier: GPL-2.0-only
/*
 * Tegra host1x activity monitor interfaes
 *
 * Copyright (c) 2023, NVIDIA Corporation.
 */

#include "dev.h"

int host1x_actmon_read_avg_count(struct host1x_client *client)
{
	struct host1x *host = dev_get_drvdata(client->host->parent);
	unsigned int offset;

	if (!host->actmon_regs)
		return -ENODEV;

	/* FIXME: Only T234 supported */

	switch (client->class) {
	case HOST1X_CLASS_NVENC:
		offset = 0x0;
		break;
	case HOST1X_CLASS_VIC:
		offset = 0x10000;
		break;
	case HOST1X_CLASS_NVDEC:
		offset = 0x20000;
		break;
	default:
		return -EINVAL;
	}

	return readl(host->actmon_regs + offset + 0xa4);
}
EXPORT_SYMBOL(host1x_actmon_read_avg_count);
