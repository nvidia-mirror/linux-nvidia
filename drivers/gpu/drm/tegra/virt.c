// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022, NVIDIA Corporation.
 */

#include <linux/host1x-next.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/version.h>

#include <soc/tegra/virt/hv-ivc.h>

#include "drm.h"

#define TEGRA_VHOST_CMD_CONNECT			0
#define TEGRA_VHOST_CMD_DISCONNECT		1
#define TEGRA_VHOST_CMD_SUSPEND			2
#define TEGRA_VHOST_CMD_RESUME			3
#define TEGRA_VHOST_CMD_GET_CONNECTION_ID	4

struct tegra_vhost_connect_params {
	u32 module;
	u64 connection_id;
};

struct tegra_vhost_cmd_msg {
	u32 cmd;
	int ret;
	u64 connection_id;
	struct tegra_vhost_connect_params connect;
};

struct virt_engine {
	struct device *dev;
	struct tegra_hv_ivc_cookie *cookie;
	int connection_id;

	struct tegra_drm_client client;
};

static inline struct virt_engine *to_virt_engine(struct tegra_drm_client *client)
{
	return container_of(client, struct virt_engine, client);
}

static int virt_engine_init(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->host);
	struct tegra_drm *tegra = dev->dev_private;
	int err;

	err = tegra_drm_register_client(tegra, drm);
	if (err < 0)
		return err;

	/*
	 * Inherit the DMA parameters (such as maximum segment size) from the
	 * parent host1x device.
	 */
	client->dev->dma_parms = client->host->dma_parms;

	return 0;
}

static int virt_engine_exit(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->host);
	struct tegra_drm *tegra = dev->dev_private;
	int err;

	/* avoid a dangling pointer just in case this disappears */
	client->dev->dma_parms = NULL;

	err = tegra_drm_unregister_client(tegra, drm);
	if (err < 0)
		return err;

	return 0;
}

static const struct host1x_client_ops virt_engine_client_ops = {
	.init = virt_engine_init,
	.exit = virt_engine_exit,
};

static int virt_engine_open_channel(struct tegra_drm_client *client,
				    struct tegra_drm_context *context)
{
	return -EINVAL;
}

static void virt_engine_close_channel(struct tegra_drm_context *context)
{
}

static int virt_engine_can_use_memory_ctx(struct tegra_drm_client *client, bool *supported)
{
	*supported = true;

	return 0;
}

static const struct tegra_drm_client_ops virt_engine_ops = {
	.open_channel = virt_engine_open_channel,
	.close_channel = virt_engine_close_channel,
	.submit = tegra_drm_submit,
	.get_streamid_offset = tegra_drm_get_streamid_offset_thi,
	.can_use_memory_ctx = virt_engine_can_use_memory_ctx,
};

static int virt_engine_setup_ivc(struct virt_engine *virt)
{
	struct device_node *host1x_dn = virt->dev->parent->of_node;
	struct device_node *hv;
	u32 ivc_instance;
	int err;

	hv = of_parse_phandle(host1x_dn, "nvidia,server-ivc", 0);
	if (!hv) {
		dev_err(virt->dev, "nvidia,server-ivc not configured\n");
		return -EINVAL;
	}

	err = of_property_read_u32_index(host1x_dn, "nvidia,server-ivc", 1, &ivc_instance);
	if (err) {
		dev_err(virt->dev, "nvidia,server-ivc not configured\n");
		return -EINVAL;
	}

	virt->cookie = tegra_hv_ivc_reserve(hv, ivc_instance, NULL);
	if (IS_ERR(virt->cookie)) {
		dev_err(virt->dev, "IVC channel reservation failed: %ld\n", PTR_ERR(virt->cookie));
		return PTR_ERR(virt->cookie);
	}

	tegra_hv_ivc_channel_reset(virt->cookie);

	while (tegra_hv_ivc_channel_notified(virt->cookie))
		;

	return 0;
}

static int virt_engine_transfer(struct virt_engine *virt, void *data, u32 size)
{
	int err;

	while (!tegra_hv_ivc_can_write(virt->cookie))
		;

	err = tegra_hv_ivc_write(virt->cookie, data, size);
	if (err != size)
		return -ENOMEM;

	while (!tegra_hv_ivc_can_read(virt->cookie))
		;

	err = tegra_hv_ivc_read(virt->cookie, data, size);
	if (err != size)
		return -EIO;

	return 0;
}

static int virt_engine_connect(struct virt_engine *virt, u32 module_id)
{
	struct tegra_vhost_cmd_msg msg = { 0 };
	int err;

	msg.cmd = TEGRA_VHOST_CMD_CONNECT;
	msg.connect.module = module_id;

	err = virt_engine_transfer(virt, &msg, sizeof(msg));
	if (err < 0)
		return err;

	return msg.connect.connection_id;
}

static int virt_engine_suspend(struct device *dev)
{
	struct virt_engine *virt = dev_get_drvdata(dev);
	struct tegra_vhost_cmd_msg msg = { 0 };

	msg.cmd = TEGRA_VHOST_CMD_SUSPEND;
	msg.connection_id = virt->connection_id;

	return virt_engine_transfer(virt, &msg, sizeof(msg));
}

static int virt_engine_resume(struct device *dev)
{
	struct virt_engine *virt = dev_get_drvdata(dev);
	struct tegra_vhost_cmd_msg msg = { 0 };

	msg.cmd = TEGRA_VHOST_CMD_RESUME;
	msg.connection_id = virt->connection_id;

	return virt_engine_transfer(virt, &msg, sizeof(msg));
}

static const struct of_device_id tegra_virt_engine_of_match[] = {
	{ .compatible = "nvidia,tegra234-host1x-virtual-engine" },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_virt_engine_of_match);

static int virt_engine_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	u32 module_id, class;
	struct virt_engine *virt;
	int err;

	err = of_property_read_u32(pdev->dev.of_node, "nvidia,module-id", &module_id);
	if (err < 0) {
		dev_err(dev, "could not read property nvidia,module-id: %d\n", err);
		return err;
	}

	err = of_property_read_u32(pdev->dev.of_node, "nvidia,class", &class);
	if (err < 0) {
		dev_err(dev, "could not read property nvidia,class: %d\n", err);
		return err;
	}

	/* inherit DMA mask from host1x parent */
	err = dma_coerce_mask_and_coherent(dev, *dev->parent->dma_mask);
	if (err < 0) {
		dev_err(dev, "failed to set DMA mask: %d\n", err);
		return err;
	}

	virt = devm_kzalloc(dev, sizeof(*virt), GFP_KERNEL);
	if (!virt)
		return -ENOMEM;

	platform_set_drvdata(pdev, virt);

	INIT_LIST_HEAD(&virt->client.base.list);
	virt->client.base.ops = &virt_engine_client_ops;
	virt->client.base.dev = dev;
	virt->client.base.class = class;
	virt->client.base.syncpts = NULL;
	virt->client.base.num_syncpts = 0;
	virt->dev = dev;

	INIT_LIST_HEAD(&virt->client.list);
	virt->client.version = 0x23;
	virt->client.ops = &virt_engine_ops;

	err = host1x_client_register(&virt->client.base);
	if (err < 0) {
		dev_err(dev, "failed to register host1x client: %d\n", err);
		return err;
	}

	err = virt_engine_setup_ivc(virt);
	if (err < 0)
		goto unregister_client;

	/* Connect to HOST module. This doesn't really do anything but we need to do it first. */
	virt_engine_connect(virt, 1);

	virt->connection_id = virt_engine_connect(virt, module_id);
	if (virt->connection_id < 0) {
		dev_err(dev, "failed to register with server\n");
		err = -EIO;
		goto cleanup_ivc;
	}

	return 0;

cleanup_ivc:
	tegra_hv_ivc_unreserve(virt->cookie);
unregister_client:
	host1x_client_unregister(&virt->client.base);

	return err;
}

static int virt_engine_remove(struct platform_device *pdev)
{
	struct virt_engine *virt_engine = platform_get_drvdata(pdev);
	int err;

	tegra_hv_ivc_unreserve(virt_engine->cookie);

	err = host1x_client_unregister(&virt_engine->client.base);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to unregister host1x client: %d\n",
			err);
		return err;
	}

	return 0;
}

static const struct dev_pm_ops virt_engine_pm_ops = {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0)
	SYSTEM_SLEEP_PM_OPS(virt_engine_suspend, virt_engine_resume)
#else
	SET_SYSTEM_SLEEP_PM_OPS(virt_engine_suspend, virt_engine_resume)
#endif
};

struct platform_driver tegra_virt_engine_driver = {
	.driver = {
		.name = "tegra-host1x-virtual-engine",
		.of_match_table = tegra_virt_engine_of_match,
		.pm = &virt_engine_pm_ops,
	},
	.probe = virt_engine_probe,
	.remove = virt_engine_remove,
};
