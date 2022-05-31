// SPDX-License-Identifier: GPL-2.0-only
/*
 * FDE dev node for FDE hardware.
 *
 * Support for File decompression algorithms.
 *
 * Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES.  All rights reserved.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/crypto.h>
#include <linux/scatterlist.h>
#include <linux/uaccess.h>
#include <linux/nospec.h>
#include <linux/mutex.h>
#include <linux/version.h>
#include <crypto/hash.h>
#include <linux/platform/tegra/common.h>
#include <soc/tegra/fuse.h>
#include <crypto/internal/skcipher.h>

#include <uapi/misc/tegra-fdedev.h>
#include <asm/barrier.h>

#define get_driver_name(tfm_type, tfm) crypto_tfm_alg_driver_name(tfm_type ## _tfm(tfm))

struct tegra_fde_ctx {
	struct crypto_ahash *sha_tfm[6];
	struct mutex lock;
};

struct tegra_fde_completion {
	struct completion restart;
	int req_err;
};

static int tegra_fde_dev_open(struct inode *inode, struct file *filp)
{
	struct tegra_fde_ctx *ctx;
	int ret = 0;

	ctx = kzalloc(sizeof(struct tegra_fde_ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mutex_init(&ctx->lock);

	filp->private_data = ctx;
	return ret;
}

static int tegra_fde_dev_release(struct inode *inode, struct file *filp)
{
	struct tegra_fde_ctx *ctx = filp->private_data;

	if (ctx) {
		mutex_destroy(&ctx->lock);
		kfree(ctx);
		filp->private_data = NULL;
	}
	return 0;
}

static int wait_async_op(struct tegra_fde_completion *tr, int ret)
{
	if (ret == -EINPROGRESS || ret == -EBUSY) {
		wait_for_completion(&tr->restart);
		reinit_completion(&tr->restart);
		ret = tr->req_err;
	}
	return ret;
}

struct tegra_fde_priv_data {
	/* Tells whether it is block or frame mode*/
	u32 fde_mode;
	/* Source buffer handle */
	u32 handle1;
	/* Destination buffer handle or buffer size if SE is used*/
	u32 handle2;
};

static int tegra_fde_dev_ioctl_process(struct tegra_fde_req *fde_req)
{
	u64 input_size;
	u64 output_buffer_size;
	u8 *input_buffer = NULL;
	u8 *output_buffer = NULL;
	struct scatterlist input_sg;
	struct ahash_request *ahash_req;
	struct tegra_fde_completion complete;
	unsigned char priv = 0;
	struct crypto_ahash *tfm;
	const char *driver_name;
	int ret = 0;
	struct tegra_fde_priv_data	priv_data;
	int fd;
	int fd1;

	input_size = fde_req->input_size;
	priv = fde_req->op_mode;

	ret = copy_from_user(&output_buffer_size,
			     fde_req->output_size,
			     sizeof(output_buffer_size));
	if (ret) {
		pr_err("%s: unable to copy result to user\n",
		       __func__);
		ret = -EFAULT;
		goto free_ahash_req;
	}

	input_buffer = kzalloc(16, GFP_KERNEL);
	if (!input_buffer) {
		ret = -ENOMEM;
		goto out;
	}

	output_buffer = kzalloc(16, GFP_KERNEL);
	if (!output_buffer) {
		ret = -ENOMEM;
		goto free_input_buffer;
	}

	tfm = crypto_alloc_ahash("fde", 0, 0);
	if (IS_ERR(tfm)) {
		pr_err("alg: hash: failed to allocate transform for sha1\n");
		ret = -EINVAL;
		goto free_buffers;
	}

	driver_name = get_driver_name(crypto_ahash, tfm);
	if (!driver_name) {
		pr_err("alg: hash: get_driver_name returned NULL\n");
		ret = -EINVAL;
		goto free_tfm;
	}

	ahash_req = ahash_request_alloc(tfm, GFP_KERNEL);
	if (!ahash_req) {
		pr_err("alg: hash: failed to allocate ahash req\n");
		ret = -ENOMEM;
		goto free_tfm;
	}

	init_completion(&complete.restart);
	complete.req_err = 0;

	ret = wait_async_op(&complete, crypto_ahash_init(ahash_req));
	if (ret) {
		pr_err("alg: hash: init failed\n");
		ret = -EINVAL;
		goto free_ahash_req;
	}

	sg_init_one(&input_sg, input_buffer, input_size);

	ahash_request_set_crypt(ahash_req, &input_sg, output_buffer, input_size);
	fd = fde_req->fd;
	fd1 = fde_req->fd1;
	ahash_req->dst_size = output_buffer_size;
	priv_data.fde_mode = priv;
	priv_data.handle1 = fd;
	priv_data.handle2 = fd1;
	ahash_req->priv = &priv_data;

	ret = wait_async_op(&complete,
			    crypto_ahash_finup(ahash_req));
	if (ret) {
		pr_err("alg: hash: update failed\n");
		goto free_ahash_req;
	}
free_ahash_req:
	ahash_request_free(ahash_req);
free_tfm:
	crypto_free_ahash(tfm);
free_buffers:
	kfree(output_buffer);
free_input_buffer:
	kfree(input_buffer);
out:

	return ret;
}

static long tegra_fde_dev_ioctl(struct file *filp, unsigned int ioctl_num,
				unsigned long arg)
{
	struct tegra_fde_ctx *ctx = filp->private_data;
	struct tegra_fde_req fde_req;
	int ret = 0;

	/*
	 * Avoid processing ioctl if the file has been closed.
	 * This will prevent crashes caused by NULL pointer dereference.
	 */
	if (!ctx) {
		pr_err("%s: ctx not allocated\n", __func__);
		return -EPERM;
	}

	mutex_lock(&ctx->lock);

	switch (ioctl_num) {
	case TEGRA_IOCTL_GET_FDE:
		if (copy_from_user(&fde_req, (void __user *)arg,
				   sizeof(fde_req))) {
			pr_err("%s: copy_from_user fail(%d)\n",
			       __func__, ret);
			ret = -EFAULT;
			goto out;
		}
		ret = tegra_fde_dev_ioctl_process(&fde_req);
		break;
	default:
		pr_debug("invalid ioctl code(%d)", ioctl_num);
		ret = -EINVAL;
	}
out:
	mutex_unlock(&ctx->lock);
	return ret;
}

static const struct file_operations tegra_fde_fops = {
	.owner = THIS_MODULE,
	.open = tegra_fde_dev_open,
	.release = tegra_fde_dev_release,
	.unlocked_ioctl = tegra_fde_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl =  tegra_fde_dev_ioctl,
#endif
};

static struct miscdevice tegra_fde_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tegra-fdedev",
	.fops = &tegra_fde_fops,
};

static int __init tegra_fde_dev_init(void)
{
	return misc_register(&tegra_fde_device);
}

late_initcall(tegra_fde_dev_init);

static void __exit tegra_fde_module_exit(void)
{
	misc_deregister(&tegra_fde_device);
}
module_exit(tegra_fde_module_exit);

MODULE_DESCRIPTION("Tegra FDE device node.");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
