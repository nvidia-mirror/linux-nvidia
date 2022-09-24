/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 */

#ifndef __TEGRA_HV_VSE_H
#define __TEGRA_HV_VSE_H

struct tegra_vse_soc_info {
	bool cmac_hw_padding_supported;
	bool gcm_decrypt_supported;
};

/* GCM Operation Supported Flag */
enum tegra_gcm_dec_supported {
	GCM_DEC_OP_NOT_SUPPORTED,
	GCM_DEC_OP_SUPPORTED,
};

struct crypto_dev_to_ivc_map {
	uint32_t ivc_id;
	uint32_t se_engine;
	uint32_t node_id;
	uint32_t priority;
	uint32_t channel_grp_id;
	enum tegra_gcm_dec_supported gcm_dec_supported;
	struct tegra_hv_ivc_cookie *ivck;
	struct completion tegra_vse_complete;
	struct task_struct *tegra_vse_task;
	bool vse_thread_start;
	struct mutex se_ivc_lock;
};

struct tegra_virtual_se_dev {
	struct device *dev;
	/* Engine id */
	unsigned int engine_id;
	/* Engine suspend state */
	atomic_t se_suspended;
	struct tegra_vse_soc_info *chipdata;
#if defined(CONFIG_HW_RANDOM)
	/* Integration with hwrng framework */
	struct hwrng *hwrng;
#endif /* CONFIG_HW_RANDOM */
	struct platform_device *host1x_pdev;
	struct crypto_dev_to_ivc_map *crypto_to_ivc_map;
};

/* Security Engine random number generator context */
struct tegra_virtual_se_rng_context {
	/* Security Engine device */
	struct tegra_virtual_se_dev *se_dev;
	/* RNG buffer pointer */
	u32 *rng_buf;
	/* RNG buffer dma address */
	dma_addr_t rng_buf_adr;
	/*Crypto dev instance*/
	uint32_t node_id;
};

/* Security Engine AES context */
struct tegra_virtual_se_aes_context {
	/* Security Engine device */
	struct tegra_virtual_se_dev *se_dev;
	struct skcipher_request *req;
	/* Security Engine key slot */
	u32 aes_keyslot;
	/* key length in bytes */
	u32 keylen;
	/* AES operation mode */
	u32 op_mode;
	/* Is key slot */
	bool is_key_slot_allocated;
	/* size of GCM tag*/
	u32 authsize;
	/*Crypto dev instance*/
	uint32_t node_id;
};

/* Security Engine AES CMAC context */
struct tegra_virtual_se_aes_cmac_context {
	unsigned int digest_size;
	u8 *hash_result;		/* Intermediate hash result */
	dma_addr_t hash_result_addr;	/* Intermediate hash result dma addr */
	bool is_first;			/* Represents first block */
	bool req_context_initialized;	/* Mark initialization status */
	u32 aes_keyslot;
	/* key length in bits */
	u32 keylen;
	bool is_key_slot_allocated;
	/*Crypto dev instance*/
	uint32_t node_id;
};

/* Security Engine AES GMAC context */
struct tegra_virtual_se_aes_gmac_context {
	/* size of GCM tag*/
	u32 authsize;
	/* Mark initialization status */
	bool req_context_initialized;
	u32 aes_keyslot;
	/* key length in bits */
	u32 keylen;
	bool is_key_slot_allocated;
	/*Crypto dev instance*/
	uint32_t node_id;
};

/* Security Engine SHA context */
struct tegra_virtual_se_sha_context {
	/* Security Engine device */
	struct tegra_virtual_se_dev *se_dev;
	/* SHA operation mode */
	u32 op_mode;
	unsigned int digest_size;
	u8 mode;
	/*Crypto dev instance*/
	uint32_t node_id;
};

/* Security Engine request context */
struct tegra_virtual_se_req_context {
	/* Security Engine device */
	struct tegra_virtual_se_dev *se_dev;
	unsigned int digest_size;
	unsigned int intermediate_digest_size;
	u8 mode;			/* SHA operation mode */
	u8 *sha_buf;			/* Buffer to store residual data */
	dma_addr_t sha_buf_addr;	/* DMA address to residual data */
	u8 *hash_result;		/* Intermediate hash result */
	dma_addr_t hash_result_addr;	/* Intermediate hash result dma addr */
	u64 total_count;		/* Total bytes in all the requests */
	u32 residual_bytes;		/* Residual byte count */
	u32 blk_size;			/* SHA block size */
	bool is_first;			/* Represents first block */
	bool req_context_initialized;	/* Mark initialization status */
	bool force_align;		/* Enforce buffer alignment */
	/*Crypto dev instance*/
	uint32_t node_id;
};
#endif /*__TEGRA_HV_VSE_H*/
