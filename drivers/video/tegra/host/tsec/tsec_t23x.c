/*
 * Tegra TSEC Module Support on t23x
 *
 * Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/slab.h>         /* for kzalloc */
#include <linux/delay.h>	/* for udelay */
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/dma-mapping.h>
#include <linux/platform/tegra/tegra_mc.h>
#include <soc/tegra/fuse.h>
#include <asm/cacheflush.h>

#include "dev.h"
#include "bus_client.h"
#include "tsec.h"
#include "tsec_t23x.h"
#include "hw_tsec_t23x.h"
#include "flcn/flcn.h"
#include "flcn/hw_flcn.h"
#include "riscv/riscv.h"
#include "hw_tsec_t23x.h"
#include "rm_flcn_cmds.h"

#define TSEC_RISCV_INIT_SUCCESS		(0xa5a5a5a5)
#define NV_RISCV_AMAP_FBGPA_START	0x0000040000000000ULL
#define NV_RISCV_AMAP_SMMU_IDX		BIT_ULL(40)

/* 'N' << 24 | 'V' << 16 | 'R' << 8 | 'M' */
#define RM_RISCV_BOOTLDR_BOOT_TYPE_RM	0x4e56524d

/* Version of bootloader struct, increment on struct changes (while on prod) */
#define RM_RISCV_BOOTLDR_VERSION	1

#define DO_IPC_OVER_GSC_CO (1)
/*
 * Uncomment below when RM is running on CCPLEX and GSC has been
 * configured via BCT files to allow access via CCPLEX to GSC-CO
 */
// #define TSEC_RM_ON_DCE (1)
static bool s_init_msg_rcvd;
static bool s_riscv_booted;
static struct RM_FLCN_MSG_GSP s_gsp_init_msg;

#ifdef DO_IPC_OVER_GSC_CO
static u64 s_ipc_gscco_base;
static u64 s_ipc_gscco_size;
static u64 s_ipc_gscco_page_base;
static u64 s_ipc_gscco_page_size;
static u64 s_ipc_gscco_page_count;
struct TSEC_BOOT_INFO {
	u32    bootFlag;
};
#endif

/* Pointer to this device */
static struct platform_device *tsec;

/* Struct to register client callback function */
struct callback_t {
	callback_func_t cb_func;
	void            *cb_ctx;
};
static struct callback_t s_callbacks[RM_GSP_UNIT_END];

DEFINE_MUTEX(sCommsMutex);

void plat_acquire_comms_mutex(void)
{
	mutex_lock(&sCommsMutex);
}

void plat_release_comms_mutex(void)
{
	mutex_unlock(&sCommsMutex);
}

/* Configuration for bootloader */
typedef struct {
	/*
	 *                   *** WARNING ***
	 * First 3 fields must be frozen like that always. Should never
	 * be reordered or changed.
	 */
	u32 bootType;        // Set to 'NVRM' if booting from RM.
	u16 size;            // Size of boot params.
	u8  version;         // Version of boot params.
	/*
	 * You can reorder or change below this point but update version.
	 */
} NV_RISCV_BOOTLDR_PARAMS;

typedef void (*work_cb_t)(void *);

struct tsec_t23x_device {
	struct platform_device *pdev;
	struct delayed_work poweron_work;
	u32 fwreq_retry_interval_ms;
	u32 fwreq_duration_ms;
	u32 fwreq_fail_threshold_ms;
	struct work_struct  initmsg_work;
	work_cb_t           initmsg_cb;
	void               *initmsg_cb_ctx;
};

void initmsg_work_handler(struct work_struct *work)
{
	struct tsec_t23x_device *tsec_dev;
	work_cb_t cb;
	void *cb_ctx;

	tsec_dev = container_of(work, struct tsec_t23x_device,
		initmsg_work);
	cb = tsec_dev->initmsg_cb;
	cb_ctx = tsec_dev->initmsg_cb_ctx;
	tsec_dev->initmsg_cb = NULL;
	tsec_dev->initmsg_cb_ctx = NULL;
	if (cb)
		cb(cb_ctx);
}

void plat_queue_work(work_cb_t cb, void *ctx)
{
	struct nvhost_device_data *pdata = nvhost_get_devdata(tsec);
	struct tsec_t23x_device *tsec_dev = (struct tsec_t23x_device *)pdata->private_data;

	tsec_dev->initmsg_cb = cb;
	tsec_dev->initmsg_cb_ctx = ctx;
	schedule_work(&tsec_dev->initmsg_work);
}

static int tsec_read_riscv_bin(struct platform_device *dev,
			const char *desc_name,
			const char *image_name)
{
	int err, w;
	const struct firmware *riscv_desc, *riscv_image;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	struct riscv_data *m = (struct riscv_data *)pdata->riscv_data;

	if (!m) {
		dev_err(&dev->dev, "riscv data is NULL\n");
		return -ENODATA;
	}

	m->dma_addr = 0;
	m->mapped = NULL;
	riscv_desc = nvhost_client_request_firmware(dev, desc_name, true);
	if (!riscv_desc) {
		dev_err(&dev->dev, "failed to get tsec desc binary\n");
		return -ENOENT;
	}

	riscv_image = nvhost_client_request_firmware(dev, image_name, true);
	if (!riscv_image) {
		dev_err(&dev->dev, "failed to get tsec image binary\n");
		release_firmware(riscv_desc);
		return -ENOENT;
	}

	m->size = riscv_image->size;
	m->mapped = dma_alloc_attrs(&dev->dev, m->size, &m->dma_addr,
				GFP_KERNEL,
				DMA_ATTR_READ_ONLY | DMA_ATTR_FORCE_CONTIGUOUS);
	if (!m->mapped) {
		dev_err(&dev->dev, "dma memory allocation failed");
		err = -ENOMEM;
		goto clean_up;
	}

	/* Copy the whole image taking endianness into account */
	for (w = 0; w < riscv_image->size/sizeof(u32); w++)
		m->mapped[w] = le32_to_cpu(((__le32 *)riscv_image->data)[w]);
	__flush_dcache_area((void *)m->mapped, riscv_image->size);

	/* Read the offsets from desc binary */
	err = riscv_compute_ucode_offsets(dev, m, riscv_desc);
	if (err) {
		dev_err(&dev->dev, "failed to parse desc binary\n");
		goto clean_up;
	}

	m->valid = true;
	release_firmware(riscv_desc);
	release_firmware(riscv_image);

	return 0;

clean_up:
	if (m->mapped) {
		dma_free_attrs(&dev->dev, m->size, m->mapped, m->dma_addr,
				DMA_ATTR_READ_ONLY | DMA_ATTR_FORCE_CONTIGUOUS);
		m->mapped = NULL;
		m->dma_addr = 0;
	}
	release_firmware(riscv_desc);
	release_firmware(riscv_image);
	return err;
}

static int nvhost_tsec_riscv_init_sw(struct platform_device *dev)
{
	int err = 0;
	NV_RISCV_BOOTLDR_PARAMS *bl_args;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	struct riscv_data *m = (struct riscv_data *)pdata->riscv_data;

	if (m)
		return 0;

	m = kzalloc(sizeof(*m), GFP_KERNEL);
	if (!m) {
		dev_err(&dev->dev, "Couldn't allocate for riscv info struct");
		return -ENOMEM;
	}
	pdata->riscv_data = m;

	err = tsec_read_riscv_bin(dev, pdata->riscv_desc_bin,
				pdata->riscv_image_bin);

	if (err || !m->valid) {
		dev_err(&dev->dev, "ucode not valid");
		goto clean_up;
	}

	/*
	 * TSEC firmware expects BL arguments in struct RM_GSP_BOOT_PARAMS.
	 * But, we only populate the first few fields of it. ie.
	 * NV_RISCV_BOOTLDR_PARAMS is located at offset 0 of RM_GSP_BOOT_PARAMS.
	 * actual sizeof(RM_GSP_BOOT_PARAMS) = 152
	 */
	m->bl_args_size = 152;
	m->mapped_bl_args = dma_alloc_attrs(&dev->dev, m->bl_args_size,
				&m->dma_addr_bl_args, GFP_KERNEL, 0);
	if (!m->mapped_bl_args) {
		dev_err(&dev->dev, "dma memory allocation for BL args failed");
		err = -ENOMEM;
		goto clean_up;
	}
	bl_args = (NV_RISCV_BOOTLDR_PARAMS *) m->mapped_bl_args;
	bl_args->bootType = RM_RISCV_BOOTLDR_BOOT_TYPE_RM;
	bl_args->size = m->bl_args_size;
	bl_args->version = RM_RISCV_BOOTLDR_VERSION;

	return 0;

clean_up:
	dev_err(&dev->dev, "RISC-V init sw failed: err=%d", err);
	kfree(m);
	pdata->riscv_data = NULL;
	return err;
}

static int nvhost_tsec_riscv_deinit_sw(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	struct riscv_data *m = (struct riscv_data *)pdata->riscv_data;

	if (!m)
		return 0;

	if (m->mapped) {
		dma_free_attrs(&dev->dev, m->size, m->mapped, m->dma_addr,
				DMA_ATTR_READ_ONLY | DMA_ATTR_FORCE_CONTIGUOUS);
		m->mapped = NULL;
		m->dma_addr = 0;
	}
	if (m->mapped_bl_args) {
		dma_free_attrs(&dev->dev, m->bl_args_size, m->mapped_bl_args,
				m->dma_addr_bl_args, 0);
		m->mapped_bl_args = NULL;
		m->dma_addr_bl_args = 0;
	}
	kfree(m);
	pdata->riscv_data = NULL;
	return 0;
}

#define CMD_INTERFACE_TEST 0
#if CMD_INTERFACE_TEST
#define NUM_OF_CMDS_TO_TEST (5)
#endif

static int nvhost_tsec_riscv_poweron(struct platform_device *dev)
{
#if CMD_INTERFACE_TEST
	union RM_FLCN_CMD cmd;
	struct RM_FLCN_HDCP22_CMD_MONITOR_OFF hdcp22Cmd;
	u8 cmd_size = RM_FLCN_CMD_SIZE(HDCP22, MONITOR_OFF);
	u32 cmdDataSize = RM_FLCN_CMD_BODY_SIZE(HDCP22, MONITOR_OFF);
	int idx;
#endif //CMD_INTERFACE_TEST
	int err = 0;
	struct riscv_data *m;
	u32 val;
	phys_addr_t dma_pa, pa;
	struct iommu_domain *domain;
	void __iomem *cpuctl_addr, *retcode_addr, *mailbox0_addr;
	struct mc_carveout_info inf;
	unsigned int gscid = 0x0;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	struct mc_carveout_info ipc_co_info;
	void __iomem *ipc_co_va = NULL;
	dma_addr_t ipc_co_iova = 0;
	dma_addr_t ipc_co_iova_with_streamid;

	err = nvhost_tsec_riscv_init_sw(dev);
	if (err)
		return err;

	m = (struct riscv_data *)pdata->riscv_data;

	/* Select RISC-V core */
	host1x_writel(dev, tsec_riscv_bcr_ctrl_r(),
			tsec_riscv_bcr_ctrl_core_select_riscv_f());

	/* Get the physical address of corresponding dma address */
	domain = iommu_get_domain_for_dev(&dev->dev);

	/* Get GSC carvout info */
	err = mc_get_carveout_info(&inf, NULL, MC_SECURITY_CARVEOUT4);
	if (err) {
		dev_err(&dev->dev, "Carveout memory allocation failed");
		err = -ENOMEM;
		goto clean_up;
	}

	dev_dbg(&dev->dev, "CARVEOUT4 base=0x%llx size=0x%llx\n",
		inf.base, inf.size);
	if (inf.base) {
		dma_pa = inf.base;
		gscid = 0x4;
		dev_info(&dev->dev, "RISC-V booting from GSC\n");
	} else {
		/* For non-secure boot only. It can be depricated later */
		dma_pa = iommu_iova_to_phys(domain, m->dma_addr);
		dev_info(&dev->dev, "RISC-V boot using kernel allocated Mem\n");
	}

	/* Get IPC Careveout */
	err = mc_get_carveout_info(&ipc_co_info, NULL, MC_SECURITY_CARVEOUT_LITE42);
	if (err) {
		dev_err(&dev->dev, "IPC Carveout memory allocation failed");
		err = -ENOMEM;
		goto clean_up;
	}
	dev_dbg(&dev->dev, "IPCCO base=0x%llx size=0x%llx\n", ipc_co_info.base, ipc_co_info.size);
	ipc_co_va = __ioremap(ipc_co_info.base, ipc_co_info.size, pgprot_noncached(PAGE_KERNEL));
	if (!ipc_co_va) {
		dev_err(&dev->dev, "IPC Carveout memory VA mapping failed");
		err = -ENOMEM;
		goto clean_up;
	}
	dev_dbg(&dev->dev, "IPCCO va=0x%llx pa=0x%llx\n",
		(phys_addr_t)(ipc_co_va), page_to_phys(vmalloc_to_page(ipc_co_va)));
#if (KERNEL_VERSION(5, 10, 0) <= LINUX_VERSION_CODE)
	ipc_co_iova = dma_map_page_attrs(&dev->dev, vmalloc_to_page(ipc_co_va),
		offset_in_page(ipc_co_va), ipc_co_info.size, DMA_BIDIRECTIONAL, 0);
#else
	ipc_co_iova = dma_map_page(&dev->dev, vmalloc_to_page(ipc_co_va),
		offset_in_page(ipc_co_va), ipc_co_info.size, DMA_BIDIRECTIONAL);
#endif
	err = dma_mapping_error(&dev->dev, ipc_co_iova);
	if (err) {
		dev_err(&dev->dev, "IPC Carveout memory IOVA mapping failed");
		ipc_co_iova = 0;
		err = -ENOMEM;
		goto clean_up;
	}
	dev_dbg(&dev->dev, "IPCCO iova=0x%llx\n", ipc_co_iova);

	/* Program manifest start address */
	pa = (dma_pa + m->os.manifest_offset) >> 8;
	host1x_writel(dev, tsec_riscv_bcr_dmaaddr_pkcparam_lo_r(),
			lower_32_bits(pa));
	host1x_writel(dev, tsec_riscv_bcr_dmaaddr_pkcparam_hi_r(),
			upper_32_bits(pa));

	/* Program FMC code start address */
	pa = (dma_pa + m->os.code_offset) >> 8;
	host1x_writel(dev, tsec_riscv_bcr_dmaaddr_fmccode_lo_r(),
			lower_32_bits(pa));
	host1x_writel(dev, tsec_riscv_bcr_dmaaddr_fmccode_hi_r(),
			upper_32_bits(pa));

	/* Program FMC data start address */
	pa = (dma_pa + m->os.data_offset) >> 8;
	host1x_writel(dev, tsec_riscv_bcr_dmaaddr_fmcdata_lo_r(),
			lower_32_bits(pa));
	host1x_writel(dev, tsec_riscv_bcr_dmaaddr_fmcdata_hi_r(),
			upper_32_bits(pa));

	/* Program DMA config registers */
	host1x_writel(dev, tsec_riscv_bcr_dmacfg_sec_r(),
			tsec_riscv_bcr_dmacfg_sec_gscid_f(gscid));
	host1x_writel(dev, tsec_riscv_bcr_dmacfg_r(),
			tsec_riscv_bcr_dmacfg_target_local_fb_f() |
			tsec_riscv_bcr_dmacfg_lock_locked_f());

	/* Pass the address of IPC Carveout via mailbox registers */
	ipc_co_iova_with_streamid = (ipc_co_iova | NV_RISCV_AMAP_SMMU_IDX);
	host1x_writel(dev, tsec_falcon_mailbox0_r(),
		lower_32_bits((unsigned long long)ipc_co_iova_with_streamid));
	host1x_writel(dev, tsec_falcon_mailbox1_r(),
		upper_32_bits((unsigned long long)ipc_co_iova_with_streamid));


	/* Kick start RISC-V and let BR take over */
	host1x_writel(dev, tsec_riscv_cpuctl_r(),
			tsec_riscv_cpuctl_startcpu_true_f());

	cpuctl_addr = get_aperture(dev, 0) + tsec_riscv_cpuctl_r();
	retcode_addr = get_aperture(dev, 0) + tsec_riscv_br_retcode_r();
	mailbox0_addr = get_aperture(dev, 0) + tsec_falcon_mailbox0_r();

	/* Check BR return code */
	err  = readl_poll_timeout(retcode_addr, val,
				(tsec_riscv_br_retcode_result_v(val) ==
				 tsec_riscv_br_retcode_result_pass_v()),
				RISCV_IDLE_CHECK_PERIOD,
				RISCV_IDLE_TIMEOUT_DEFAULT);
	if (err) {
		dev_err(&dev->dev, "BR return code timeout! val=0x%x\n", val);
		goto clean_up;
	}

	/* Check cpuctl active state */
	err  = readl_poll_timeout(cpuctl_addr, val,
				(tsec_riscv_cpuctl_active_stat_v(val) ==
				 tsec_riscv_cpuctl_active_stat_active_v()),
				RISCV_IDLE_CHECK_PERIOD,
				RISCV_IDLE_TIMEOUT_DEFAULT);
	if (err) {
		dev_err(&dev->dev, "cpuctl active state timeout! val=0x%x\n",
			val);
		goto clean_up;
	}

	/* Check tsec has reached a proper initialized state */
	err  = readl_poll_timeout(mailbox0_addr, val,
				(val == TSEC_RISCV_INIT_SUCCESS),
				RISCV_IDLE_CHECK_PERIOD_LONG,
				RISCV_IDLE_TIMEOUT_LONG);
	if (err) {
		dev_err(&dev->dev,
			"not reached initialized state, timeout! val=0x%x\n",
			val);
		goto clean_up;
	}

	/* Host should not receive SWGEN1, as it uses only SWGEN0 for message
	 * communication with tsec. RISCV Fw is generating SWGEN1 for some debug
	 * purpose at below path,, we want to ensure that this doesn't interrupt
	 * Arm driver code.
	 * nvriscv/drivers/src/debug/debug.c:164: irqFireSwGen(SYS_INTR_SWGEN1)
	 */
	host1x_writel(dev, riscv_irqmclr_r(), riscv_irqmclr_swgen1_set_f());

#ifdef DO_IPC_OVER_GSC_CO
	/* Set IPC CO Info before enabling Msg Interrupts from TSEC to CCPLEX */
	s_ipc_gscco_base = (u64)ipc_co_va;
	s_ipc_gscco_size = ipc_co_info.size;
	s_ipc_gscco_page_size = (64 * 1024);
	/* First Page Reserved */
	if (s_ipc_gscco_size > s_ipc_gscco_page_size) {
		s_ipc_gscco_page_count = (s_ipc_gscco_size -
			s_ipc_gscco_page_size) / s_ipc_gscco_page_size;
	} else {
		s_ipc_gscco_page_count = 0;
	}
	s_ipc_gscco_page_base = s_ipc_gscco_page_count ?
		s_ipc_gscco_base + s_ipc_gscco_page_size : 0;
#endif
	enable_irq(pdata->irq);

	s_riscv_booted = true;
	/* Booted-up successfully */
	dev_info(&dev->dev, "RISC-V boot success\n");


#if CMD_INTERFACE_TEST
	pr_debug("cmd_size=%d, cmdDataSize=%d\n", cmd_size, cmdDataSize);
	msleep(3000);
	for (idx = 0; idx < NUM_OF_CMDS_TO_TEST; idx++) {
		hdcp22Cmd.cmdType = RM_FLCN_HDCP22_CMD_ID_MONITOR_OFF;
		hdcp22Cmd.sorNum = -1;
		hdcp22Cmd.dfpSublinkMask = -1;
		cmd.cmdGen.hdr.size = cmd_size;
		cmd.cmdGen.hdr.unitId = RM_GSP_UNIT_HDCP22WIRED;
		cmd.cmdGen.hdr.seqNumId = idx+1;
		cmd.cmdGen.hdr.ctrlFlags = 0;
		memcpy(&cmd.cmdGen.cmd, &hdcp22Cmd, cmdDataSize);
		nvhost_tsec_send_cmd((void *)&cmd, 0, NULL, NULL);
		msleep(200);
	}
#endif //CMD_INTERFACE_TEST

	return err;
clean_up:
	if (ipc_co_iova) {
#if (KERNEL_VERSION(5, 10, 0) <= LINUX_VERSION_CODE)
		dma_unmap_page_attrs(&dev->dev, ipc_co_iova,
			ipc_co_info.size, DMA_BIDIRECTIONAL, 0);
#else
		dma_unmap_page(&dev->dev, ipc_co_iova,
			ipc_co_info.size, DMA_BIDIRECTIONAL);
#endif
	}
	if (ipc_co_va)
		iounmap(ipc_co_va);
	s_riscv_booted = false;
	nvhost_tsec_riscv_deinit_sw(dev);
	return err;
}

int nvhost_tsec_finalize_poweron_t23x(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	if (!pdata) {
		dev_err(&dev->dev, "no platform data\n");
		return -ENODATA;
	}

	flcn_enable_thi_sec(dev);
	if (pdata->enable_riscv_boot) {
		return nvhost_tsec_riscv_poweron(dev);
	} else {
		dev_err(&dev->dev,
			"Falcon boot is not supported from t23x tsec driver\n");
		return -ENOTSUPP;
	}
}

int nvhost_tsec_prepare_poweroff_t23x(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	if (!pdata) {
		dev_err(&dev->dev, "no platform data\n");
		return -ENODATA;
	}

	/*
	 * Below call is redundant, but there are something statically declared
	 * in $(srctree.nvidia)/drivers/video/tegra/host/tsec/tsec.c,
	 * which needs to be reset.
	 */
	if (pdata->irq < 0) {
		dev_err(&dev->dev, "found interrupt number to be negative\n");
		return -ENODATA;
	}
	disable_irq((unsigned int) pdata->irq);
	nvhost_tsec_prepare_poweroff(dev);
	return 0;
}

#define TSEC_CMD_EMEM_SIZE 4
#define TSEC_MSG_QUEUE_PORT 0
#define TSEC_EMEM_START 0x1000000
#define TSEC_EMEM_SIZE 0x2000
#define TSEC_POLL_TIME_MS 2000
#define TSEC_TAIL_POLL_TIME 50
#define TSEC_SMMU_IDX BIT_ULL(40);

#ifdef DO_IPC_OVER_GSC_CO

static int ipc_gscco_txfr(struct platform_device *pdev, u32 offset, u8 *buff, u32 size, bool read)
{
	u8 *gscCo;
	u32 idx;

	if (offset < TSEC_EMEM_START) {
		dev_err(&pdev->dev,
			"Invalid Offset %x less than TSEC_EMEM_START\n", offset);
		return -EINVAL;
	}

	offset -= TSEC_EMEM_START;

	if (!s_ipc_gscco_base || !s_ipc_gscco_size) {
		dev_err(&pdev->dev, "Invalid IPC GSC-CO address/size\n");
		return -EINVAL;
	}
	if (!buff || !size) {
		dev_err(&pdev->dev, "Invalid client buf/size\n");
		return -EINVAL;
	}
	if (offset  > s_ipc_gscco_size || ((offset + size) > s_ipc_gscco_size)) {
		dev_err(&pdev->dev, "Client buf beyond IPC GSC-CO limits\n");
		return -EINVAL;
	}

	gscCo = (u8 *)(s_ipc_gscco_base + offset);
	if (read) {
		for (idx = 0; idx < size; idx++)
			buff[idx] = gscCo[idx];
	} else {
		for (idx = 0; idx < size; idx++)
			gscCo[idx] = buff[idx];
	}

	return 0;
}

#else

static int emem_transfer(struct platform_device *pdev,
	u32 dmem_addr, u8 *buff, u32 size, u8 port, bool copy_from)
{
	u32     num_words;
	u32     num_bytes;
	u32    *pData = (u32 *)buff;
	u32     reg32;
	u32     i;
	u32     ememc_offset = tsec_ememc_r(port);
	u32     ememd_offset = tsec_ememd_r(port);
	u32     emem_start = TSEC_EMEM_START;
	u32     emem_end = TSEC_EMEM_START + TSEC_EMEM_SIZE;

	if (!size || (port >= TSEC_CMD_EMEM_SIZE))
		return -EINVAL;

	if ((dmem_addr < emem_start) || ((dmem_addr + size) > emem_end)) {
		dev_err(&pdev->dev, "CMD: FAILED: copy must be in EMEM aperature [0x%x, 0x%x)\n",
			emem_start, emem_end);
		return -EINVAL;
	}

	dmem_addr -= emem_start;

	num_words = size >> 2;
	num_bytes = size & 0x3; /* MASK_BITS(2); */

	/* (DRF_SHIFTMASK(NV_PGSP_EMEMC_OFFS) |
	 * DRF_SHIFTMASK(NV_PGSP_EMEMC_BLK));
	 */
	reg32 = dmem_addr & 0x00007ffc;

	if (copy_from) {
		/* PSEC_EMEMC EMEMC_AINCR enable
		 * indicate auto increment on read
		 */
		reg32 = reg32 | 0x02000000;
	} else {
		/* PSEC_EMEMC EMEMC_AINCW enable
		 * mark auto-increment on write
		 */
		reg32 = reg32 | 0x01000000;
	}

	host1x_writel(pdev, ememc_offset, reg32);

	for (i = 0; i < num_words; i++) {
		if (copy_from)
			pData[i] = host1x_readl(pdev, ememd_offset);
		else
			host1x_writel(pdev, ememd_offset, pData[i]);
	}

	/* Check if there are leftover bytes to copy */
	if (num_bytes > 0) {
		u32 bytes_copied = num_words << 2;

		/* Read the contents first. If we're copying to the EMEM,
		 * we've set autoincrement on write,
		 * so reading does not modify the pointer.
		 * We can, thus, do a read/modify/write without needing
		 * to worry about the pointer having moved forward.
		 * There is no special explanation needed
		 * if we're copying from the EMEM since this is the last
		 * access to HW in that case.
		 */
		reg32 = host1x_readl(pdev, ememd_offset);
		if (copy_from) {
			for (i = 0; i < num_bytes; i++)
				buff[bytes_copied + i] = ((u8 *)&reg32)[i];
		} else {
			for (i = 0; i < num_bytes; i++)
				((u8 *)&reg32)[i] = buff[bytes_copied + i];

			host1x_writel(pdev, ememd_offset, reg32);
		}
	}

	return 0;
}

#endif

static irqreturn_t tsec_riscv_isr(int irq, void *dev_id)
{
	unsigned long flags;
	struct platform_device *pdev = (struct platform_device *)(dev_id);
	struct nvhost_device_data *pdata = nvhost_get_devdata(pdev);

	spin_lock_irqsave(&pdata->mirq_lock, flags);

	/* logic to clear the interrupt */
	host1x_writel(pdev, flcn_thi_int_stat_r(),
		      flcn_thi_int_stat_clr_f());
	host1x_writel(pdev, flcn_irqsclr_r(),
		      flcn_irqsclr_swgen0_set_f());
	/* Clear RISCV Mask for SWGEN0, so that no more SWGEN0
	 * interrupts will be routed to CCPLEX, it will be re-enabled
	 * by the bottom half
	 */
	host1x_writel(pdev, riscv_irqmclr_r(), riscv_irqmclr_swgen0_set_f());

	spin_unlock_irqrestore(&pdata->mirq_lock, flags);

	return IRQ_WAKE_THREAD;
}

static int ipc_copy_to(u32 head, u8 *pSrc, u32 num_bytes, u32 port)
{
#ifdef DO_IPC_OVER_GSC_CO
	return ipc_gscco_txfr(tsec, head, pSrc, num_bytes, false);
#else
	return emem_transfer(tsec, head, pSrc, num_bytes, port, false);
#endif
}

static int ipc_copy_from(u32 tail, u8 *pdst, u32 num_bytes, u32 port)
{
#ifdef DO_IPC_OVER_GSC_CO
	return ipc_gscco_txfr(tsec, tail, pdst, num_bytes, true);
#else
	return emem_transfer(tsec, tail, pdst, num_bytes, port, true);
#endif
}

/* gspQueueCmdValidate */
static int validate_cmd(union RM_FLCN_CMD *flcn_cmd,  u32 queue_id)
{
	if (flcn_cmd == NULL)
		return -EINVAL;

	if ((flcn_cmd->cmdGen.hdr.size < RM_FLCN_QUEUE_HDR_SIZE) ||
		(queue_id != RM_DPU_CMDQ_LOG_ID) ||
		(flcn_cmd->cmdGen.hdr.unitId  >= RM_GSP_UNIT_END)) {
		return -EINVAL;
	}

	return 0;
}

/*
 * cmd - Falcon command
 * queue_id - ID of queue (usually 0)
 * callback_func - callback func to caller on command completion
 *
 */
int nvhost_tsec_send_cmd(void *cmd, u32 queue_id,
	callback_func_t cb_func, void *cb_ctx)
{
	int i;
	int placeholder;
	u32 head;
	u32 tail;
	u8 cmd_size;
	u32 cmd_size_aligned;
	u32 cmdq_head_base;
	u32 cmdq_tail_base;
	u32 cmdq_head_stride;
	u32 cmdq_tail_stride;
	u32 cmdq_size = 0x80;
	static u32 cmdq_start;
	union RM_FLCN_CMD *flcn_cmd;
	struct RM_FLCN_QUEUE_HDR hdr;

	if (!s_init_msg_rcvd) {
		pr_err_once("TSEC RISCV hasn't booted successfully\n");
		return -ENODEV;
	}

	cmdq_head_base = tsec_queue_head_r(0);
	cmdq_head_stride = tsec_queue_head_r(1) - tsec_queue_head_r(0);
	cmdq_tail_base = tsec_queue_tail_r(0);
	cmdq_tail_stride = tsec_queue_tail_r(1) - tsec_queue_tail_r(0);

	for (i = 0; !cmdq_start && i < TSEC_POLL_TIME_MS; i++) {
		cmdq_start = host1x_readl(tsec, (cmdq_tail_base +
						(queue_id * cmdq_tail_stride)));
		if (!cmdq_start)
			udelay(TSEC_TAIL_POLL_TIME);
	}

	if (!cmdq_start) {
		dev_warn(&tsec->dev, "cmdq_start=0x%x\n", cmdq_start);
		return -ENODEV;
	}

	if (validate_cmd(cmd, queue_id != 0)) {
		dev_dbg(&tsec->dev, "CMD: %s: %d Invalid command\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	flcn_cmd = (union RM_FLCN_CMD *)cmd;
	plat_acquire_comms_mutex();
	if (s_callbacks[flcn_cmd->cmdGen.hdr.unitId].cb_func) {
		plat_release_comms_mutex();
		dev_err(&tsec->dev, "CMD: %s: %d More than 1 outstanding cmd for unit 0x%x\n",
			__func__, __LINE__, flcn_cmd->cmdGen.hdr.unitId);
		return -EINVAL;
	}
	plat_release_comms_mutex();
	cmd_size = flcn_cmd->cmdGen.hdr.size;
	placeholder = ALIGN(cmd_size, 4);
	if (placeholder < 0) {
		dev_err(&tsec->dev, "Alignment found to be negative\n");
		return -EINVAL;
	}
	cmd_size_aligned = (unsigned int) placeholder;
	head = host1x_readl(tsec,
		(cmdq_head_base + (queue_id * cmdq_head_stride)));

check_space:
	tail = host1x_readl(tsec, (cmdq_tail_base +
				   (queue_id * cmdq_tail_stride)));
	if (head < cmdq_start || tail < cmdq_start)
		pr_err("***** head/tail invalid, h=0x%x,t=0x%x\n", head, tail);
	if (UINT_MAX - head < cmd_size_aligned) {
		pr_err("addition of head and offset wraps\n");
		return -EINVAL;
	}
	if (tail > head) {
		if ((head + cmd_size_aligned) < tail)
			goto enqueue;
		udelay(TSEC_TAIL_POLL_TIME);
		goto check_space;
	} else {

		if ((head + cmd_size_aligned) < (cmdq_start + cmdq_size)) {
			goto enqueue;
		} else {
			if ((cmdq_start + cmd_size_aligned) < tail) {
				goto rewind;
			} else {
				udelay(TSEC_TAIL_POLL_TIME);
				goto check_space;
			}
		}
	}

rewind:
	hdr.unitId = RM_GSP_UNIT_REWIND;
	hdr.size = RM_FLCN_QUEUE_HDR_SIZE;
	hdr.ctrlFlags = 0;
	hdr.seqNumId = 0;
	if (ipc_copy_to(head, (u8 *)&hdr, hdr.size, 0))
		return -EINVAL;
	head = cmdq_start;
	host1x_writel(tsec, (cmdq_head_base +
			     (queue_id * cmdq_head_stride)), head);
	pr_debug("CMDQ: rewind h=%x,t=%x\n", head, tail);

enqueue:
	plat_acquire_comms_mutex();
	s_callbacks[flcn_cmd->cmdGen.hdr.unitId].cb_func = cb_func;
	s_callbacks[flcn_cmd->cmdGen.hdr.unitId].cb_ctx  = cb_ctx;
	plat_release_comms_mutex();
	if (ipc_copy_to(head, (u8 *)flcn_cmd, cmd_size, 0)) {
		plat_acquire_comms_mutex();
		s_callbacks[flcn_cmd->cmdGen.hdr.unitId].cb_func = NULL;
		s_callbacks[flcn_cmd->cmdGen.hdr.unitId].cb_ctx  = NULL;
		plat_release_comms_mutex();
		return -EINVAL;
	}
	head += cmd_size_aligned;
	host1x_writel(tsec, (cmdq_head_base +
		(queue_id * cmdq_head_stride)), head);

	dev_dbg(&tsec->dev, "Cmd sent to unit 0x%x\n", flcn_cmd->cmdGen.hdr.unitId);

	return 0;
}
EXPORT_SYMBOL(nvhost_tsec_send_cmd);

#ifdef DO_IPC_OVER_GSC_CO
#define TSEC_BOOT_POLL_TIME_US     (100000)
#define TSEC_BOOT_POLL_INTERVAL_US (50)
#define TSEC_BOOT_POLL_COUNT       (TSEC_BOOT_POLL_TIME_US / TSEC_BOOT_POLL_INTERVAL_US)
#define TSEC_BOOT_FLAG_MAGIC       (0xA5A5A5A5)
#endif

#ifdef DO_IPC_OVER_GSC_CO
static u32 tsec_get_boot_flag(void)
{
	struct TSEC_BOOT_INFO *bootInfo = (struct TSEC_BOOT_INFO *)(s_ipc_gscco_base);

	if (!s_ipc_gscco_base || !s_ipc_gscco_size) {
		dev_err(&tsec->dev, "%s: Invalid GSC-CO address/size\n", __func__);
		return 0;
	} else {
		return bootInfo->bootFlag;
	}
}

static void tsec_reset_boot_flag(void)
{
	struct TSEC_BOOT_INFO *bootInfo = (struct TSEC_BOOT_INFO *)(s_ipc_gscco_base);

	if (!s_ipc_gscco_base || !s_ipc_gscco_size) {
		dev_err(&tsec->dev, "%s: Invalid GSC-CO address/size\n", __func__);
	} else {
		bootInfo->bootFlag = 0;
	}
}
#endif

static void drain_msg(bool invoke_cb)
{
	int i;
	u32 tail = 0;
	u32 head = 0;
	u32 queue_id = 0;
	u32 msgq_head_base;
	u32 msgq_tail_base;
	u32 msgq_head_stride;
	u32 msgq_tail_stride;
	static u32 msgq_start;
	struct RM_FLCN_MSG_GSP gsp_msg;
	struct RM_GSP_INIT_MSG_GSP_INIT *gsp_init_msg;
	callback_func_t cb_func;
	void *cb_ctx;

#ifndef TSEC_RM_ON_DCE
	return;
#endif

	msgq_head_base = tsec_msgq_head_r(TSEC_MSG_QUEUE_PORT);
	msgq_tail_base = tsec_msgq_tail_r(TSEC_MSG_QUEUE_PORT);

	msgq_head_stride = tsec_msgq_head_r(1) - tsec_msgq_head_r(0);
	msgq_tail_stride = tsec_msgq_tail_r(1) - tsec_msgq_tail_r(0);
	gsp_init_msg = (struct RM_GSP_INIT_MSG_GSP_INIT *) &gsp_msg.msg;

	for (i = 0; !msgq_start && i < TSEC_POLL_TIME_MS; i++) {
		msgq_start = host1x_readl(tsec, (msgq_tail_base +
						 (msgq_tail_stride * queue_id)));
		if (!msgq_start)
			udelay(TSEC_TAIL_POLL_TIME);
	}

	if (!msgq_start)
		dev_warn(&tsec->dev, "msgq_start=0x%x\n", msgq_start);

	for (i = 0; i < TSEC_POLL_TIME_MS; i++) {
		tail = host1x_readl(tsec, (msgq_tail_base +
					   (msgq_tail_stride * queue_id)));
		head = host1x_readl(tsec, (msgq_head_base +
					   (msgq_head_stride * queue_id)));
		if (tail != head)
			break;
		udelay(TSEC_TAIL_POLL_TIME);
	}

	if (head == 0 || tail == 0) {
		dev_err(&tsec->dev, "Err: Invalid MSGQ head=0x%x, tail=0x%x\n",
			head, tail);
		goto exit;
	}

	if (tail == head) {
		dev_dbg(&tsec->dev, "Empty MSGQ tail(0x%x): 0x%x head(0x%x): 0x%x\n",
			(msgq_tail_base + (msgq_tail_stride * queue_id)),
			tail,
			(msgq_head_base + (msgq_head_stride * queue_id)),
			head);
		goto exit;
	}

	while (tail != head) {

		/* read header */
		ipc_copy_from(tail, (u8 *)&gsp_msg.hdr,
			RM_FLCN_QUEUE_HDR_SIZE, 0);
		pr_debug("seqNumId=%d\n", gsp_msg.hdr.seqNumId);
		/* copy msg body */
		if (gsp_msg.hdr.size > RM_FLCN_QUEUE_HDR_SIZE) {
			ipc_copy_from(tail + RM_FLCN_QUEUE_HDR_SIZE, (u8 *)&gsp_msg.msg,
				gsp_msg.hdr.size - RM_FLCN_QUEUE_HDR_SIZE, 0);
		}

		if (gsp_msg.hdr.unitId == RM_GSP_UNIT_INIT) {
			dev_dbg(&tsec->dev, "MSGQ: %s(%d) init msg\n",
				__func__, __LINE__);
			if (gsp_init_msg->numQueues < 2) {
				dev_err(&tsec->dev, "MSGQ: Initing less queues than expected %d\n",
					gsp_init_msg->numQueues);
				goto FAIL;
			}
#ifdef DO_IPC_OVER_GSC_CO
			/* DCE can access the GSC-CO hence can poll for the Tsec
			 * booted flag and also reset it
			 */
			for (i = 0; i < TSEC_BOOT_POLL_COUNT; i++) {
				if (tsec_get_boot_flag() == TSEC_BOOT_FLAG_MAGIC)
					break;
				udelay(TSEC_BOOT_POLL_INTERVAL_US);
			}
			if (i >= TSEC_BOOT_POLL_COUNT) {
				dev_err(&tsec->dev, "Tsec GSC-CO Boot Flag not set\n");
				goto FAIL;
			} else {
				tsec_reset_boot_flag();
				dev_dbg(&tsec->dev, "Tsec GSC-CO Boot Flag reset done\n");
			}
#endif
			memcpy(&s_gsp_init_msg.hdr, &gsp_msg.hdr, RM_FLCN_QUEUE_HDR_SIZE);
			memcpy(&s_gsp_init_msg.msg, &gsp_msg.msg,
				gsp_msg.hdr.size - RM_FLCN_QUEUE_HDR_SIZE);

			/* Invoke the callback and clear it */
			plat_acquire_comms_mutex();
			s_init_msg_rcvd = true;
			if (invoke_cb) {
				cb_func = s_callbacks[gsp_msg.hdr.unitId].cb_func;
				cb_ctx  = s_callbacks[gsp_msg.hdr.unitId].cb_ctx;
				s_callbacks[gsp_msg.hdr.unitId].cb_func = NULL;
				s_callbacks[gsp_msg.hdr.unitId].cb_ctx = NULL;
			}
			plat_release_comms_mutex();
			if (cb_func && invoke_cb) {
				cb_func(cb_ctx, (void *)&gsp_msg);
			}
		} else if (gsp_msg.hdr.unitId < RM_GSP_UNIT_END) {
			dev_dbg(&tsec->dev, "Msg received from unit 0x%x\n", gsp_msg.hdr.unitId);

			if (gsp_msg.hdr.unitId == RM_GSP_UNIT_HDCP22WIRED) {
				dev_dbg(&tsec->dev, "MSGQ: %s(%d) RM_GSP_UNIT_HDCP22WIRED\n",
					__func__, __LINE__);
			} else if (gsp_msg.hdr.unitId == RM_GSP_UNIT_REWIND) {
				tail = msgq_start;
				host1x_writel(tsec, (msgq_tail_base +
						     (msgq_tail_stride * queue_id)), tail);
				head = host1x_readl(tsec,
					(msgq_head_base + (msgq_head_stride * queue_id)));
				pr_debug("MSGQ tail rewinded\n");
				continue;
			} else {
				dev_dbg(&tsec->dev, "MSGQ: %s(%d) what msg could it be 0x%x?\n",
					__func__, __LINE__, gsp_msg.hdr.unitId);
			}

			/* Invoke the callback and clear it */
			if (invoke_cb) {
				plat_acquire_comms_mutex();
				cb_func = s_callbacks[gsp_msg.hdr.unitId].cb_func;
				cb_ctx  = s_callbacks[gsp_msg.hdr.unitId].cb_ctx;
				s_callbacks[gsp_msg.hdr.unitId].cb_func = NULL;
				s_callbacks[gsp_msg.hdr.unitId].cb_ctx = NULL;
				plat_release_comms_mutex();
				if (cb_func)
					cb_func(cb_ctx, (void *)&gsp_msg);
			}
		} else {
			dev_err(&tsec->dev, "MSGQ: %s(%d) invalid msg uintId 0x%x?\n",
				__func__, __LINE__, gsp_msg.hdr.unitId);
		}

FAIL:
		tail += ALIGN(gsp_msg.hdr.size, 4);
		head = host1x_readl(tsec,
			(msgq_head_base + (msgq_head_stride * queue_id)));
		host1x_writel(tsec,
			(msgq_tail_base + (msgq_tail_stride * queue_id)), tail);
	}

exit:
	return;
}

static irqreturn_t process_msg(int irq, void *args)
{
	drain_msg(true);

	/* Set RISCV Mask for SWGEN0, so that it is re-enabled
	 * and if it is pending the CCPLEX will be interrupted
	 * by this the top half
	 */
	host1x_writel(tsec, riscv_irqmset_r(), riscv_irqmset_swgen0_set_f());

	return IRQ_HANDLED;
}

static void invoke_init_cb(void *unused)
{
	callback_func_t cb_func;
	void *cb_ctx;

	plat_acquire_comms_mutex();
	cb_func = s_callbacks[RM_GSP_UNIT_INIT].cb_func;
	cb_ctx  = s_callbacks[RM_GSP_UNIT_INIT].cb_ctx;
	s_callbacks[RM_GSP_UNIT_INIT].cb_func = NULL;
	s_callbacks[RM_GSP_UNIT_INIT].cb_ctx  = NULL;
	plat_release_comms_mutex();

	if (cb_func)
		cb_func(cb_ctx, &s_gsp_init_msg);
}

int nvhost_tsec_set_init_cb(callback_func_t cb_func, void *cb_ctx)
{
	int err = 0;

	plat_acquire_comms_mutex();

	if (s_callbacks[RM_GSP_UNIT_INIT].cb_func) {
		dev_err(&tsec->dev, "%s: %d: INIT unit cb_func already set\n",
			__func__, __LINE__);
		err = -EINVAL;
		goto FAIL;
	}
	if (!cb_func) {
		dev_err(&tsec->dev, "%s: %d: Init CallBack NULL\n",
			__func__, __LINE__);
		err = -EINVAL;
		goto FAIL;
	}

	s_callbacks[RM_GSP_UNIT_INIT].cb_func = cb_func;
	s_callbacks[RM_GSP_UNIT_INIT].cb_ctx = cb_ctx;

	if (s_init_msg_rcvd) {
		dev_dbg(&tsec->dev, "Init msg already received invoking callback\n");
		plat_queue_work(invoke_init_cb, NULL);
	}
#ifdef DO_IPC_OVER_GSC_CO
	else if (tsec_get_boot_flag() == TSEC_BOOT_FLAG_MAGIC) {
		dev_dbg(&tsec->dev, "Doorbell missed tsec booted first, invoke init callback\n");
		/* Interrupt missed as tsec booted first
		 * Explicitly call drain_msg
		 */
		plat_release_comms_mutex();
		drain_msg(false);
		plat_acquire_comms_mutex();
		/* Init message is drained now, hence queue the work item to invoke init callback*/
		plat_queue_work(invoke_init_cb, NULL);
	}
#endif

FAIL:
	plat_release_comms_mutex();
	return err;
}
EXPORT_SYMBOL(nvhost_tsec_set_init_cb);

static void tsec_poweron_handler(struct work_struct *work)
{
	struct tsec_t23x_device *tsec_dev;
	struct nvhost_device_data *pdata;
	const struct firmware *tsec_fw_desc;

	tsec_dev = container_of(to_delayed_work(work), struct tsec_t23x_device,
			    poweron_work);
	pdata = nvhost_get_devdata(tsec_dev->pdev);
	tsec_fw_desc = nvhost_client_request_firmware(tsec_dev->pdev, pdata->riscv_desc_bin, false);
	tsec_dev->fwreq_duration_ms += tsec_dev->fwreq_retry_interval_ms;

	if (tsec_fw_desc) {
		dev_info(&(tsec_dev->pdev->dev),
			"tsec fw req success in %d ms\n",
			tsec_dev->fwreq_duration_ms);
		release_firmware(tsec_fw_desc);
		nvhost_module_busy(tsec_dev->pdev);
	} else if (tsec_dev->fwreq_duration_ms < tsec_dev->fwreq_fail_threshold_ms) {
		dev_info(&(tsec_dev->pdev->dev),
			"retry tsec fw req, total retry duration %d ms\n",
			tsec_dev->fwreq_duration_ms);
		schedule_delayed_work(&tsec_dev->poweron_work,
			msecs_to_jiffies(tsec_dev->fwreq_retry_interval_ms));
	} else {
		dev_err(&(tsec_dev->pdev->dev),
			"tsec boot failure, fw not available within %d ms\n",
			tsec_dev->fwreq_fail_threshold_ms);
	}
}

int nvhost_t23x_tsec_intr_init(struct platform_device *pdev)
{
	int ret = 0;
	struct nvhost_device_data *pdata = nvhost_get_devdata(pdev);
	struct tsec_t23x_device *tsec_dev = NULL;

	tsec_dev = devm_kzalloc(&pdev->dev, sizeof(*tsec_dev), GFP_KERNEL);
	if (!tsec_dev)
		return -ENOMEM;
	tsec_dev->pdev = pdev;
	INIT_DELAYED_WORK(&tsec_dev->poweron_work, tsec_poweron_handler);
	INIT_WORK(&tsec_dev->initmsg_work, initmsg_work_handler);
	tsec_dev->fwreq_retry_interval_ms = 10;
	tsec_dev->fwreq_duration_ms = 0;
	tsec_dev->fwreq_fail_threshold_ms = tsec_dev->fwreq_retry_interval_ms * 100;
	pdata->private_data = tsec_dev;

	tsec = pdev;
	pdata->irq = platform_get_irq(pdev, 0);
	if (pdata->irq < 0) {
		dev_err(&pdev->dev, "CMD: failed to get irq %d\n", -pdata->irq);
		devm_kfree(&pdev->dev, tsec_dev);
		return -ENXIO;
	}

	spin_lock_init(&pdata->mirq_lock);
	ret = request_threaded_irq(pdata->irq, tsec_riscv_isr,
				   process_msg, 0, "tsec_riscv_irq", pdev);
	if (ret) {
		dev_err(&pdev->dev, "CMD: failed to request irq %d\n", ret);
		devm_kfree(&pdev->dev, tsec_dev);
		return ret;
	}

	/* keep irq disabled */
	disable_irq(pdata->irq);

	/* schedule work item to turn on tsec */
	schedule_delayed_work(&tsec_dev->poweron_work,
		msecs_to_jiffies(tsec_dev->fwreq_retry_interval_ms));

	return 0;
}

void *nvhost_tsec_alloc_payload_mem(size_t size, dma_addr_t *dma_addr)
{
	void *cpu_addr;

	if (!size || !dma_addr)
		return ERR_PTR(-EINVAL);

	cpu_addr = dma_alloc_attrs(&tsec->dev, size, dma_addr, GFP_KERNEL, 0);

	if (!cpu_addr)
		return ERR_PTR(-ENOMEM);

	*dma_addr |= TSEC_SMMU_IDX;

	return cpu_addr;
}
EXPORT_SYMBOL(nvhost_tsec_alloc_payload_mem);

void nvhost_tsec_free_payload_mem(size_t size, void *cpu_addr, dma_addr_t dma_addr)
{
	dma_addr &= ~TSEC_SMMU_IDX;
	dma_free_attrs(&tsec->dev, size, cpu_addr, dma_addr, 0);
}
EXPORT_SYMBOL(nvhost_tsec_free_payload_mem);

void *nvhost_tsec_get_gscco_page(u32 page_number, u32 *gscco_offset)
{
#ifdef DO_IPC_OVER_GSC_CO
	u8 *page_va;

	if (!s_ipc_gscco_page_base || (page_number >= s_ipc_gscco_page_count)) {
		dev_err(&tsec->dev,
			"%s: No reserved memory for Page %d\n",
			__func__, page_number);
		return NULL;
	}

	page_va = (u8 *)s_ipc_gscco_page_base;
	page_va += (page_number * s_ipc_gscco_page_size);
	if (gscco_offset) {
		*gscco_offset =
			(u32)((s_ipc_gscco_page_base - s_ipc_gscco_base) +
			(page_number * s_ipc_gscco_page_size));
	}
	return page_va;
#else
	dev_err(&tsec->dev, "%s: IPC over GSC-CO not enabled\n", __func__);
	return NULL;
#endif
}
EXPORT_SYMBOL(nvhost_tsec_get_gscco_page);

int nvhost_tsec_cmdif_open(void)
{
	return nvhost_module_busy(tsec);
}

void nvhost_tsec_cmdif_close(void)
{
	nvhost_module_idle(tsec);
}
