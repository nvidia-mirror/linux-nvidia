/*
 * Tegra 19x SoC-specific mcerr code.
 *
 * Copyright (c) 2017-2022, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#define pr_fmt(fmt) "mc-err: " fmt

#include <linux/bitops.h>
#include <linux/of.h>
#include <linux/platform/tegra/mc-regs-t19x.h>
#include <linux/platform/tegra/mcerr.h>
#include <dt-bindings/memory/tegra-swgroup.h>
#include <linux/interrupt.h>

/*** Auto generated by `mcp.pl'. Do not modify! ***/

static struct mc_client mc_clients[] = {
	client("ptc", "csr_ptcr", INVALID),
	client("miu", "csr_miu7r", INVALID),
	client("miu", "csw_miu7w", INVALID),
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	client("hda", "csr_hdar", INVALID),
	client("hc", "csr_host1xdmar", INVALID),
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	client("nvenc", "csr_nvencsrd", INVALID),
	dummy_client,
	dummy_client,
	client("sata", "csr_satar", INVALID),
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	client("mpcore", "csr_mpcorer", INVALID),
	dummy_client,
	dummy_client,
	dummy_client,
	client("nvenc", "csw_nvencswr", INVALID),
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	client("hda", "csw_hdaw", INVALID),
	dummy_client,
	dummy_client,
	dummy_client,
	client("mpcore", "csw_mpcorew", INVALID),
	dummy_client,
	dummy_client,
	dummy_client,
	client("sata", "csw_sataw", INVALID),
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	client("ispa", "csr_ispra", INVALID),
	client("ispfal", "csr_ispfalr", INVALID),
	client("isp2", "csw_ispwa", INVALID),
	client("isp2a", "csw_ispwb", INVALID),
	dummy_client,
	dummy_client,
	client("xusb_host", "csr_xusb_hostr", INVALID),
	client("xusb_host", "csw_xusb_hostw", INVALID),
	client("xusb_dev", "csr_xusb_devr", INVALID),
	client("xusb_dev", "csw_xusb_devw", INVALID),
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	client("tsec", "csr_tsecsrd", INVALID),
	client("tsec", "csw_tsecswr", INVALID),
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	client("sdmmc1a", "csr_sdmmcra", INVALID),
	dummy_client,
	client("sdmmc3a", "csr_sdmmcr", INVALID),
	client("sdmmc4a", "csr_sdmmcrab", INVALID),
	client("sdmmc1a", "csw_sdmmcwa", INVALID),
	dummy_client,
	client("sdmmc3a", "csw_sdmmcw", INVALID),
	client("sdmmc4a", "csw_sdmmcwab", INVALID),
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	client("vic", "csr_vicsrd", INVALID),
	client("vic", "csw_vicswr", INVALID),
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	client("vi", "csw_viw", INVALID),
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	client("nvdec", "csr_nvdecsrd", INVALID),
	client("nvdec", "csw_nvdecswr", INVALID),
	client("ape", "csr_aper", INVALID),
	client("ape", "csw_apew", INVALID),
	dummy_client,
	dummy_client,
	client("nvjpg", "csr_nvjpgsrd", INVALID),
	client("nvjpg", "csw_nvjpgswr", INVALID),
	client("se", "csr_sesrd", INVALID),
	client("se", "csw_seswr", INVALID),
	client("axiap", "csr_axiapr", INVALID),
	client("axiap", "csw_axiapw", INVALID),
	client("etr", "csr_etrr", INVALID),
	client("etr", "csw_etrw", INVALID),
	client("tsecb", "csr_tsecsrdb", INVALID),
	client("tsecb", "csw_tsecswrb", INVALID),
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	client("axis", "csr_axisr", INVALID),
	client("axis", "csw_axisw", INVALID),
	client("eqos", "csr_eqosr", INVALID),
	client("eqos", "csw_eqosw", INVALID),
	client("ufshc", "csr_ufshcr", INVALID),
	client("ufshc", "csw_ufshcw", INVALID),
	client("nvdisplay", "csr_nvdisplayr", INVALID),
	client("bpmp", "csr_bpmpr", INVALID),
	client("bpmp", "csw_bpmpw", INVALID),
	client("bpmpdma", "csr_bpmpdmar", INVALID),
	client("bpmpdma", "csw_bpmpdmaw", INVALID),
	client("aon", "csr_aonr", INVALID),
	client("aon", "csw_aonw", INVALID),
	client("aondma", "csr_aondmar", INVALID),
	client("aondma", "csw_aondmaw", INVALID),
	client("sce", "csr_scer", INVALID),
	client("sce", "csw_scew", INVALID),
	client("scedma", "csr_scedmar", INVALID),
	client("scedma", "csw_scedmaw", INVALID),
	client("apedma", "csr_apedmar", INVALID),
	client("apedma", "csw_apedmaw", INVALID),
	client("nvdisplay", "csr_nvdisplayr1", INVALID),
	client("vic", "csr_vicsrd1", INVALID),
	client("nvdec", "csr_nvdecsrd1", INVALID),
	dummy_client,
	dummy_client,
	client("miu", "csr_miu0r", INVALID),
	client("miu", "csw_miu0w", INVALID),
	client("miu", "csr_miu1r", INVALID),
	client("miu", "csw_miu1w", INVALID),
	dummy_client,
	dummy_client,
	dummy_client,
	dummy_client,
	client("miu", "csr_miu2r", INVALID),
	client("miu", "csw_miu2w", INVALID),
	client("miu", "csr_miu3r", INVALID),
	client("miu", "csw_miu3w", INVALID),
	client("miu", "csr_miu4r", INVALID),
	client("miu", "csw_miu4w", INVALID),
	client("NA", "csr_dpmur", INVALID),
	client("NA", "csw_dpmuw", INVALID),
	client("NA", "csr_nvl0r", INVALID),
	client("NA", "csw_nvl0w", INVALID),
	client("NA", "csr_nvl1r", INVALID),
	client("NA", "csw_nvl1w", INVALID),
	client("NA", "csr_nvl2r", INVALID),
	client("NA", "csw_nvl2w", INVALID),
	client("vifal", "csr_vifalr", INVALID),
	client("vifal", "csw_vifalw", INVALID),
	client("dlaa", "csr_dla0rda", INVALID),
	client("dlaa", "csr_dla0falrdb", INVALID),
	client("dlaa", "csw_dla0wra", INVALID),
	client("dlaa", "csw_dla0falwrb", INVALID),
	client("dla1a", "csr_dla1rda", INVALID),
	client("dla1a", "csr_dla1falrdb", INVALID),
	client("dla1a", "csw_dla1wra", INVALID),
	client("dla1a", "csw_dla1falwrb", INVALID),
	client("pva0a", "csr_pva0rda", INVALID),
	client("pva0b", "csr_pva0rdb", INVALID),
	client("pva0c", "csr_pva0rdc", INVALID),
	client("pva0a", "csw_pva0wra", INVALID),
	client("pva0b", "csw_pva0wrb", INVALID),
	client("pva0c", "csw_pva0wrc", INVALID),
	client("pva1a", "csr_pva1rda", INVALID),
	client("pva1b", "csr_pva1rdb", INVALID),
	client("pva1c", "csr_pva1rdc", INVALID),
	client("pva1a", "csw_pva1wra", INVALID),
	client("pva1b", "csw_pva1wrb", INVALID),
	client("pva1c", "csw_pva1wrc", INVALID),
	client("rce", "csr_rcer", INVALID),
	client("rce", "csw_rcew", INVALID),
	client("rcedma", "csr_rcedmar", INVALID),
	client("rcedma", "csw_rcedmaw", INVALID),
	client("nvencb", "csr_nvenc1srd", INVALID),
	client("nvencb", "csw_nvenc1swr", INVALID),
	client("pcie0a", "csr_pcie0r", INVALID),
	client("pcie0a2", "csw_pcie0w", INVALID),
	client("pcie", "csr_pcie1r", INVALID),
	client("pcie", "csw_pcie1w", INVALID),
	client("pcie2a", "csr_pcie2ar", INVALID),
	client("pcie2a", "csw_pcie2aw", INVALID),
	client("pcie3", "csr_pcie3r", INVALID),
	client("pcie3a", "csw_pcie3w", INVALID),
	client("pcie4a", "csr_pcie4r", INVALID),
	client("pcie4a", "csw_pcie4w", INVALID),
	client("pcie5a", "csr_pcie5r", INVALID),
	client("pcie5a", "csw_pcie5w", INVALID),
	client("ispfal", "csw_ispfalw", INVALID),
	client("NA", "csr_nvl3r", INVALID),
	client("NA", "csw_nvl3w", INVALID),
	client("NA", "csr_nvl4r", INVALID),
	client("NA", "csw_nvl4w", INVALID),
	client("dlaa", "csr_dla0rda1", INVALID),
	client("dla1a", "csr_dla1rda1", INVALID),
	client("pva0a", "csr_pva0rda1", INVALID),
	client("pva0b", "csr_pva0rdb1", INVALID),
	client("pva1a", "csr_pva1rda1", INVALID),
	client("pva1b", "csr_pva1rdb1", INVALID),
	client("pcie5a", "csr_pcie5r1", INVALID),
	client("nvenc", "csr_nvencsrd1", INVALID),
	client("nvencb", "csr_nvenc1srd1", INVALID),
	client("ispa", "csr_ispra1", INVALID),
	client("pcie0a", "csr_pcie0r1", INVALID),
	client("NA", "csr_nvl0rhp", INVALID),
	client("NA", "csr_nvl1rhp", INVALID),
	client("NA", "csr_nvl2rhp", INVALID),
	client("NA", "csr_nvl3rhp", INVALID),
	client("NA", "csr_nvl4rhp", INVALID),
	client("nvdec2a", "csr_nvdec1srd", INVALID),
	client("nvdec2a", "csr_nvdec1srd1", INVALID),
	client("nvdec2a", "csw_nvdec1swr", INVALID),
	client("miu", "csr_miu5r", INVALID),
	client("miu", "csw_miu5w", INVALID),
	client("miu", "csr_miu6r", INVALID),
	client("miu", "csw_miu6w", INVALID),
};
static int mc_client_last = ARRAY_SIZE(mc_clients) - 1;
/*** Done. ***/

static u32 mc_channel = MC_BROADCAST_CHANNEL;
static u32 global_intstatus;
static u32 global_intstatus_1;
static u32 slice_int_status;
static u32 ch_int_status;
static u32 hubc_int_status;
static u32 sbs_int_status;
static u32 hub_int_status;

static const char *intr_info[] = {
	NULL,		/* Bit 0 */
	NULL,
	NULL,
	NULL,
	NULL,		/* Bit 4 */
	NULL,
	"decerr-emem",
	NULL,
	"secerr",	/* Bit 8 */
	"arb-emem",
	NULL,
	NULL,
	"decerr-vpr",	/* Bit 12 */
	"decerr-sec",
	NULL,
	NULL,
	"decerr-mts",	/* Bit 16 */
	"decerr-gsc",
	"scrub-ecc",
	"wcam-err",
	"decerr-route",	/* Bit 20 */
	NULL,
	NULL,
	NULL,
	NULL,		/* Bit 24 */
	NULL,
	NULL,
	NULL,
	NULL,		/* Bit 28 */
	NULL,
	NULL,
	NULL,
};

enum {
	/* GLOBAL_INTSTATUS_0 bits */
	GIS_CH0 = 0,
	GIS_CH1 = 1,
	GIS_CH2 = 2,
	GIS_CH3 = 3,
	GIS_CH4 = 4,
	GIS_CH5 = 5,
	GIS_CH6 = 6,
	GIS_CH7 = 7,
	GIS_SLICE0 = 8,
	GIS_SLICE1 = 9,
	GIS_SLICE2 = 10,
	GIS_SLICE3 = 11,
	GIS_HUB0 = 16,
	GIS_HUB1 = 17,
	GIS_HUB2 = 18,
	GIS_HUB3 = 19,
	GIS_nvlink0 = 20,
	GIS_nvlink1 = 21,
	GIS_nvlink2 = 22,
	GIS_nvlink3 = 23,
	GIS_nvlink4 = 24,
	GIS_HUBC = 25,
	GIS_SBS = 26,
	GIS_CH_MASK = 0xFF,
	GIS_SLICE_MASK = 0xF00,
	GIS_HUB_MASK = 0xF0000,
	GIS_NVLINK_MASK = 0x1F00000,

	/* GLOBAL_INTSTATUS_1 bits */
	GIS_1_CH8 = 0,
	GIS_1_CH9 = 1,
	GIS_1_CH10 = 2,
	GIS_1_CH11 = 3,
	GIS_1_CH12 = 4,
	GIS_1_CH13 = 5,
	GIS_1_CH14 = 6,
	GIS_1_CH15 = 7,
	GIS_1_CH_MASK = 0xFF,

	/* MC Intr clear enums */
	INTSTATUS_CLEAR = 0x00133340,
	HUBC_INTSTATUS_CLEAR = 0x00000001,
	HUB_INTSTATUS_CLEAR = 0x00000003,
	GLOBAL_INTSTATUS_CLEAR = 0x07FF0FFF,
	GLOBAL_INTSTATUS_1_CLEAR = 0x000000FF,
	CH_INTSTATUS_CLEAR = 0x00080200,
	SBS_INTSTATUS_CLEAR = 0x00000007,

	/* MC Intr bits */
	MC_INT_DECERR_MTS = (1<<16),
	MC_INT_WCAM_ERR = (1<<19),
	MC_INT_DECERR_ROUTE_SANITY = (1<<20),
	MC_HUB_INT_HUB_COALESCER_ERR = (1<<1),
	MC_HUB_INT_ILLEGAL_HUB_REQ = (1<<0),
	MC_HUBC_INT_SCRUB_ECC_WR_ACK = (1 << 0),

	MC_SBS_INT_FILL_FIFO_ISO_OF = (1<<0),
	MC_SBS_INT_FILL_FIFO_SISO_OF = (1<<1),
	MC_SBS_INT_FILL_FIFO_NISO_OF = (1<<2),

	MC_ERR_STATUS_ADR_HI_BITS = (0xFF << 20)
};

/* reported in MC_INTSTATUS_0 */
static const struct mc_error slice_mc_errors[] = {
	MC_ERR_HI(MC_INT_DECERR_EMEM,
	       "EMEM address decode error",
	       0, MC_ERR_STATUS, MC_ERR_ADR, MC_ERR_ADR_HI),
	MC_ERR_HI(MC_INT_SECURITY_VIOLATION,
	       "non secure access to secure region",
	       0, MC_ERR_STATUS, MC_ERR_ADR, MC_ERR_ADR_HI),
	MC_ERR(MC_INT_DECERR_VPR,
	       "MC request violates VPR requirements",
	       E_VPR, MC_ERR_VPR_STATUS, MC_ERR_VPR_ADR),
	MC_ERR(MC_INT_SECERR_SEC,
	       "MC request violated SEC carveout requirements",
	       0, MC_ERR_SEC_STATUS, MC_ERR_SEC_ADR),
	MC_ERR(MC_INT_DECERR_MTS,
	       "MTS carveout access violation",
	       0, MC_ERR_MTS_STATUS, MC_ERR_MTS_ADR),
	MC_ERR_GSC(MC_INT_DECERR_GENERALIZED_CARVEOUT,
	       "GSC access violation", 0,
	       MC_ERR_GENERALIZED_CARVEOUT_STATUS,
	       MC_ERR_GENERALIZED_CARVEOUT_ADR,
	       MC_ERR_GENERALIZED_CARVEOUT_STATUS_1),
	MC_ERR(MC_INT_DECERR_ROUTE_SANITY,
		"Route Sanity error", 0,
		MC_ERR_ROUTE_SANITY_STATUS,
		MC_ERR_ROUTE_SANITY_ADR),

	/* combination interrupts */
	MC_ERR_HI(MC_INT_DECERR_EMEM | MC_INT_SECURITY_VIOLATION,
	       "non secure access to secure region",
	       0, MC_ERR_STATUS, MC_ERR_ADR, MC_ERR_ADR_HI),
	MC_ERR(MC_INT_DECERR_GENERALIZED_CARVEOUT | MC_INT_DECERR_EMEM,
	       "EMEM GSC access violation", 0,
	       MC_ERR_GENERALIZED_CARVEOUT_STATUS,
	       MC_ERR_GENERALIZED_CARVEOUT_ADR),

	/* NULL terminate. */
	MC_ERR(0, NULL, 0, 0, 0),
};

/* reported in MC_CH_INTSTATUS_0 */
static const struct mc_error ch_mc_errors[] = {
	MC_ERR(MC_INT_WCAM_ERR, "WCAM error", E_TWO_STATUS,
	       MC_WCAM_IRQ_P0_STATUS0,
	       MC_WCAM_IRQ_P1_STATUS0),

	/* NULL terminate. */
	MC_ERR(0, NULL, 0, 0, 0),
};

/* reported in MC_HUBC_INTSTATUS_0 */
static const struct mc_error hubc_mc_errors[] = {
	MC_ERR(MC_HUBC_INT_SCRUB_ECC_WR_ACK,
	       "ECC scrub complete", E_NO_STATUS, 0, 0),

	/* NULL terminate. */
	MC_ERR(0, NULL, 0, 0, 0),
};

/* reported in MC_HUB_INTSTATUS_0 */
static const struct mc_error hub_mc_errors[] = {
	MC_ERR_HI(MC_HUB_INT_HUB_COALESCER_ERR,
	       "Hub coalescer error", 0,
	       MC_COALESCE_ERR_STATUS, MC_COALESCE_ERR_ADR,
	       MC_COALESCE_ERR_ADR_HI),
	MC_ERR_HI(MC_HUB_INT_ILLEGAL_HUB_REQ,
	       "Illegal Hub Request", 0,
	       MC_COALESCE_ERR_STATUS, MC_COALESCE_ERR_ADR,
	       MC_COALESCE_ERR_ADR_HI),

	/* NULL terminate. */
	MC_ERR(0, NULL, 0, 0, 0),
};

/* reported in MC_SBS_INTSTATUS_0 */
static const struct mc_error sbs_mc_errors[] = {
	MC_ERR(MC_SBS_INT_FILL_FIFO_ISO_OF,
	       "SBS ISO fifo overflow", 0, E_NO_STATUS, 0),
	MC_ERR(MC_SBS_INT_FILL_FIFO_SISO_OF,
	       "SBS SISO fifo overflow", 0, E_NO_STATUS, 0),
	MC_ERR(MC_SBS_INT_FILL_FIFO_NISO_OF,
	       "SBS NISO fifo overflow", 0, E_NO_STATUS, 0),

	/* NULL terminate. */
	MC_ERR(0, NULL, 0, 0, 0),
};

static void set_intstatus(unsigned int irq)
{
}

static void save_intstatus(unsigned int irq)
{
	global_intstatus = mc_readl(MC_GLOBAL_INTSTATUS);
	global_intstatus_1 = mc_readl(MC_GLOBAL_INTSTATUS_1);

	/*
	 * If multiple interrupts come in just handle the first one we see. The
	 * HW only keeps track of 1 interrupt's data and we don't know which
	 * particular fault is actually being kept...
	 */

	if (global_intstatus & GIS_CH_MASK) {
		mc_channel = __ffs(global_intstatus & GIS_CH_MASK);
	} else if (global_intstatus & GIS_SLICE_MASK) {
		mc_channel = __ffs((global_intstatus & GIS_SLICE_MASK) >> GIS_SLICE0);
	} else if (global_intstatus & GIS_HUB_MASK) {
		mc_channel = __ffs((global_intstatus & GIS_HUB_MASK) >> GIS_HUB0);
	} else if (global_intstatus & GIS_NVLINK_MASK) {
		mc_channel = __ffs((global_intstatus & GIS_NVLINK_MASK) >> GIS_nvlink0);
	} else if (global_intstatus & BIT(GIS_HUBC)) {
		mc_channel = MC_BROADCAST_CHANNEL;
	} else if (global_intstatus & BIT(GIS_SBS)) {
		mc_channel = MC_BROADCAST_CHANNEL;
	} else if (global_intstatus_1 & GIS_1_CH_MASK) {
		mc_channel = 8 + __ffs(global_intstatus_1 & GIS_1_CH_MASK);
	} else {
		mcerr_pr("mcerr: unknown intr source intstatus = 0x%08x, "
			 "intstatus_1 = 0x%08x\n", global_intstatus, global_intstatus_1);
	}
	slice_int_status = __mc_readl(mc_channel, MC_INTSTATUS);
	ch_int_status = __mc_readl(mc_channel, MC_CH_INTSTATUS);
	hubc_int_status = __mc_readl(mc_channel, MC_HUBC_INTSTATUS);
	sbs_int_status = __mc_readl(mc_channel, MC_MSS_SBS_INTSTATUS);
	hub_int_status = __mc_readl(mc_channel, MC_HUB_INTSTATUS);
}

static void clear_intstatus(unsigned int irq)
{
	/* Clear int status to clear MSS to GIC interrupts */
	mc_writel(INTSTATUS_CLEAR, MC_INTSTATUS);
	mc_writel(HUBC_INTSTATUS_CLEAR, MC_HUBC_INTSTATUS);
	mc_writel(HUB_INTSTATUS_CLEAR, MC_HUB_INTSTATUS);
	mc_writel(CH_INTSTATUS_CLEAR, MC_CH_INTSTATUS);
	mc_writel(SBS_INTSTATUS_CLEAR, MC_MSS_SBS_INTSTATUS);
	mc_writel(GLOBAL_INTSTATUS_CLEAR, MC_GLOBAL_INTSTATUS);
	mc_writel(GLOBAL_INTSTATUS_1_CLEAR, MC_GLOBAL_INTSTATUS_1);
}

static void log_fault(int src_chan, const struct mc_error *fault)
{
	phys_addr_t addr;
	struct mc_client *client;
	u32 status, write, secure, client_id;
	u32 gsc_status_1, high_addr_reg = 0;


	if (fault->flags & E_VPR)
		mcerr_pr("vpr base=%x:%x, size=%x, ctrl=%x, override:(%x, %x, %x, %x)\n",
			 mc_readl(MC_VIDEO_PROTECT_BOM_ADR_HI),
			 mc_readl(MC_VIDEO_PROTECT_BOM),
			 mc_readl(MC_VIDEO_PROTECT_SIZE_MB),
			 mc_readl(MC_VIDEO_PROTECT_REG_CTRL),
			 mc_readl(MC_VIDEO_PROTECT_VPR_OVERRIDE),
			 mc_readl(MC_VIDEO_PROTECT_VPR_OVERRIDE1),
			 mc_readl(MC_VIDEO_PROTECT_GPU_OVERRIDE_0),
			 mc_readl(MC_VIDEO_PROTECT_GPU_OVERRIDE_1));

	if (fault->flags & E_NO_STATUS) {
		mcerr_pr("MC fault - no status: %s\n", fault->msg);
		return;
	}

	status = __mc_readl(src_chan, fault->stat_reg);
	addr = __mc_readl(src_chan, fault->addr_reg);

	if (fault->flags & E_TWO_STATUS) {
		mcerr_pr("MC fault - %s\n", fault->msg);
		mcerr_pr("status: 0x%08x status2: 0x%08llx\n",
			status, addr);
		return;
	}

	secure = !!(status & MC_ERR_STATUS_SECURE);
	write = !!(status & MC_ERR_STATUS_WRITE);
	client_id = status & 0xff;
	client = &mc_clients[client_id <= mc_client_last
			     ? client_id : mc_client_last];

	if (fault->flags & E_GSC) {
		high_addr_reg = __mc_readl(src_chan, fault->addr_hi_reg);
		addr |= ((phys_addr_t)(high_addr_reg >> 16) << 32);
	} else if (fault->flags & E_ADR_HI_REG) {
		high_addr_reg = __mc_readl(src_chan, fault->addr_hi_reg);
		addr |= ((phys_addr_t)high_addr_reg << 32);
	} else {
		addr |= (((phys_addr_t)(status & MC_ERR_STATUS_ADR_HI_BITS)) << 12);
	}

	mcerr_pr("(%d) %s: %s\n", client->swgid, client->name, fault->msg);
	mcerr_pr("  status = 0x%08x; addr = 0x%16llx; hi_adr_reg=0x%x\n",
		status, (long long unsigned int)addr, high_addr_reg);
	mcerr_pr("  secure: %s, access-type: %s\n",
		secure ? "yes" : "no", write ? "write" : "read");
	if (fault->flags & E_GSC) {
		gsc_status_1 = __mc_readl(src_chan, fault->addr_hi_reg);
		mcerr_pr("gsc_id=%d, gsc_co_id=%d\n",
			((status >> 8) & 0x7) | ((gsc_status_1 & 3) << 3),
			((status >> 24) & 0x7) | (((gsc_status_1 >> 7) & 0x3) << 3));
	}
}

#define LOG_FAULT(n, m, r) \
	for (err = n##_mc_errors; \
	     n##_int_status && err->sig && err->msg; err++) { \
		if ((n##_int_status & m) != err->sig) \
			continue; \
		log_fault(mc_channel, err); \
		__mc_writel(mc_channel, n##_int_status, MC##r##INTSTATUS); \
		faults_handled++; \
		break; \
	} \

static void log_mcerr_fault(unsigned int irq)
{
	int faults_handled = 0;
	const struct mc_error *err;
	u32 g_intstatus = global_intstatus;
	u32 g_intstatus_1 = global_intstatus_1;

	LOG_FAULT(slice, mc_int_mask, _);
	LOG_FAULT(hub, U32_MAX, _HUB_);
	LOG_FAULT(ch, U32_MAX, _CH_);
	LOG_FAULT(hubc, U32_MAX, _HUBC_);
	LOG_FAULT(sbs, U32_MAX, _MSS_SBS_);

	if (faults_handled) {
		mc_writel(g_intstatus, MC_GLOBAL_INTSTATUS);
		mc_writel(g_intstatus_1, MC_GLOBAL_INTSTATUS_1);
	} else {
		pr_err("unknown mcerr fault, int_status=0x%08x, "
			"ch_int_status=0x%08x, hubc_int_status=0x%08x "
			"sbs_int_status=0x%08x, hub_int_status=0x%08x\n",
			slice_int_status, ch_int_status, hubc_int_status,
			sbs_int_status, hub_int_status);
	}
}

static struct mcerr_ops mcerr_ops = {
	.nr_clients = ARRAY_SIZE(mc_clients),
	.intr_descriptions = intr_info,
	.set_intstatus = set_intstatus,
	.clear_intstatus = clear_intstatus,
	.save_intstatus = save_intstatus,
	.log_mcerr_fault = log_mcerr_fault,
	.mc_clients = mc_clients,
};

static struct mcerr_ops *t18x_mcerr_of_setup(struct device_node *np)
{
	pr_info("mcerr ops are set to t19x\n");
	return &mcerr_ops;
}

MCERR_OF_DECLARE(mcerr_of, "nvidia,tegra-t19x-mc", t18x_mcerr_of_setup);
