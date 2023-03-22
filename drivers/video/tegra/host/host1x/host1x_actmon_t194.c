/*
 * Tegra Graphics Host Actmon support for T194
 *
 * Copyright (c) 2015-2023, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/nvhost.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#include "dev.h"
#include "chip_support.h"
#include "host1x/host1x_actmon.h"
#include "bus_client_t194.h"
#include "nvhost_scale.h"

static void host1x_actmon_process_isr(u32 hintstat, void *priv);

static void actmon_writel(struct host1x_actmon *actmon, u32 val, u32 reg)
{
	nvhost_dbg(dbg_reg, " r=0x%x v=0x%x", reg, val);
	writel(val, actmon->regs + reg);
}

static u32 actmon_readl(struct host1x_actmon *actmon, u32 reg)
{
	u32 val = readl(actmon->regs + reg);
	nvhost_dbg(dbg_reg, " r=0x%x v=0x%x", reg, val);
	return val;
}

static void host1x_actmon_dump_regs(struct host1x_actmon *actmon)
{
	nvhost_dbg(dbg_reg, "global ctrl: 0x%x\n",
		actmon_readl(actmon, actmon_glb_ctrl_r()));

	nvhost_dbg(dbg_reg, "global intr enable: 0x%x\n",
		actmon_readl(actmon, actmon_glb_intr_en_r()));

	nvhost_dbg(dbg_reg, "global intr status: 0x%x\n",
		actmon_readl(actmon, actmon_glb_intr_status_r()));

	nvhost_dbg(dbg_reg, "local ctrl: 0x%x\n",
		actmon_readl(actmon, actmon_local_ctrl_r()));

	nvhost_dbg(dbg_reg, "local intr enable: 0x%x\n",
		actmon_readl(actmon, actmon_local_intr_en_r()));

	nvhost_dbg(dbg_reg, "local intr status: 0x%x\n",
		actmon_readl(actmon, actmon_local_intr_status_r()));

	nvhost_dbg(dbg_reg, "local avg upper watermark: 0x%x\n",
		actmon_readl(actmon, actmon_local_avg_upper_wmark_r()));

	nvhost_dbg(dbg_reg, "local avg lower watermark: 0x%x\n",
		actmon_readl(actmon, actmon_local_avg_lower_wmark_r()));

	nvhost_dbg(dbg_reg, "local init avg: 0x%x\n",
		actmon_readl(actmon, actmon_local_init_avg_r()));

	nvhost_dbg(dbg_reg, "local count : 0x%x\n",
		actmon_readl(actmon, actmon_local_count_r()));

	nvhost_dbg(dbg_reg, "local avg count : 0x%x\n",
		actmon_readl(actmon, actmon_local_avg_count_r()));

	nvhost_dbg(dbg_reg, "local cumulative count : 0x%x\n",
		actmon_readl(actmon, actmon_local_cumulative_r()));
}

static void host1x_actmon_event_fn(struct host1x_actmon *actmon, int type)
{
	struct platform_device *pdev = actmon->pdev;
	struct nvhost_device_data *engine_pdata = platform_get_drvdata(pdev);

	/* ensure that the device remains powered */
	nvhost_module_busy_noresume(pdev);
	if (pm_runtime_active(&pdev->dev)) {
		/* first, handle scaling */
		nvhost_scale_actmon_irq(pdev, type);

		/* then, rewire the actmon IRQ */
		nvhost_intr_enable_host_irq(&nvhost_get_host(pdev)->intr,
					    engine_pdata->actmon_irq,
					    host1x_actmon_process_isr,
					    actmon);
	}
	host1x_actmon_dump_regs(actmon);
	nvhost_module_idle(pdev);
}

static void host1x_actmon_process_isr(u32 hintstat, void *priv)
{
	struct host1x_actmon *actmon = priv;
	struct platform_device *host_pdev = actmon->host->dev;
	struct nvhost_device_data *engine_pdata =
		platform_get_drvdata(actmon->pdev);
	long val;

	/* Disable the actmon interrupt related to the monitored engine */
	nvhost_intr_disable_host_irq(&nvhost_get_host(host_pdev)->intr,
				     engine_pdata->actmon_irq);

	/* Clear global interrupt status register */
	val = actmon_readl(actmon, actmon_glb_intr_status_r());
	actmon_writel(actmon, val, actmon_glb_intr_status_r());

	/* Clear local interrupt status register */
	val = actmon_readl(actmon, actmon_local_intr_status_r());
	actmon_writel(actmon, val, actmon_local_intr_status_r());

	/* Determine watermark event type for the interrupt */
	if (actmon_local_intr_status_avg_above_wmark_v(val))
		host1x_actmon_event_fn(actmon, ACTMON_INTR_ABOVE_WMARK);
	else if (actmon_local_intr_status_avg_below_wmark_v(val))
		host1x_actmon_event_fn(actmon, ACTMON_INTR_BELOW_WMARK);
	else if (actmon_local_intr_status_consec_above_wmark_v(val))
		host1x_actmon_event_fn(actmon, ACTMON_INTR_CONSEC_ABOVE_WMARK);
	else if (actmon_local_intr_status_consec_below_wmark_v(val))
		host1x_actmon_event_fn(actmon, ACTMON_INTR_CONSEC_BELOW_WMARK);
}

/*
 * actmon_update_sample_period_safe(host)
 *
 * This function updates frequency specific values on actmon using the current
 * actmon frequency. The function should be called only when host1x is active.
 *
 */

static void actmon_update_sample_period_safe(struct host1x_actmon *actmon)
{
	long freq_mhz, clks_per_sample;
	u32 val = actmon_readl(actmon, actmon_glb_ctrl_r());

	/*
	 * We use (MHz and usec) instead of (Hz and sec)
	 * due to numerical limitations
	 */
	freq_mhz = clk_get_rate(actmon->clk) / 1000000;

	/* Set SOURCE as TICK */
	val |= actmon_glb_ctrl_source_f(2);

	/* Determine the SAMPLE_TICK and divider based on the usec_per_sample */
	if ((freq_mhz * actmon->usecs_per_sample) / 256 > 255) {
		val |= actmon_glb_ctrl_sample_tick_f(1);
		actmon->divider = 65536;
	} else {
		val &= ~actmon_glb_ctrl_sample_tick_f(1);
		actmon->divider = 256;
	}

	/* Number of clock cycles for each sample */
	clks_per_sample = (freq_mhz * actmon->usecs_per_sample) /
		actmon->divider;
	actmon->clks_per_sample = clks_per_sample + 1;

	/* Update the SAMPLE_PERIOD in the control register */
	val &= ~actmon_glb_ctrl_sample_period_m();
	val |= actmon_glb_ctrl_sample_period_f(clks_per_sample);
	actmon_writel(actmon, val, actmon_glb_ctrl_r());

	/* AVG value depends on sample period => clear it */
	actmon_writel(actmon, 0, actmon_local_init_avg_r());

	host1x_actmon_dump_regs(actmon);
}

static void __iomem *host1x_actmon_get_regs(struct host1x_actmon *actmon)
{
	struct nvhost_device_data *pdata =
		platform_get_drvdata(actmon->pdev);
	void __iomem *actmon_base = NULL;

	actmon_base = get_aperture(actmon->host->dev, HOST1X_ACTMON_APERTURE);
	if (actmon_base)
		return actmon_base + pdata->actmon_regs;

	return NULL;
}

static unsigned long get_module_freq(struct host1x_actmon *actmon)
{
	unsigned long freq = 0;

	struct platform_device *pdev = actmon->pdev;
	struct nvhost_device_data *engine_pdata = platform_get_drvdata(pdev);
	struct devfreq *df = engine_pdata->power_manager;
	struct nvhost_device_data *pdata = dev_get_drvdata(df->dev.parent);
	struct nvhost_device_profile *profile = pdata->power_profile;

	freq = clk_get_rate(profile->clk);

	return freq;
}

static void host1x_actmon_reset(struct host1x_actmon *actmon)
{
	/*
	 * Reg: HOST1X_THOST_ACTMON_XXX_GLB_CTRL_0
	 * Reset actmon sampling configuration.
	 */
	actmon_writel(actmon, 0, actmon_glb_ctrl_r());

	/*
	 * Reg: HOST1X_THOST_ACTMON_XXX_GLB_INT_EN_0
	 * Disable interrupt for all unit actmon instances [0:7].
	 * It acts as a filter for local interrupt signals.
	 */
	actmon_writel(actmon, 0, actmon_glb_intr_en_r());

	/*
	 * Reg: HOST1X_THOST_ACTMON_XXX_CTRL_0
	 * Disable actmon for monitoring the engine activity
	 */
	actmon_writel(actmon, 0, actmon_local_ctrl_r());

	/*
	 * Reg: HOST1X_THOST_ACTMON_XXX_INTR_ENABLE_0
	 * Disable local actmon interrupts. (e.g. consecutive and average)
	 */
	actmon_writel(actmon, 0, actmon_local_intr_en_r());

	/*
	 * Reg: HOST1X_THOST_ACTMON_XXX_INIT_AVG_0
	 * Clear initial value of moving average acitvity counter
	 */
	actmon_writel(actmon, 0, actmon_local_init_avg_r());

	/*
	 * Reg: HOST1X_THOST_ACTMON_XXX_AVG_UPPER_WMARK_0
	 * Reg: HOST1X_THOST_ACTMON_XXX_AVG_LOWER_WMARK_0
	 * Clear average watermark thresholds
	 */
	actmon_writel(actmon, 0, actmon_local_avg_lower_wmark_r());
	actmon_writel(actmon, 0, actmon_local_avg_upper_wmark_r());

	/*
	 * Reg: HOST1X_THOST_ACTMON_XXX_UPPER_WMARK_0
	 * Reg: HOST1X_THOST_ACTMON_XXX_LOWER_WMARK_0
	 * Clear consecutive watermark thresholds
	 */
	actmon_writel(actmon, 0, actmon_local_lower_wmark_r());
	actmon_writel(actmon, 0, actmon_local_upper_wmark_r());

	/*
	 * Reg: HOST1X_THOST_ACTMON_XXX_INTR_STATUS_0
	 * Clear local interrupt status register (e.g. consecutive and average)
	 */
	actmon_writel(actmon, 0xffffffff, actmon_local_intr_status_r());
}

static int host1x_actmon_init(struct host1x_actmon *actmon)
{
	struct platform_device *host_pdev = actmon->host->dev;
	struct nvhost_device_data *host_pdata =
		platform_get_drvdata(host_pdev);
	struct nvhost_device_data *engine_pdata =
		platform_get_drvdata(actmon->pdev);
	u32 val;

	/* Avoid to perform actmon initialization repeatedly */
	if (actmon->init == ACTMON_READY)
		return 0;

	/* Actmon default settings */
	if (actmon->init == ACTMON_OFF) {
		actmon->consec_upper_num = 7;
		actmon->consec_lower_num = 7;
		actmon->usecs_per_sample = 1500;
		actmon->k = 4;
	}

	/* Check the existence of actmon reference clock */
	actmon->clk = host_pdata->clk[1];
	if (!actmon->clk)
		return -ENODEV;

	/* Reset actmon-related registers to default settings */
	host1x_actmon_reset(actmon);

	/* Write (normalised) sample period. */
	actmon_update_sample_period_safe(actmon);

	/* Configure local actmon control register */
	val = actmon_readl(actmon, actmon_local_ctrl_r());
	val |= actmon_local_ctrl_actmon_enable_f(1);
	val |= actmon_local_ctrl_enb_periodic_f(1);
	val |= actmon_local_ctrl_enb_cumulative_f(1);
	val |= actmon_local_ctrl_k_val_f(actmon->k);
	val |= actmon_local_ctrl_consec_upper_num_f(actmon->consec_upper_num);
	val |= actmon_local_ctrl_consec_lower_num_f(actmon->consec_lower_num);
	actmon_writel(actmon, val, actmon_local_ctrl_r());

	/* Configure COUNT_WEIGHT based on associated engine max freq */
	actmon_writel(actmon, engine_pdata->actmon_weight_count,
					actmon_local_count_weight_r());

	/* Enable actmon global interrupt to */
	if (engine_pdata->actmon_irq)
		actmon_writel(actmon, 0x1, actmon_glb_intr_en_r());

	/* Let host1x recognize the interrupt from this actmon instance */
	nvhost_intr_enable_host_irq(&nvhost_get_host(host_pdev)->intr,
				    engine_pdata->actmon_irq,
				    host1x_actmon_process_isr,
				    actmon);

	/* Actmon is ready to serve */
	actmon->init = ACTMON_READY;

	host1x_actmon_dump_regs(actmon);

	return 0;
}

static void host1x_actmon_deinit(struct host1x_actmon *actmon)
{
	struct platform_device *host_pdev = actmon->host->dev;
	struct nvhost_device_data *engine_pdata =
		platform_get_drvdata(actmon->pdev);

	if (actmon->init != ACTMON_READY)
		return;

	/* Disable interrupts */
	if (engine_pdata->actmon_irq) {
		actmon_writel(actmon, 0x0, actmon_glb_intr_en_r());
		actmon_writel(actmon, 0x0, actmon_local_intr_en_r());
	}

	/* clear intrrupt status registers */
	actmon_writel(actmon, 0xffffffff, actmon_glb_intr_status_r());
	actmon_writel(actmon, 0xffffffff, actmon_local_intr_status_r());

	/* Disable actmon */
	actmon_writel(actmon, 0x0, actmon_local_ctrl_r());
	actmon_writel(actmon, 0x0, actmon_glb_ctrl_r());

	actmon->init = ACTMON_SLEEP;

	nvhost_intr_disable_host_irq(&nvhost_get_host(host_pdev)->intr,
				     engine_pdata->actmon_irq);

	host1x_actmon_dump_regs(actmon);
}

static int host1x_actmon_avg(struct host1x_actmon *actmon, u32 *val)
{
	int err;

	if (actmon->init != ACTMON_READY) {
		*val = 0;
		return 0;
	}

	err = nvhost_module_busy(actmon->host->dev);
	if (err)
		return err;

	rmb();
	*val = actmon_readl(actmon, actmon_local_avg_count_r());
	nvhost_module_idle(actmon->host->dev);

	return 0;
}

static int host1x_actmon_avg_norm(struct host1x_actmon *actmon, u32 *avg)
{
	long val;
	int err;
	unsigned long freq = 0;
	u32 dev_cycle_per_sample = 0;

	if (!(actmon->init == ACTMON_READY && actmon->clks_per_sample > 0 &&
	    actmon->divider)) {
		*avg = 0;
		return 0;
	}

	/* Read load from hardware */
	err = nvhost_module_busy(actmon->host->dev);
	if (err)
		return err;

	rmb();
	val = actmon_readl(actmon, actmon_local_avg_count_r());
	nvhost_module_idle(actmon->host->dev);

	freq = get_module_freq(actmon);
	dev_cycle_per_sample = ((freq / 1000) * actmon->usecs_per_sample)
								/ 1000;

	*avg = (val * 1000) / dev_cycle_per_sample;

	return 0;
}

static int host1x_actmon_count(struct host1x_actmon *actmon, u32 *val)
{
	int err;

	if (actmon->init != ACTMON_READY) {
		*val = 0;
		return 0;
	}

	/* Read load from hardware */
	err = nvhost_module_busy(actmon->host->dev);
	if (err)
		return err;

	rmb();
	*val = actmon_readl(actmon, actmon_local_count_r());
	nvhost_module_idle(actmon->host->dev);

	return 0;
}

static int host1x_actmon_count_norm(struct host1x_actmon *actmon, u32 *avg)
{
	long val;
	int err;

	if (!(actmon->init == ACTMON_READY && actmon->clks_per_sample > 0 &&
	    actmon->divider)) {
		*avg = 0;
		return 0;
	}

	/* Read load from hardware */
	err = nvhost_module_busy(actmon->host->dev);
	if (err)
		return err;

	rmb();
	val = actmon_readl(actmon, actmon_local_count_r());
	nvhost_module_idle(actmon->host->dev);

	*avg = (val * 1000) / (actmon->clks_per_sample * actmon->divider);

	return 0;
}

static int host1x_set_high_wmark(struct host1x_actmon *actmon, u32 scale)
{
	struct platform_device *pdev = actmon->host->dev;
	u32 freq_khz, dev_cycle_per_sample, val, avg_wmark, consec_wmark;
	int err;

	/* Check whether actmon has been initialized or not */
	if (actmon->init != ACTMON_READY)
		return 0;

	/* Power on the host1x first before writing to the actmon register */
	err = nvhost_module_busy(actmon->host->dev);
	if (err) {
		nvhost_warn(&pdev->dev, "failed to power on host1x. update high watermark failed");
		return err;
	}

	/* Determine number of clock cycles for each sampling in actmon */
	freq_khz = get_module_freq(actmon) / 1000;
	dev_cycle_per_sample = (actmon->usecs_per_sample * freq_khz) / 1000;

	/* Determine average watermark threshold */
	avg_wmark = (scale < 1000) ?
		((scale * dev_cycle_per_sample) / 1000) :
		dev_cycle_per_sample;

	/* Determine consecutive watermark threshold */
	consec_wmark = (scale < 900) ?
		(((scale + 100) * avg_wmark) / scale) :
		avg_wmark;

	/* Program new watermark thresholds to the actmon register */
	actmon_writel(actmon, avg_wmark, actmon_local_avg_upper_wmark_r());
	actmon_writel(actmon, consec_wmark, actmon_local_upper_wmark_r());

	/* Enable or disable watermark interrupt depending on the scale value */
	val = actmon_readl(actmon, actmon_local_intr_en_r());
	if (scale < 1000) {
		val |= actmon_local_intr_en_avg_above_wmark_en_f(1);
		val |= actmon_local_intr_en_consecutive_above_wmark_en_f(1);
	} else {
		val &= ~actmon_local_intr_en_avg_above_wmark_en_f(1);
		val &= ~actmon_local_intr_en_consecutive_above_wmark_en_f(1);
	}
	actmon_writel(actmon, val, actmon_local_intr_en_r());

	host1x_actmon_dump_regs(actmon);

	return 0;
}

static int host1x_set_low_wmark(struct host1x_actmon *actmon, u32 scale)
{
	struct platform_device *pdev = actmon->host->dev;
	u32 freq_khz, dev_cycle_per_sample, val, avg_wmark, consec_wmark;
	int err;

	/* Check whether actmon has been initialized or not */
	if (actmon->init != ACTMON_READY)
		return 0;

	/* Power on the host1x first before writing to the actmon register */
	err = nvhost_module_busy(actmon->host->dev);
	if (err) {
		nvhost_warn(&pdev->dev, "failed to power on host1x. update low watermark failed");
		return err;
	}

	/* Determine number of clock cycles for each sampling in actmon */
	freq_khz = get_module_freq(actmon) / 1000;
	dev_cycle_per_sample = (actmon->usecs_per_sample * freq_khz) / 1000;

	/* Determine average watermark threshold */
	avg_wmark = (scale < 1000) ?
		((scale * dev_cycle_per_sample) / 1000) :
		dev_cycle_per_sample;

	/* Determine consecutive watermark threshold */
	consec_wmark = (scale > 100) ?
		(((scale - 100) * avg_wmark) / scale) :
		avg_wmark;

	/* Program new watermark thresholds to the actmon register */
	actmon_writel(actmon, avg_wmark, actmon_local_avg_lower_wmark_r());
	actmon_writel(actmon, consec_wmark, actmon_local_lower_wmark_r());

	/* Enable or disable watermark interrupt depending on the scale value */
	val = actmon_readl(actmon, actmon_local_intr_en_r());
	if (scale) {
		val |= actmon_local_intr_en_avg_below_wmark_en_f(1);
		val |= actmon_local_intr_en_consecutive_below_wmark_en_f(1);
	} else {
		val &= ~actmon_local_intr_en_avg_below_wmark_en_f(1);
		val &= ~actmon_local_intr_en_consecutive_below_wmark_en_f(1);
	}
	actmon_writel(actmon, val, actmon_local_intr_en_r());

	host1x_actmon_dump_regs(actmon);

	return 0;
}

static void host1x_actmon_update_sample_period(struct host1x_actmon *actmon)
{
	struct platform_device *pdev = actmon->host->dev;
	int err;

	/* No sense to update actmon if actmon is inactive */
	if (actmon->init != ACTMON_READY)
		return;

	err = nvhost_module_busy(actmon->host->dev);
	if (err) {
		nvhost_warn(&pdev->dev, "failed to power on host1x. sample period update failed");
		return;
	}

	actmon_update_sample_period_safe(actmon);
	nvhost_module_idle(actmon->host->dev);
}

static void host1x_actmon_set_sample_period_norm(struct host1x_actmon *actmon,
						 long usecs)
{
	actmon->usecs_per_sample = usecs;
	host1x_actmon_update_sample_period(actmon);
}

static void host1x_actmon_set_k(struct host1x_actmon *actmon, u32 k)
{
	struct platform_device *pdev = actmon->host->dev;
	long val;
	int err;

	actmon->k = k;

	err = nvhost_module_busy(actmon->host->dev);
	if (err) {
		nvhost_warn(&pdev->dev, "failed to power on host1x. actmon k value update failed");
		return;
	}

	val = actmon_readl(actmon, actmon_local_ctrl_r());
	val &= ~(actmon_local_ctrl_k_val_m());
	val |= actmon_local_ctrl_k_val_f(actmon->k);
	actmon_writel(actmon, val, actmon_local_ctrl_r());
	nvhost_module_idle(actmon->host->dev);
}

static u32 host1x_actmon_get_k(struct host1x_actmon *actmon)
{
	return actmon->k;
}

static void host1x_actmon_set_consec_upper_num(struct host1x_actmon *actmon, u32 num)
{
	struct platform_device *pdev = actmon->host->dev;
	long val;
	int err;

	/* The consecutive number (N) should within 1 to 8 */
	if (num < 1 || num > 8)
		return;

	/* Power on the host1x first before writing to the actmon register */
	err = nvhost_module_busy(actmon->host->dev);
	if (err) {
		nvhost_warn(&pdev->dev, "failed to power on host1x. consec_upper_num update failed");
		return;
	}

	/* Actmon will consider the number as N + 1 */
	actmon->consec_upper_num = num-1;

	/* Write the value to the register */
	val = actmon_readl(actmon, actmon_local_ctrl_r());
	val &= ~(actmon_local_ctrl_consec_upper_num_m());
	val |= actmon_local_ctrl_consec_upper_num_f(actmon->consec_upper_num);
	actmon_writel(actmon, val, actmon_local_ctrl_r());

	nvhost_module_idle(actmon->host->dev);
}

static u32 host1x_actmon_get_consec_upper_num(struct host1x_actmon *actmon)
{
	return actmon->consec_upper_num+1;
}

static void host1x_actmon_set_consec_lower_num(struct host1x_actmon *actmon, u32 num)
{
	struct platform_device *pdev = actmon->host->dev;
	long val;
	int err;

	/* The consecutive number (N) should within 1 to 8 */
	if (num < 1 || num > 8)
		return;

	/* Power on the actmon first before writing to the register */
	err = nvhost_module_busy(actmon->host->dev);
	if (err) {
		nvhost_warn(&pdev->dev, "failed to power on host1x. consec_lower_num update failed");
		return;
	}

	/* Actmon will consider the number as N + 1 */
	actmon->consec_lower_num = num-1;

	/* Write the value to the register */
	val = actmon_readl(actmon, actmon_local_ctrl_r());
	val &= ~(actmon_local_ctrl_consec_lower_num_m());
	val |= actmon_local_ctrl_consec_lower_num_f(actmon->consec_lower_num);
	actmon_writel(actmon, val, actmon_local_ctrl_r());

	nvhost_module_idle(actmon->host->dev);
}

static u32 host1x_actmon_get_consec_lower_num(struct host1x_actmon *actmon)
{
	return actmon->consec_lower_num+1;
}

static long host1x_actmon_get_sample_period(struct host1x_actmon *actmon)
{
	return actmon->clks_per_sample;
}

static long host1x_actmon_get_sample_period_norm(struct host1x_actmon *actmon)
{
	return actmon->usecs_per_sample;
}

static int host1x_actmon_cumulative(struct host1x_actmon *actmon, u32 *val)
{
	int err;

	if (actmon->init != ACTMON_READY) {
		*val = 0;
		return 0;
	}

	/* Read load from hardware */
	err = nvhost_module_busy(actmon->host->dev);
	if (err)
		return err;

	rmb();
	*val = actmon_readl(actmon, actmon_local_cumulative_r());
	nvhost_module_idle(actmon->host->dev);

	return 0;
}

static int actmon_cumulative_show(struct seq_file *s, void *unused)
{
	struct host1x_actmon *actmon = s->private;
	u32 avg;
	int err;

	err = host1x_actmon_cumulative(actmon, &avg);
	if (!err)
		seq_printf(s, "%d\n", avg);
	return err;
}

static int actmon_cumulative_open(struct inode *inode, struct file *file)
{
	return single_open(file, actmon_cumulative_show, inode->i_private);
}

static const struct file_operations actmon_cumulative_fops = {
	.open		= actmon_cumulative_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void t18x_actmon_debug_init(struct host1x_actmon *actmon,
				     struct dentry *de)
{
	if (!actmon)
		return;

	debugfs_create_file("actmon_cumulative", S_IRUGO, de,
			actmon, &actmon_cumulative_fops);
}

static const struct nvhost_actmon_ops host1x_actmon_ops = {
	.get_actmon_regs = host1x_actmon_get_regs,
	.init = host1x_actmon_init,
	.deinit = host1x_actmon_deinit,
	.read_avg = host1x_actmon_avg,
	.read_avg_norm = host1x_actmon_avg_norm,
	.read_count = host1x_actmon_count,
	.read_count_norm = host1x_actmon_count_norm,
	.update_sample_period = host1x_actmon_update_sample_period,
	.set_sample_period_norm = host1x_actmon_set_sample_period_norm,
	.get_sample_period_norm = host1x_actmon_get_sample_period_norm,
	.get_sample_period = host1x_actmon_get_sample_period,
	.get_k = host1x_actmon_get_k,
	.set_k = host1x_actmon_set_k,
	.get_consec_upper_num = host1x_actmon_get_consec_upper_num,
	.set_consec_upper_num = host1x_actmon_set_consec_upper_num,
	.get_consec_lower_num = host1x_actmon_get_consec_lower_num,
	.set_consec_lower_num = host1x_actmon_set_consec_lower_num,
	.debug_init = t18x_actmon_debug_init,
	.set_high_wmark = host1x_set_high_wmark,
	.set_low_wmark = host1x_set_low_wmark,
};

/* Sample period need to be same as the one programmed in NvHost Server
 * TODO - Move the attribute readable from DT if supported post
 *        server migration to HVRTOS
 */
#define ACTMON_SAMPLE_PERIOD_US 100U
static void host1x_actmon_obs_init(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	unsigned long freq = 0;
	struct clk *c;

	/*
	 * Fetch the module rate in Hz, convert to MHz and
	 * calculate device cycles per sample
	 */
	c = devm_clk_get(&pdev->dev, pdata->clocks[0].name);
	freq = clk_get_rate(c);

	pdata->cycles_per_actmon_sample = ((freq / 1000000) *
					   ACTMON_SAMPLE_PERIOD_US);
}

static void host1x_actmon_obs_avg_norm(struct platform_device *pdev, u32 *avg)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_master *host = nvhost_get_host(pdev);
	long val;

	if (!pdata || !host || !host->actmon_aperture ||
			(pdata->cycles_per_actmon_sample == 0)) {
		*avg = 0;
		return;
	}

	/* Read the average usage count for the engine */
	val = readl(host->actmon_aperture + pdata->actmon_regs +
		    actmon_local_avg_count_r());

	/* Should not happen, adding just in case */
	if (val > pdata->cycles_per_actmon_sample)
		val = pdata->cycles_per_actmon_sample;

	/* Using 10 times percentage value to be consistent with tegrastats */
	*avg = (val * 1000) / pdata->cycles_per_actmon_sample;
}

/* Operations on actmon just for reading the usage, programming taken care by Server */
static const struct nvhost_actmon_obs_ops host1x_actmon_obs_ops = {
	.init = host1x_actmon_obs_init,
	.read_avg_norm = host1x_actmon_obs_avg_norm,
};
