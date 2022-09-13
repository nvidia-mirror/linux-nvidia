/*
 * Copyright (c) 2020-2022, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <linux/interconnect-provider.h>
#include <linux/platform/tegra/mc_utils.h>
#include <linux/version.h>
#include <dt-bindings/interconnect/tegra_icc_id.h>
#include "tegra_icc.h"
#include <soc/tegra/fuse.h>

static int bwmgr_int_mrq_request(struct tegra_icc_provider *tp,
	uint32_t client_id,
	struct mrq_bwmgr_int_request *bwmgr_req,
	struct mrq_bwmgr_int_response *bwmgr_resp)
{
	int ret = 0;

	memset(&tp->msg, 0, sizeof(struct tegra_bpmp_message));
	tp->msg.mrq = MRQ_BWMGR_INT;
	tp->msg.tx.data = bwmgr_req;
	tp->msg.tx.size = sizeof(struct mrq_bwmgr_int_request);
	tp->msg.rx.data = bwmgr_resp;
	tp->msg.rx.size = sizeof(struct mrq_bwmgr_int_response);

	ret = tegra_bpmp_transfer(tp->bpmp_dev, &tp->msg);
	if (ret == 0 && tp->msg.rx.ret < 0) {
		pr_err("bwmgr_int req %u for client %u failed with ret %d\n",
			bwmgr_req->cmd,
			client_id,
			tp->msg.rx.ret);
		ret = -EINVAL;
	}

	tp->msg.tx.data = NULL;
	tp->msg.rx.data = NULL;

	return ret;
}

static int tegra23x_cap_set(struct tegra_icc_provider *tp,
	unsigned long cap_kbps)
{
	int ret = 0;
	int clk_fwk_ret, bpmp_ret;
	unsigned long cap_req;
	long round_rate_ret;
	struct mrq_bwmgr_int_request bwmgr_req = {0};
	struct mrq_bwmgr_int_response bwmgr_resp = {0};

	/* Determine what the rounded rate should be */
	if (cap_kbps) {
		clk_fwk_ret = clk_set_max_rate(tp->dram_clk, UINT_MAX);
		if (clk_fwk_ret) {
			pr_err("clk_set_max_rate failed %d\n", clk_fwk_ret);
			return clk_fwk_ret;
		}

		cap_req = cap_kbps;
		cap_req = emc_bw_to_freq(cap_req);
		cap_req = (UINT_MAX / 1000) <= cap_req ? UINT_MAX / 1000 : cap_req;
		round_rate_ret = clk_round_rate(tp->dram_clk, cap_req * 1000);
		if (round_rate_ret < 0) {
			pr_err("clk_round_rate fail %ld\n", round_rate_ret);
			cap_req = UINT_MAX;
		} else {
			cap_req = (unsigned long) round_rate_ret;
		}
	} else {
		cap_req = UINT_MAX;
	}

	/* Issue cap-set requests to clk-fwk and bpmp */
	clk_fwk_ret = clk_set_max_rate(tp->dram_clk, cap_req);
	if (clk_fwk_ret)
		pr_err("clk_set_max_rate fail %d\n", ret);

	bwmgr_req.cmd = CMD_BWMGR_INT_CAP_SET;
	bwmgr_req.bwmgr_cap_set_req.rate = cap_req;
	bpmp_ret = bwmgr_int_mrq_request(tp, TEGRA_ICC_NVPMODEL,
		&bwmgr_req, &bwmgr_resp);
	if (bpmp_ret)
		pr_err("BPMP cap_set MRQ fail %d\n", ret);

	/* If either failed, set ret flag and set both to UINT_MAX */
	if (bpmp_ret || clk_fwk_ret) {
		ret = -BPMP_EINVAL;
		clk_fwk_ret = clk_set_max_rate(tp->dram_clk, UINT_MAX);
		bwmgr_req.cmd = CMD_BWMGR_INT_CAP_SET;
		bwmgr_req.bwmgr_cap_set_req.rate = UINT_MAX;
		bpmp_ret = bwmgr_int_mrq_request(tp, TEGRA_ICC_NVPMODEL,
			&bwmgr_req, &bwmgr_resp);
	}

	return ret;
}

static int tegra23x_icc_set(struct icc_node *src, struct icc_node *dst)
{
	struct icc_provider *provider = src->provider;
	struct tegra_icc_provider *tp = to_tegra_provider(provider);
	struct tegra_icc_node *tn = src->data;
	int ret = 0;

	if (!tegra_platform_is_silicon() || !tp->mrqs_available)
		return 0;

	/* nvpmodel emc cap request */
	if (src->id == TEGRA_ICC_NVPMODEL) {
		ret = tegra23x_cap_set(tp, src->peak_bw);
	} else {
		struct mrq_bwmgr_int_request bwmgr_req = {0};
		struct mrq_bwmgr_int_response bwmgr_resp = {0};

		bwmgr_req.cmd = CMD_BWMGR_INT_CALC_AND_SET;
		bwmgr_req.bwmgr_calc_set_req.client_id = src->id;

		if (tn->type == TEGRA_ICC_NISO)
			bwmgr_req.bwmgr_calc_set_req.niso_bw = src->avg_bw;
		else
			bwmgr_req.bwmgr_calc_set_req.iso_bw = src->avg_bw;


		bwmgr_req.bwmgr_calc_set_req.mc_floor = src->peak_bw;
		bwmgr_req.bwmgr_calc_set_req.floor_unit = BWMGR_INT_UNIT_KBPS;

		ret = bwmgr_int_mrq_request(tp, src->id, &bwmgr_req, &bwmgr_resp);
	}

	return ret;
}

static int tegra23x_icc_get_init_bw(struct icc_node *node, u32 *avg, u32 *peak)
{
	/*
	 * These are required to maintain sane values before
	 * every client can request bw during boot. Not supported.
	 */
	*avg = 0;
	*peak = 0;

	return 0;
}

const struct tegra_icc_ops tegra23x_icc_ops = {
	.plat_icc_set = tegra23x_icc_set,
	.plat_icc_aggregate = icc_std_aggregate,
	.plat_icc_get_bw = tegra23x_icc_get_init_bw,
};
