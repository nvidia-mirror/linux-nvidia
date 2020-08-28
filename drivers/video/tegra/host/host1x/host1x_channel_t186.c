/*
 * drivers/video/tegra/host/host1x/channel_host1x.c
 *
 * Tegra Graphics Host Channel
 *
 * Copyright (c) 2010-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include "bus_client_t186.h"
#include "nvhost_channel.h"
#include "dev.h"
#include "class_ids.h"
#include "nvhost_acm.h"
#include "nvhost_job.h"
#include <trace/events/nvhost.h>
#include <linux/slab.h>
#include <linux/version.h>
#include "nvhost_sync.h"

#include "nvhost_intr.h"
#include "nvhost_vm.h"
#include "class_ids.h"
#include "debug.h"

#define MLOCK_TIMEOUT_MS (16)

static void submit_work_done_increment(struct nvhost_job *job)
{
	struct nvhost_channel *ch = job->ch;
	struct nvhost_syncpt *sp = &nvhost_get_host(ch->dev)->syncpt;
	struct nvhost_device_data *pdata = platform_get_drvdata(ch->dev);

	if (!pdata->push_work_done)
		return;

	/* make the last increment at job boundary. this will ensure
	 * that the user command buffer is no longer in use */
	job->sp[0].fence = nvhost_syncpt_incr_max(sp, job->sp[0].id, 1);
	nvhost_cdma_push(&ch->cdma,
			 nvhost_opcode_setclass(NV_HOST1X_CLASS_ID, 0, 1),
			 job->sp[0].id);
}

static void serialize(struct nvhost_job *job)
{
	struct nvhost_channel *ch = job->ch;
	struct nvhost_syncpt *sp = &nvhost_get_host(ch->dev)->syncpt;
	struct nvhost_device_data *pdata = platform_get_drvdata(ch->dev);
	int i;

	if (!job->serialize && !pdata->serialize)
		return;

	/*
	 * Force serialization by inserting a host wait for the
	 * previous job to finish before this one can commence.
	 *
	 * NOTE! This cannot be packed because otherwise we might
	 * overwrite the RESTART opcode at the end of the push
	 * buffer.
	 */

	for (i = 0; i < job->num_syncpts; ++i) {
		u32 id = job->sp[i].id;
		u32 max = nvhost_syncpt_read_max(sp, id);

		nvhost_cdma_push(&ch->cdma,
			nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
				host1x_uclass_load_syncpt_payload_32_r(), 1),
				max);
		nvhost_cdma_push(&ch->cdma,
			nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
				host1x_uclass_wait_syncpt_32_r(), 1),
				id);
	}
}

#if defined(CONFIG_TEGRA_GRHOST_SYNC)
static int validate_syncpt_id_cb(struct nvhost_ctrl_sync_fence_info info,
				 void *data)
{
	struct nvhost_syncpt *sp = data;

	if (!nvhost_syncpt_is_valid_hw_pt(sp, info.id))
		return -EINVAL;

	return 0;
}

static int push_wait_cb(struct nvhost_ctrl_sync_fence_info info, void *data)
{
	struct nvhost_channel *ch = data;
	struct nvhost_master *host = nvhost_get_host(ch->dev);
	struct nvhost_syncpt *sp = &host->syncpt;

	if (nvhost_syncpt_is_expired(sp, info.id, info.thresh))
		return 0;

	nvhost_cdma_push(&ch->cdma,
		nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
			host1x_uclass_load_syncpt_payload_32_r(), 1),
			info.thresh);
	nvhost_cdma_push(&ch->cdma,
		nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
			host1x_uclass_wait_syncpt_32_r(), 1),
			info.id);

	return 0;
}

static void add_sync_waits(struct nvhost_channel *ch, int fd)
{
	struct nvhost_master *host = nvhost_get_host(ch->dev);
	struct nvhost_syncpt *sp = &host->syncpt;
	struct nvhost_fence *fence;

	if (fd < 0)
		return;

	fence = nvhost_fence_get(fd);
	if (!fence)
		return;

	if (nvhost_fence_foreach_pt(fence, validate_syncpt_id_cb, sp)) {
		nvhost_fence_put(fence);
		return;
	}

	/*
	 * Force serialization by inserting a host wait for the
	 * previous job to finish before this one can commence.
	 *
	 * NOTE! This cannot be packed because otherwise we might
	 * overwrite the RESTART opcode at the end of the push
	 * buffer.
	 */
	nvhost_fence_foreach_pt(fence, push_wait_cb, ch);

	nvhost_fence_put(fence);
}
#else
static void add_sync_waits(struct nvhost_channel *ch, int fd)
{
	(void)ch;
	(void)fd;
}
#endif

static void push_waits(struct nvhost_job *job)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(job->ch->dev);
	struct nvhost_syncpt *sp = &nvhost_get_host(job->ch->dev)->syncpt;
	struct nvhost_channel *ch = job->ch;
	int i;

	for (i = 0; i < job->num_waitchk; i++) {
		struct nvhost_waitchk *wait = &job->waitchk[i];

		/* skip pushing waits if we allow them (map-at-open mode)
		 * and userspace wants to push a wait to some explicit
		 * position */
		if (pdata->resource_policy == RESOURCE_PER_DEVICE && wait->mem)
			continue;

		/* Skip pushing wait if it has already been expired */
		if (nvhost_syncpt_is_expired(sp, wait->syncpt_id,
					     wait->thresh))
			continue;

		nvhost_cdma_push(&ch->cdma,
			nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
				host1x_uclass_load_syncpt_payload_32_r(), 1),
				wait->thresh);
		nvhost_cdma_push(&ch->cdma,
			nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
				host1x_uclass_wait_syncpt_32_r(), 1),
				wait->syncpt_id);
	}

	for (i = 0; i < job->num_gathers; i++) {
		struct nvhost_job_gather *g = &job->gathers[i];
		add_sync_waits(job->ch, g->pre_fence);
	}
}

static inline int get_streamid(struct nvhost_job *job)
{
	struct platform_device *pdev = job->ch->dev;
	struct platform_device *host_dev = nvhost_get_host(pdev)->dev;
	int streamid;

	/* if vm is defined, take vm specific */
	if (job->ch->vm)
		return nvhost_vm_get_id(job->ch->vm);

	/* attempt using the engine streamid */
	streamid = nvhost_vm_get_hwid(pdev, nvhost_host1x_get_vmid(host_dev));
	if (streamid >= 0)
		return streamid;

	return nvhost_vm_get_bypass_hwid();
}

static void submit_setstreamid(struct nvhost_job *job)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(job->ch->dev);
	int streamid = get_streamid(job);
	int i;

	for (i = 0; i < ARRAY_SIZE(pdata->vm_regs); i++) {
		if (!pdata->vm_regs[i].addr)
			return;
		if (!pdata->vm_regs[i].dynamic)
			continue;

		nvhost_cdma_push(&job->ch->cdma,
			nvhost_opcode_setpayload(streamid),
			nvhost_opcode_setstreamid(
			pdata->vm_regs[i].addr >> 2));
	}
}

#ifdef NVHOST_HAS_SUBMIT_HOST1XSTREAMID
#include "host1x/submit_host1x_streamid.h"
#endif

static void submit_work(struct nvhost_job *job)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(job->ch->dev);
	void *cpuva = NULL;
	int i;

	/* First, move us into host class */
	u32 cur_class = NV_HOST1X_CLASS_ID;

	nvhost_cdma_push(&job->ch->cdma,
			 nvhost_opcode_acquire_mlock(cur_class),
			 nvhost_opcode_setclass(cur_class, 0, 0));
#ifdef NVHOST_HAS_SUBMIT_HOST1XSTREAMID
	submit_host1xstreamid(job);
#endif

	/* make all waits in the beginning */
	push_waits(job);

	/* push user gathers */
	for (i = 0; i < job->num_gathers; i++) {
		struct nvhost_job_gather *g = &job->gathers[i];
		u32 op1 = NVHOST_OPCODE_NOOP;
		u32 op2 = NVHOST_OPCODE_NOOP;

		/* handle class changing */
		if (cur_class != g->class_id) {
			/* first, release current class */
			nvhost_cdma_push(&job->ch->cdma,
					NVHOST_OPCODE_NOOP,
					nvhost_opcode_release_mlock(cur_class));

			/* As per bug: 200406973, Host1x HW expects these
			 * initial commands in sequence. Otherwise, Host1x HW
			 * will raise an interrupt.
			 * - Acquire Mlock
			 * - SetClass
			 * - SetStreamID in sequence.
			 */

			/* acquire lock of the new class */
			op1 = nvhost_opcode_acquire_mlock(g->class_id);
			op2 = nvhost_opcode_setclass(g->class_id, 0, 0);

			/* ..and finally, push opcode pair to hardware */
			nvhost_cdma_push(&job->ch->cdma, op1, op2);

			/* update current class */
			cur_class = g->class_id;
			if (g->class_id != NV_HOST1X_CLASS_ID)
				submit_setstreamid(job);
#ifdef NVHOST_HAS_SUBMIT_HOST1XSTREAMID
			else
				submit_host1xstreamid(job);
#endif

			/* initialize class context */
			if (cur_class != NV_HOST1X_CLASS_ID) {
				if (pdata->init_class_context)
					pdata->init_class_context(job->ch->dev,
								&job->ch->cdma);
				if (pdata->enable_timestamps)
					pdata->enable_timestamps(job->ch->dev,
						&job->ch->cdma,
						job->engine_timestamps.dma);
			}
		}

		op1 = nvhost_opcode_gather(g->words);
		op2 = job->gathers[i].mem_base + g->offset;

		if (nvhost_debug_trace_cmdbuf)
			cpuva = dma_buf_vmap(g->buf);

		nvhost_cdma_push_gather(&job->ch->cdma,
				cpuva,
				job->gathers[i].mem_base,
				g->offset,
				op1, op2);
		if (cpuva)
			dma_buf_vunmap(g->buf, cpuva);
	}

	/* wait all work to complete */
	serialize(job);

	/* make final increment */
	submit_work_done_increment(job);

	/* release the engine */
	nvhost_cdma_push(&job->ch->cdma,
			NVHOST_OPCODE_NOOP,
			nvhost_opcode_release_mlock(cur_class));
}

static void set_mlock_timeout(struct nvhost_channel *ch)
{
	u32 mlock_timeout;
	struct nvhost_master *host = nvhost_get_host(ch->dev);
	struct nvhost_device_data *pdata = platform_get_drvdata(ch->dev);
	struct nvhost_device_data *host_pdata = platform_get_drvdata(host->dev);

	/* set mlock timeout */
	if (host->info.vmserver_owns_engines) {
		u64 clk_rate_khz = clk_get_rate(host_pdata->clk[0]) / 1000;
		mlock_timeout = clk_rate_khz * MLOCK_TIMEOUT_MS;

		if (pdata->mlock_timeout_factor)
			mlock_timeout *= pdata->mlock_timeout_factor;

		host1x_channel_writel(ch, host1x_channel_intrmask_r(), 1);
		host1x_channel_writel(ch, host1x_channel_mlock_timeout_r(),
					mlock_timeout);
	}
}

static int host1x_channel_submit(struct nvhost_job *job)
{
	struct nvhost_channel *ch = job->ch;
	struct platform_device *host_dev = nvhost_get_host(job->ch->dev)->dev;
	struct nvhost_syncpt *sp = &nvhost_get_host(job->ch->dev)->syncpt;
	u32 prev_max = 0;
	int err, i;
	void *completed_waiters[NVHOST_SUBMIT_MAX_NUM_SYNCPT_INCRS];
	int streamid;

	memset(completed_waiters, 0, sizeof(void *) * job->num_syncpts);

	/* Turn on the client module and host1x */
	for (i = 0; i < job->num_syncpts; ++i) {
		err = nvhost_module_busy(ch->dev);
		if (err) {
			nvhost_module_idle_mult(ch->dev, i);
			nvhost_putchannel(ch, i);
			return err;
		}

		nvhost_getchannel(ch);
	}

	/* before error checks, return current max */
	prev_max = job->sp->fence = nvhost_syncpt_read_max(sp, job->sp->id);

	/* get submit lock */
	err = mutex_lock_interruptible(&ch->submitlock);
	if (err) {
		nvhost_module_idle_mult(ch->dev, job->num_syncpts);
		nvhost_putchannel(ch, job->num_syncpts);
		goto error;
	}

	for (i = 0; i < job->num_syncpts; ++i) {
		completed_waiters[i] = nvhost_intr_alloc_waiter();
		if (!completed_waiters[i]) {
			nvhost_module_idle_mult(ch->dev, job->num_syncpts);
			nvhost_putchannel(ch, job->num_syncpts);
			mutex_unlock(&ch->submitlock);
			err = -ENOMEM;
			goto error;
		}
		if (nvhost_intr_has_pending_jobs(
			&nvhost_get_host(ch->dev)->intr, job->sp[i].id, ch))
			dev_warn(&ch->dev->dev,
				"%s: cross-channel dependencies on syncpt %d\n",
				__func__, job->sp[i].id);
	}

	/* get host1x streamid */
	streamid = nvhost_vm_get_hwid(host_dev, nvhost_host1x_get_vmid(host_dev));
	if (streamid < 0)
		streamid = nvhost_vm_get_bypass_hwid();

	/* set channel streamid */
	host1x_channel_writel(ch, host1x_channel_smmu_streamid_r(), streamid);

	set_mlock_timeout(ch);

	/* begin a CDMA submit */
	err = nvhost_cdma_begin(&ch->cdma, job);
	if (err) {
		nvhost_module_idle_mult(ch->dev, job->num_syncpts);
		nvhost_putchannel(ch, job->num_syncpts);
		mutex_unlock(&ch->submitlock);
		goto error;
	}

	/* determine fences for all syncpoints */
	for (i = 0; i < job->num_syncpts; ++i) {
		u32 incrs = job->sp[i].incrs;

		/* create a valid max for client managed syncpoints */
		if (nvhost_syncpt_client_managed(sp, job->sp[i].id)) {
			u32 min = nvhost_syncpt_read(sp, job->sp[i].id);
			nvhost_syncpt_set_max(sp, job->sp[i].id, min);
			nvhost_syncpt_set_manager(sp, job->sp[i].id, false);
		}

		job->sp[i].fence =
			nvhost_syncpt_incr_max(sp, job->sp[i].id, incrs);

		/* mark syncpoint used by this channel */
		nvhost_syncpt_get_ref(sp, job->sp[i].id);
		nvhost_syncpt_mark_used(sp, ch->chid, job->sp[i].id);
	}

	/* mark also client managed syncpoint used by this channel */
	if (job->client_managed_syncpt)
		nvhost_syncpt_mark_used(sp, ch->chid,
					job->client_managed_syncpt);

	/* push work to hardware */
	submit_work(job);

	/* end CDMA submit & stash pinned hMems into sync queue */
	nvhost_cdma_end(&ch->cdma, job);

	trace_nvhost_channel_submitted(ch->dev->name, prev_max,
		job->sp->fence);

	for (i = 0; i < job->num_syncpts; ++i) {
		/* schedule a submit complete interrupt */
		err = nvhost_intr_add_action(&nvhost_get_host(ch->dev)->intr,
			job->sp[i].id, job->sp[i].fence,
			NVHOST_INTR_ACTION_SUBMIT_COMPLETE, ch,
			completed_waiters[i],
			NULL);
		WARN(err, "Failed to set submit complete interrupt");
	}

	mutex_unlock(&ch->submitlock);

	return 0;

error:
	for (i = 0; i < job->num_syncpts; ++i)
		kfree(completed_waiters[i]);
	return err;
}

static int host1x_channel_init_security(struct platform_device *pdev,
	struct nvhost_channel *ch)
{
	u32 val = 0U;

	if (nvhost_dev_is_virtual(pdev) == false) {
		val = host1x_hypervisor_readl(pdev,
					host1x_channel_filter_gbuffer_r() +
					BIT_WORD(ch->chid) * sizeof(u32));
		host1x_hypervisor_writel(pdev,
					 host1x_channel_filter_gbuffer_r() +
					 BIT_WORD(ch->chid) * sizeof(u32),
					 val | BIT_MASK(ch->chid));
	}

	return 0;
}

static int host1x_channel_init(struct nvhost_channel *ch,
	struct nvhost_master *dev)
{
	ch->aperture = host1x_channel_aperture(dev->aperture, ch->chid);

	if (nvhost_dev_is_virtual(dev->dev) == false) {
		/* move channel to VM */
		host1x_hypervisor_writel(dev->dev,
				(host1x_channel_ch_vm_0_r() + ch->chid * 4),
				nvhost_host1x_get_vmid(dev->dev));
	}

	return 0;
}

static const struct nvhost_channel_ops host1x_channel_ops = {
	.init = host1x_channel_init,
	.submit = host1x_channel_submit,
	.init_gather_filter = host1x_channel_init_security,
};
