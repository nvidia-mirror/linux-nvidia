// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2023, NVIDIA CORPORATION & AFFILIATES. All rights reserved.

#define PMUNAME		"arm_gicv3"
#define DRVNAME		PMUNAME "_pmu"
#define pr_fmt(fmt)	DRVNAME ": " fmt

#include <linux/acpi.h>
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/bug.h>
#include <linux/cpumask.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/perf_event.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/smp.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#include <asm/local64.h>

#define GIC_PMU_MAX_HW_CNTRS	32

#define GICP_EVCNTR_32(n)	(0x000 + ((n) * 4))
#define GICP_EVCNTR_64(n)	(0x000 + ((n) * 8))
#define GICP_EVTYPER(n)		(0x400 + ((n) * 4))
#define GICP_FR(n)		(0xA00 + ((n) * 4))
#define GICP_CNTENSET0		0xC00
#define GICP_CNTENCLR0		0xC20
#define GICP_INTENSET0		0xC40
#define GICP_INTENCLR0		0xC60
#define GICP_OVSCLR0		0xC80
#define GICP_CFGR		0xE00
#define GICP_CR			0xE04

#define GICP_CR_E		BIT(0)
#define GICP_CFGR_NCTR		GENMASK(5, 0)
#define GICP_CFGR_SIZE		GENMASK(13, 8)
#define GICP_FR_DISABLE		0xC0000000

#define GIC_PMU_ACTIVE_CPU_MASK 0x0
#define GIC_PMU_ASSOCIATED_CPU_MASK 0x1

/*
 * Format for raw filter (config1)
 * Bits - Field
 * [32] - enable_filter
 *	0 = Disable filter
 *	1 = Enable filter
 * [31:30] - filter_type
 *	0b00 = Filter on core
 *	0b01 = Filter on INTID
 *	0b10 = Filter on chip or ITS
 *	0b11 = Reserved, no effect
 * [29] - filter_encoding
 *	0 = Filter on range
 *	1 = Filter on an exact match
 * [28:16 - reserved]
 * [15:0] - filter
 *	If filter_encoding = 1, count events that are associated with
 *	an exact match of the filter_type.
 *	If filter_encoding = 0, count events that are associated within
 *	this range of the filter_type. The least-significant bit[0]
 *	indicates the uppermost of a contiguous span of least-significant
 *	bits that are ignored for the purposes of matching.
 *	e.g. 0b11110111_11110111 matches with values of 0b11110111_1111xxxx
 *	so the range is 0b11110111_11110000 to 0b11110111_11111111
 */
#define CONFIG1_FR(_config)		(_config & 0xffffffff)
#define CONFIG1_FR_ENABLE(_config)	(_config >> 32)

#define GIC_PMU_COUNTER_WIDTH_32	32
#define GIC_PMU_COUNTER_WIDTH_64	64

#define GIC_PMU_COUNTER_MASK(_width) \
	GENMASK_ULL(((_width) - 1), 0)

#define GIC_EXT_ATTR(_name, _func, _config)		\
	(&((struct dev_ext_attribute[]) {				\
		{							\
			.attr = __ATTR(_name, 0444, _func, NULL),	\
			.var = (void *)_config				\
		}							\
	})[0].attr.attr)

#define GIC_EVENT_ATTR(_name, _config) \
	GIC_EXT_ATTR(_name, gic_pmu_sysfs_event_show, (unsigned long)_config)

#define GIC_FORMAT_ATTR(_name, _config) \
	GIC_EXT_ATTR(_name, gic_pmu_sysfs_format_show, (char *)_config)

#define GIC_CPUMASK_ATTR(_name, _config) \
	GIC_EXT_ATTR(_name, gic_pmu_cpumask_show, (unsigned long)_config)

struct gic_hw_events {
	DECLARE_BITMAP(used_mask, GIC_PMU_MAX_HW_CNTRS);
	struct perf_event *events[GIC_PMU_MAX_HW_CNTRS];
};

/*
 * struct gic_pmu	- GIC PMU descriptor
 *
 * @hw_events		: Holds the event counter state.
 * @associated_cpus	: CPUs attached to the GIC.
 * @active_cpu		: CPU to which the PMU is bound for accesses.
 * @cpuhp_node		: Node for CPU hotplug notifier link.
 * @num_counters	: Number of event counters implemented by the PMU.
 * @counter_width	: Width of supported PMU counters.
 * @irq			: Interrupt line for counter overflow.
 * @base		: Base address for GICP registers.
 * @read_counter	: Read 32/64 bit counter callback.
 * @write_counter	: Write 32/64 bit counter callback.
 */
struct gic_pmu {
	struct pmu		pmu;
	struct device		*dev;
	struct gic_hw_events	hw_events;
	cpumask_t		associated_cpus;
	cpumask_t		active_cpu;
	struct hlist_node	cpuhp_node;
	s8			num_counters;
	s8			counter_width;
	int			irq;
	void __iomem		*base;
	u64 (*read_counter)	(struct perf_event *event);
	void (*write_counter)	(struct perf_event *event, u64 val);
};

static unsigned long gic_pmu_cpuhp_state;

static inline struct gic_pmu *to_gic_pmu(struct pmu *pmu)
{
	return container_of(pmu, struct gic_pmu, pmu);
}

static ssize_t gic_pmu_sysfs_event_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct dev_ext_attribute *eattr =
		container_of(attr, struct dev_ext_attribute, attr);

	return snprintf(buf, PAGE_SIZE, "event=0x%lx\n",
					(unsigned long)eattr->var);
}

static ssize_t gic_pmu_sysfs_format_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct dev_ext_attribute *eattr =
		container_of(attr, struct dev_ext_attribute, attr);

	return snprintf(buf, PAGE_SIZE, "%s\n", (char *)eattr->var);
}

static ssize_t gic_pmu_cpumask_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct pmu *pmu = dev_get_drvdata(dev);
	struct gic_pmu *gic_pmu = to_gic_pmu(pmu);
	struct dev_ext_attribute *eattr = container_of(attr,
					struct dev_ext_attribute, attr);
	unsigned long mask_id = (unsigned long)eattr->var;
	const cpumask_t *cpumask;

	switch (mask_id) {
	case GIC_PMU_ACTIVE_CPU_MASK:
		cpumask = &gic_pmu->active_cpu;
		break;
	case GIC_PMU_ASSOCIATED_CPU_MASK:
		cpumask = &gic_pmu->associated_cpus;
		break;
	default:
		return 0;
	}
	return cpumap_print_to_pagebuf(true, buf, cpumask);
}

static struct attribute *gic_pmu_format_attrs[] = {
	GIC_FORMAT_ATTR(event, "config:0-31"),
	GIC_FORMAT_ATTR(filter, "config1:0-15"),
	GIC_FORMAT_ATTR(filter_encoding, "config1:29"),
	GIC_FORMAT_ATTR(filter_type, "config1:30-31"),
	GIC_FORMAT_ATTR(enable_filter, "config1:32"),
	NULL,
};

static const struct attribute_group gic_pmu_format_attr_group = {
	.name = "format",
	.attrs = gic_pmu_format_attrs,
};

static struct attribute *gic_pmu_event_attrs[] = {
	GIC_EVENT_ATTR(clk, 0x0),
	GIC_EVENT_ATTR(clk_ng, 0x1),
	GIC_EVENT_ATTR(dn_msg, 0x4),
	GIC_EVENT_ATTR(dn_set, 0x5),
	GIC_EVENT_ATTR(dn_set1Ofn, 0x6),
	GIC_EVENT_ATTR(up_msg, 0x8),
	GIC_EVENT_ATTR(up_act, 0x9),
	GIC_EVENT_ATTR(up_rel, 0xa),
	GIC_EVENT_ATTR(up_actrel, 0xb),
	GIC_EVENT_ATTR(up_set_comp, 0xc),
	GIC_EVENT_ATTR(up_deact, 0xd),
	GIC_EVENT_ATTR(sgi_brd, 0x10),
	GIC_EVENT_ATTR(sgi_tar, 0x11),
	GIC_EVENT_ATTR(sgi_all, 0x12),
	GIC_EVENT_ATTR(sgi_acc, 0x13),
	GIC_EVENT_ATTR(sgi_brd_cc_in, 0x14),
	GIC_EVENT_ATTR(sgi_tar_cc_in, 0x15),
	GIC_EVENT_ATTR(sgi_tar_cc_out, 0x16),
	GIC_EVENT_ATTR(its_nll_lpi, 0x20),
	GIC_EVENT_ATTR(its_ll_lpi, 0x21),
	GIC_EVENT_ATTR(its_lpi, 0x22),
	GIC_EVENT_ATTR(its_lpi_cmd, 0x23),
	GIC_EVENT_ATTR(its_did_miss, 0x24),
	GIC_EVENT_ATTR(its_vid_miss, 0x25),
	GIC_EVENT_ATTR(its_col_miss, 0x26),
	GIC_EVENT_ATTR(its_lat, 0x27),
	GIC_EVENT_ATTR(its_mpfa, 0x28),
	GIC_EVENT_ATTR(lpi_cc_out, 0x29),
	GIC_EVENT_ATTR(lpi_cmd_cc_out, 0x2a),
	GIC_EVENT_ATTR(lpi_cc_in, 0x2b),
	GIC_EVENT_ATTR(lpi_cmd_cc_in, 0x2c),
	GIC_EVENT_ATTR(lpi_own_stored, 0x30),
	GIC_EVENT_ATTR(lpi_ool_stored, 0x31),
	GIC_EVENT_ATTR(lpi_hit_en, 0x32),
	GIC_EVENT_ATTR(lpi_hit_dis, 0x33),
	GIC_EVENT_ATTR(lpi_hit, 0x34),
	GIC_EVENT_ATTR(lpi_match, 0x35),
	GIC_EVENT_ATTR(lpi_fas, 0x36),
	GIC_EVENT_ATTR(lpi_prop_en, 0x37),
	GIC_EVENT_ATTR(lpi_prop_dis, 0x38),
	GIC_EVENT_ATTR(lpi_prop, 0x39),
	GIC_EVENT_ATTR(lpi_comp_inc_merge, 0x3a),
	GIC_EVENT_ATTR(spi_col_msg, 0x50),
	GIC_EVENT_ATTR(spi_enabled, 0x51),
	GIC_EVENT_ATTR(spi_disabled, 0x52),
	GIC_EVENT_ATTR(spi_pending_set, 0x53),
	GIC_EVENT_ATTR(spi_pending_clr, 0x54),
	GIC_EVENT_ATTR(spi_match, 0x55),
	GIC_EVENT_ATTR(spi_cc_in, 0x57),
	GIC_EVENT_ATTR(spi_cc_out, 0x58),
	GIC_EVENT_ATTR(spi_cc_deact, 0x5a),
	GIC_EVENT_ATTR(pt_in_en, 0x60),
	GIC_EVENT_ATTR(pt_in_dis, 0x61),
	GIC_EVENT_ATTR(pt_pri, 0x62),
	GIC_EVENT_ATTR(pt_in, 0x63),
	GIC_EVENT_ATTR(pt_match, 0x64),
	GIC_EVENT_ATTR(pt_out_en, 0x65),
	GIC_EVENT_ATTR(pt_out_dis, 0x66),
	GIC_EVENT_ATTR(pt_out, 0x67),
	GIC_EVENT_ATTR(pt_block_sent_cc, 0x68),
	GIC_EVENT_ATTR(spi_cc_latency, 0x70),
	GIC_EVENT_ATTR(spi_cc_lat_wait, 0x71),
	GIC_EVENT_ATTR(lpi_cc_latency, 0x72),
	GIC_EVENT_ATTR(lpi_cc_lat_wait, 0x73),
	GIC_EVENT_ATTR(sgi_cc_latency, 0x74),
	GIC_EVENT_ATTR(sgi_lat_wait, 0x75),
	GIC_EVENT_ATTR(acc, 0x80),
	GIC_EVENT_ATTR(oflow, 0x81),

	NULL,
};

static const struct attribute_group gic_pmu_events_attr_group = {
	.name = "events",
	.attrs = gic_pmu_event_attrs,
};

static struct attribute *gic_pmu_cpumask_attrs[] = {
	GIC_CPUMASK_ATTR(cpumask, GIC_PMU_ACTIVE_CPU_MASK),
	GIC_CPUMASK_ATTR(associated_cpus, GIC_PMU_ASSOCIATED_CPU_MASK),
	NULL,
};

static const struct attribute_group gic_pmu_cpumask_attr_group = {
	.attrs = gic_pmu_cpumask_attrs,
};

static const struct attribute_group *gic_pmu_attr_groups[] = {
	&gic_pmu_cpumask_attr_group,
	&gic_pmu_events_attr_group,
	&gic_pmu_format_attr_group,
	NULL,
};

static int gic_pmu_get_online_cpu_any_but(struct gic_pmu *gic_pmu, int cpu)
{
	struct cpumask online_supported;

	cpumask_and(&online_supported,
			 &gic_pmu->associated_cpus, cpu_online_mask);
	return cpumask_any_but(&online_supported, cpu);
}

static inline u64 gic_pmu_read_counter_32(struct perf_event *event)
{
	u64 val;
	struct gic_pmu *gic_pmu = to_gic_pmu(event->pmu);
	int idx = event->hw.idx;

	val = ioread32(gic_pmu->base + GICP_EVCNTR_32(idx));

	return val;
}

static inline u64 gic_pmu_read_counter_64(struct perf_event *event)
{
	u64 val;
	struct gic_pmu *gic_pmu = to_gic_pmu(event->pmu);
	int idx = event->hw.idx;

	val = ioread64(gic_pmu->base + GICP_EVCNTR_64(idx));

	return val;
}

static void gic_pmu_write_counter_32(struct perf_event *event, u64 val)
{
	struct gic_pmu *gic_pmu = to_gic_pmu(event->pmu);
	int idx = event->hw.idx;

	iowrite32(val, gic_pmu->base + GICP_EVCNTR_32(idx));

}

static void gic_pmu_write_counter_64(struct perf_event *event, u64 val)
{
	struct gic_pmu *gic_pmu = to_gic_pmu(event->pmu);
	int idx = event->hw.idx;

	iowrite64(val, gic_pmu->base + GICP_EVCNTR_64(idx));
}

static int gic_pmu_get_event_idx(struct gic_hw_events *hw_events,
				 struct perf_event *event)
{
	int idx;
	struct gic_pmu *gic_pmu = to_gic_pmu(event->pmu);
	unsigned long *used_mask = hw_events->used_mask;

	idx = find_first_zero_bit(used_mask, gic_pmu->num_counters);
	if (idx >= gic_pmu->num_counters)
		return -EAGAIN;
	set_bit(idx, hw_events->used_mask);
	return idx;
}

static void gic_pmu_enable_counter(struct gic_pmu *gic_pmu, int idx)
{
	iowrite32(BIT(idx), gic_pmu->base + GICP_INTENSET0);
	iowrite32(BIT(idx), gic_pmu->base + GICP_CNTENSET0);
}

static void gic_pmu_disable_counter(struct gic_pmu *gic_pmu, int idx)
{
	iowrite32(BIT(idx), gic_pmu->base + GICP_CNTENCLR0);
	iowrite32(BIT(idx), gic_pmu->base + GICP_INTENCLR0);
}

static inline void gic_pmu_set_event(struct gic_pmu *gic_pmu,
				     struct perf_event *event)
{
	int idx = event->hw.idx;

	iowrite32(event->hw.config_base, gic_pmu->base + GICP_EVTYPER(idx));
	if (CONFIG1_FR_ENABLE(event->attr.config1))
		iowrite32(CONFIG1_FR(event->attr.config1), gic_pmu->base + GICP_FR(idx));
	else
		iowrite32(GICP_FR_DISABLE, gic_pmu->base + GICP_FR(idx));
}

static void gic_pmu_event_update(struct perf_event *event)
{
	struct gic_pmu *gic_pmu = to_gic_pmu(event->pmu);
	struct hw_perf_event *hwc = &event->hw;
	u64 delta, prev_count, new_count;

	do {
		/* We may also be called from the irq handler */
		prev_count = local64_read(&hwc->prev_count);
		new_count = gic_pmu->read_counter(event);
	} while (local64_cmpxchg(&hwc->prev_count, prev_count, new_count) !=
			 prev_count);
	delta = (new_count - prev_count) & GIC_PMU_COUNTER_MASK(gic_pmu->counter_width);
	local64_add(delta, &event->count);
}

static void gic_pmu_read(struct perf_event *event)
{
	gic_pmu_event_update(event);
}

static inline u32 gic_pmu_get_reset_overflow(struct gic_pmu *gic_pmu)
{
	u32 val = ioread32(gic_pmu->base + GICP_OVSCLR0);
	/* Clear the bit */
	iowrite32(val, gic_pmu->base + GICP_OVSCLR0);
	return val;
}

/*
 * gic_pmu_set_event_period: Set the period for the counter.
 *
 * To handle cases of extreme interrupt latency, we program
 * the counter with half of the max count for the counters.
 */
static void gic_pmu_set_event_period(struct perf_event *event)
{
	struct gic_pmu *gic_pmu = to_gic_pmu(event->pmu);
	u64 val = GIC_PMU_COUNTER_MASK(gic_pmu->counter_width) >> 1;

	local64_set(&event->hw.prev_count, val);
	gic_pmu->write_counter(event, val);
}

static irqreturn_t gic_pmu_handle_irq(int irq_num, void *dev)
{
	int i;
	bool handled = false;
	struct gic_pmu *gic_pmu = dev;
	struct gic_hw_events *hw_events = &gic_pmu->hw_events;
	unsigned long overflow;

	overflow = gic_pmu_get_reset_overflow(gic_pmu);
	if (!overflow)
		return IRQ_NONE;

	for_each_set_bit(i, &overflow, GIC_PMU_MAX_HW_CNTRS) {
		struct perf_event *event = hw_events->events[i];

		if (!event)
			continue;
		gic_pmu_event_update(event);
		gic_pmu_set_event_period(event);
		handled = true;
	}

	return IRQ_RETVAL(handled);
}

static void gic_pmu_start(struct perf_event *event, int pmu_flags)
{
	struct gic_pmu *gic_pmu = to_gic_pmu(event->pmu);

	/* We always reprogram the counter */
	if (pmu_flags & PERF_EF_RELOAD)
		WARN_ON(!(event->hw.state & PERF_HES_UPTODATE));
	gic_pmu_set_event_period(event);
	gic_pmu_set_event(gic_pmu, event);
	event->hw.state = 0;
	gic_pmu_enable_counter(gic_pmu, event->hw.idx);
}

static void gic_pmu_stop(struct perf_event *event, int pmu_flags)
{
	struct gic_pmu *gic_pmu = to_gic_pmu(event->pmu);

	if (event->hw.state & PERF_HES_STOPPED)
		return;
	gic_pmu_disable_counter(gic_pmu, event->hw.idx);
	gic_pmu_event_update(event);
	event->hw.state |= PERF_HES_STOPPED | PERF_HES_UPTODATE;
}

static int gic_pmu_add(struct perf_event *event, int flags)
{
	struct gic_pmu *gic_pmu = to_gic_pmu(event->pmu);
	struct gic_hw_events *hw_events = &gic_pmu->hw_events;
	struct hw_perf_event *hwc = &event->hw;
	int idx;

	idx = gic_pmu_get_event_idx(hw_events, event);
	if (idx < 0)
		return idx;

	hwc->idx = idx;
	hw_events->events[idx] = event;
	hwc->state = PERF_HES_STOPPED | PERF_HES_UPTODATE;

	if (flags & PERF_EF_START)
		gic_pmu_start(event, PERF_EF_RELOAD);

	perf_event_update_userpage(event);
	return 0;
}

static void gic_pmu_del(struct perf_event *event, int flags)
{
	struct gic_pmu *gic_pmu = to_gic_pmu(event->pmu);
	struct gic_hw_events *hw_events = &gic_pmu->hw_events;
	struct hw_perf_event *hwc = &event->hw;
	int idx = hwc->idx;

	gic_pmu_stop(event, PERF_EF_UPDATE);
	hw_events->events[idx] = NULL;
	clear_bit(idx, hw_events->used_mask);
	perf_event_update_userpage(event);
}

static void gic_pmu_enable(struct pmu *pmu)
{
	u32 cr;
	struct gic_pmu *gic_pmu = to_gic_pmu(pmu);

	/* If no counters are added, skip enabling the PMU */
	if (bitmap_empty(gic_pmu->hw_events.used_mask, GIC_PMU_MAX_HW_CNTRS))
		return;

	cr = GICP_CR_E;
	iowrite32(cr, gic_pmu->base + GICP_CR);
}

static void gic_pmu_disable(struct pmu *pmu)
{
	u32 cr;
	struct gic_pmu *gic_pmu = to_gic_pmu(pmu);

	cr = 0;
	iowrite32(cr, gic_pmu->base + GICP_CR);
}

static bool gic_pmu_validate_event(struct pmu *pmu,
				   struct gic_hw_events *hw_events,
				   struct perf_event *event)
{
	if (is_software_event(event))
		return true;
	/* Reject groups spanning multiple HW PMUs. */
	if (event->pmu != pmu)
		return false;
	return gic_pmu_get_event_idx(hw_events, event) >= 0;
}

/*
 * Make sure the group of events can be scheduled at once
 * on the PMU.
 */
static bool gic_pmu_validate_group(struct perf_event *event)
{
	struct perf_event *sibling, *leader = event->group_leader;
	struct gic_hw_events fake_hw;

	if (event->group_leader == event)
		return true;

	memset(fake_hw.used_mask, 0, sizeof(fake_hw.used_mask));
	if (!gic_pmu_validate_event(event->pmu, &fake_hw, leader))
		return false;
	for_each_sibling_event(sibling, leader) {
		if (!gic_pmu_validate_event(event->pmu, &fake_hw, sibling))
			return false;
	}
	return gic_pmu_validate_event(event->pmu, &fake_hw, event);
}

static int gic_pmu_event_init(struct perf_event *event)
{
	struct gic_pmu *gic_pmu = to_gic_pmu(event->pmu);

	if (event->attr.type != event->pmu->type)
		return -ENOENT;

	/* We don't support sampling */
	if (is_sampling_event(event)) {
		dev_dbg(gic_pmu->pmu.dev, "Can't support sampling events\n");
		return -EOPNOTSUPP;
	}

	/* We cannot support task bound events */
	if (event->cpu < 0 || event->attach_state & PERF_ATTACH_TASK) {
		dev_dbg(gic_pmu->pmu.dev, "Can't support per-task counters\n");
		return -EINVAL;
	}

	if (has_branch_stack(event)) {
		dev_dbg(gic_pmu->pmu.dev, "Can't support filtering\n");
		return -EINVAL;
	}

	if (!cpumask_test_cpu(event->cpu, &gic_pmu->associated_cpus)) {
		dev_dbg(gic_pmu->pmu.dev,
			 "Requested cpu is not associated with the GIC\n");
		return -EINVAL;
	}

	/*
	 * Choose the current active CPU to read the events. We don't want
	 * to migrate the event contexts, irq handling etc to the requested
	 * CPU.
	 */
	event->cpu = cpumask_first(&gic_pmu->active_cpu);
	if (event->cpu >= nr_cpu_ids)
		return -EINVAL;
	if (!gic_pmu_validate_group(event))
		return -EINVAL;

	event->hw.config_base = event->attr.config;
	return 0;
}

static struct gic_pmu *gic_pmu_alloc(struct platform_device *pdev)
{
	struct gic_pmu *gic_pmu;

	gic_pmu = devm_kzalloc(&pdev->dev, sizeof(*gic_pmu), GFP_KERNEL);
	if (!gic_pmu)
		return ERR_PTR(-ENOMEM);

	return gic_pmu;
}

/**
 * gic_pmu_dt_get_cpus: Get the list of CPUs in the socket
 * from device tree.
 */
static int gic_pmu_dt_get_cpus(struct device *dev, cpumask_t *mask)
{
	int i = 0, n, cpu;
	struct device_node *cpu_node;

	n = of_count_phandle_with_args(dev->of_node, "cpus", NULL);
	if (n <= 0)
		return -ENODEV;
	for (; i < n; i++) {
		cpu_node = of_parse_phandle(dev->of_node, "cpus", i);
		if (!cpu_node)
			break;
		cpu = of_cpu_node_to_id(cpu_node);
		of_node_put(cpu_node);
		/*
		 * We have to ignore the failures here and continue scanning
		 * the list to handle cases where the nr_cpus could be capped
		 * in the running kernel.
		 */
		if (cpu < 0)
			continue;
		cpumask_set_cpu(cpu, mask);
	}
	return 0;
}

static void gic_pmu_set_active_cpu(int cpu, struct gic_pmu *gic_pmu)
{
	cpumask_set_cpu(cpu, &gic_pmu->active_cpu);
	WARN_ON(irq_set_affinity(gic_pmu->irq, &gic_pmu->active_cpu));
}

static int gic_pmu_device_probe(struct platform_device *pdev)
{
	int irq, rc;
	struct gic_pmu *gic_pmu;
	struct fwnode_handle *fwnode = dev_fwnode(&pdev->dev);
	char *name;
	static atomic_t pmu_idx = ATOMIC_INIT(-1);
	void __iomem *base;
	u32 num_counters, counter_width;
	u32 reg;

	gic_pmu = gic_pmu_alloc(pdev);
	if (IS_ERR(gic_pmu))
		return PTR_ERR(gic_pmu);

	if (IS_ERR_OR_NULL(fwnode))
		return -ENOENT;

	if (is_of_node(fwnode))
		rc = gic_pmu_dt_get_cpus(&pdev->dev, &gic_pmu->associated_cpus);
	else
		return -ENOENT;

	if (rc) {
		dev_warn(&pdev->dev, "Failed to parse the CPUs\n");
		return rc;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -EINVAL;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	reg = ioread32(base + GICP_CFGR);
	num_counters = FIELD_GET(GICP_CFGR_NCTR, reg);
	if (num_counters == 0)
		return -EACCES;
	counter_width = FIELD_GET(GICP_CFGR_SIZE, reg);

	name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s_%d",
				PMUNAME, atomic_inc_return(&pmu_idx));
	if (!name)
		return -ENOMEM;
	if (irq) {
		rc = devm_request_irq(&pdev->dev, irq, gic_pmu_handle_irq,
				IRQF_NOBALANCING, name, gic_pmu);
		if (rc) {
			dev_warn(&pdev->dev, "Failed to request IRQ %d\n", irq);
			return rc;
		}
	}

	gic_pmu->irq = irq;
	gic_pmu->base = base;
	/* GICP_CFGR_NCTR is set to max counter index */
	gic_pmu->num_counters = num_counters + 1;
	/* GICP_CFGR_SIZE is set to max width bit index */
	gic_pmu->counter_width = counter_width + 1;
	gic_pmu->read_counter = gic_pmu->counter_width == GIC_PMU_COUNTER_WIDTH_32 ?
			gic_pmu_read_counter_32 : gic_pmu_read_counter_64;
	gic_pmu->write_counter = gic_pmu->counter_width == GIC_PMU_COUNTER_WIDTH_32 ?
			gic_pmu_write_counter_32 : gic_pmu_write_counter_64;
	platform_set_drvdata(pdev, gic_pmu);
	rc = cpuhp_state_add_instance(gic_pmu_cpuhp_state,
				      &gic_pmu->cpuhp_node);
	if (rc)
		return rc;

	gic_pmu->pmu = (struct pmu){
		.task_ctx_nr = perf_invalid_context,
		.module = THIS_MODULE,
		.pmu_enable = gic_pmu_enable,
		.pmu_disable = gic_pmu_disable,
		.event_init = gic_pmu_event_init,
		.add = gic_pmu_add,
		.del = gic_pmu_del,
		.start = gic_pmu_start,
		.stop = gic_pmu_stop,
		.read = gic_pmu_read,

		.attr_groups = gic_pmu_attr_groups,
		.capabilities = PERF_PMU_CAP_NO_EXCLUDE |
				(irq ? 0 : PERF_PMU_CAP_NO_INTERRUPT),
	};

	rc = perf_pmu_register(&gic_pmu->pmu, name, -1);
	if (rc) {
		cpuhp_state_remove_instance(gic_pmu_cpuhp_state,
					    &gic_pmu->cpuhp_node);
		irq_set_affinity(gic_pmu->irq, NULL);
	}
	return rc;
}

static int gic_pmu_device_remove(struct platform_device *pdev)
{
	struct gic_pmu *gic_pmu = platform_get_drvdata(pdev);

	perf_pmu_unregister(&gic_pmu->pmu);
	cpuhp_state_remove_instance(gic_pmu_cpuhp_state, &gic_pmu->cpuhp_node);
	irq_set_affinity(gic_pmu->irq, NULL);

	return 0;
}

static const struct of_device_id gic_pmu_of_match[] = {
	{ .compatible = "arm,gic-v3-pmu", },
	{},
};
MODULE_DEVICE_TABLE(of, gic_pmu_of_match);

static struct platform_driver gic_pmu_driver = {
	.driver = {
		.name = DRVNAME,
		.of_match_table = of_match_ptr(gic_pmu_of_match),
		.suppress_bind_attrs = true,
	},
	.probe = gic_pmu_device_probe,
	.remove = gic_pmu_device_remove,
};

static int gic_pmu_cpu_online(unsigned int cpu, struct hlist_node *node)
{
	struct gic_pmu *gic_pmu = hlist_entry_safe(node, struct gic_pmu,
						   cpuhp_node);

	if (!cpumask_test_cpu(cpu, &gic_pmu->associated_cpus))
		return 0;

	/* If the PMU is already managed, there is nothing to do */
	if (!cpumask_empty(&gic_pmu->active_cpu))
		return 0;

	gic_pmu_set_active_cpu(cpu, gic_pmu);

	return 0;
}

static int gic_pmu_cpu_teardown(unsigned int cpu, struct hlist_node *node)
{
	int dst;
	struct gic_pmu *gic_pmu = hlist_entry_safe(node, struct gic_pmu,
						   cpuhp_node);

	if (!cpumask_test_and_clear_cpu(cpu, &gic_pmu->active_cpu))
		return 0;

	dst = gic_pmu_get_online_cpu_any_but(gic_pmu, cpu);
	/* If there are no active CPUs in the GIC, leave IRQ disabled */
	if (dst >= nr_cpu_ids) {
		irq_set_affinity(gic_pmu->irq, NULL);
		return 0;
	}

	perf_pmu_migrate_context(&gic_pmu->pmu, cpu, dst);
	gic_pmu_set_active_cpu(dst, gic_pmu);

	return 0;
}

static int __init gic_pmu_init(void)
{
	int ret;

	ret = cpuhp_setup_state_multi(CPUHP_AP_ONLINE_DYN,
				      DRVNAME,
				      gic_pmu_cpu_online,
				      gic_pmu_cpu_teardown);
	if (ret < 0)
		return ret;
	gic_pmu_cpuhp_state = ret;
	ret = platform_driver_register(&gic_pmu_driver);
	if (ret < 0)
		cpuhp_remove_multi_state(gic_pmu_cpuhp_state);
	return ret;
}

static void __exit gic_pmu_exit(void)
{
	platform_driver_unregister(&gic_pmu_driver);
	cpuhp_remove_multi_state(gic_pmu_cpuhp_state);
}

module_init(gic_pmu_init);
module_exit(gic_pmu_exit);

MODULE_DESCRIPTION("Perf driver for ARM Generic Interrupt Controller (GICv3)");
MODULE_AUTHOR("Eric Funsten <efunsten@nvidia.com>");
MODULE_LICENSE("GPL v2");
