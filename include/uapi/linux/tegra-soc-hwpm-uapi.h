/*
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
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
 *
 * tegra-soc-hwpm-uapi.h:
 * This is the userspace API header for the Tegra SOC HWPM driver.
 */

#ifndef TEGRA_SOC_HWPM_UAPI_H
#define TEGRA_SOC_HWPM_UAPI_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define TEGRA_SOC_HWPM_DEV_NODE		"/dev/tegra-soc-hwpm"

/* The IPs which can be profiled */
enum tegra_soc_hwpm_ip {
	TEGRA_SOC_HWPM_IP_VI,
	TEGRA_SOC_HWPM_IP_ISP,
	TEGRA_SOC_HWPM_IP_VIC,
	TEGRA_SOC_HWPM_IP_OFA,
	TEGRA_SOC_HWPM_IP_PVA,
	TEGRA_SOC_HWPM_IP_NVDLA,
	TEGRA_SOC_HWPM_IP_MGBE,
	TEGRA_SOC_HWPM_IP_SCF,
	TEGRA_SOC_HWPM_IP_NVDEC,
	TEGRA_SOC_HWPM_IP_NVENC,
	TEGRA_SOC_HWPM_IP_PCIE,
	TEGRA_SOC_HWPM_IP_DISPLAY,
	TEGRA_SOC_HWPM_IP_MSS_CHANNEL,
	TEGRA_SOC_HWPM_IP_MSS_GPU_HUB,
	TEGRA_SOC_HWPM_IP_MSS_ISO_NISO_HUBS,
	TEGRA_SOC_HWPM_IP_MSS_MCF,
	TEGRA_SOC_HWPM_IP_MSS_NVLINK,
	TERGA_SOC_HWPM_NUM_IPS
};

/* TEGRA_CTRL_CMD_SOC_HWPM_DEVICE_INFO IOCTL */
struct tegra_soc_hwpm_device_info {
	__u32 chip;	/* chip id, eg. 0x23 (t23x) */
	__u32 chip_revision;	/* chip_id revision, eg. 0x4 (t234) */
	__u32 revision;	/* major-minor revision, eg. A01, A02 */
	__u32 platform;	/* Eg. Pre-Si, Si */
};

struct tegra_soc_hwpm_ip_floorsweep_info_query {
	/* input */
	__u16 ip_type;		/* enum tegra_soc_hwpm_ip */
	/* output */
#define TEGRA_SOC_HWPM_IP_STATUS_VALID		0
#define TEGRA_SOC_HWPM_IP_STATUS_INVALID	1
	__u8 status;		/* IP status */
	__u8 reserved1;
	__u32 reserved2;
	__u64 ip_inst_mask;	/* each set bit corresponds to an IP instance */
};

#define TEGRA_SOC_HWPM_IP_QUERIES_MAX	32
/* TEGRA_CTRL_CMD_SOC_HWPM_IP_FLOORSWEEP_INFO IOCTL */
struct tegra_soc_hwpm_ip_floorsweep_info {
	/* Holds queries */
	struct tegra_soc_hwpm_ip_floorsweep_info_query ip_fsinfo[TEGRA_SOC_HWPM_IP_QUERIES_MAX];
	__u32 num_queries;
};

enum tegra_soc_hwpm_timer_relation_cpu_clk {
	TEGRA_SOC_HWPM_TIMER_RELATION_CPU_CLK_OSTIME, /* FIXME: Is this needed? */
	TEGRA_SOC_HWPM_TIMER_RELATION_CPU_CLK_PLAT_API,
	TEGRA_SOC_HWPM_TIMER_RELATION_CPU_CLK_TSC /* FIXME: Is this needed? */
};

struct tegra_soc_hwpm_timer_relation_sample {
	__u64 cpu_time;
	__u64 gpu_time;
};

/* TEGRA_CTRL_CMD_SOC_HWPM_GET_GPU_CPU_TIME_CORRELATION_INFO IOCTL */
struct tegra_soc_hwpm_timer_relation {
	/*
	 * Inputs
	 */
	enum tegra_soc_hwpm_timer_relation_cpu_clk cpu_clk;

	/*
	 * Outputs
	 */
	struct tegra_soc_hwpm_timer_relation_sample sample;
};

/* The resources which can be reserved for profiling */
enum tegra_soc_hwpm_resource {
	TEGRA_SOC_HWPM_RESOURCE_VI,
	TEGRA_SOC_HWPM_RESOURCE_ISP,
	TEGRA_SOC_HWPM_RESOURCE_VIC,
	TEGRA_SOC_HWPM_RESOURCE_OFA,
	TEGRA_SOC_HWPM_RESOURCE_PVA,
	TEGRA_SOC_HWPM_RESOURCE_NVDLA,
	TEGRA_SOC_HWPM_RESOURCE_MGBE,
	TEGRA_SOC_HWPM_RESOURCE_SCF,
	TEGRA_SOC_HWPM_RESOURCE_NVDEC,
	TEGRA_SOC_HWPM_RESOURCE_NVENC,
	TEGRA_SOC_HWPM_RESOURCE_PCIE,
	TEGRA_SOC_HWPM_RESOURCE_DISPLAY,
	TEGRA_SOC_HWPM_RESOURCE_MSS_CHANNEL,
	TEGRA_SOC_HWPM_RESOURCE_MSS_GPU_HUB,
	TEGRA_SOC_HWPM_RESOURCE_MSS_ISO_NISO_HUBS,
	TEGRA_SOC_HWPM_RESOURCE_MSS_MCF,
	TEGRA_SOC_HWPM_RESOURCE_MSS_NVLINK,

	/*
	 * - SYS0 PERMON in RPG_PMG
	 * - PERFMUX: PMA_CHANNEL_PERFMUX_CONFIG_SECURE
	 */
	TEGRA_SOC_HWPM_RESOURCE_PMA,

	/*
	 * - PMA: Everything except PMA_CHANNEL_PERFMUX_CONFIG_SECURE
	 * - RTR: Entire aperture
	 */
	TEGRA_SOC_HWPM_RESOURCE_CMD_SLICE_RTR,

	TERGA_SOC_HWPM_NUM_RESOURCES
};

/* TEGRA_CTRL_CMD_SOC_HWPM_RESERVE_RESOURCE IOCTL */
struct tegra_soc_hwpm_reserve_resource {
	/*
	 * Inputs
	 */
	enum tegra_soc_hwpm_resource resource;
};

/* TEGRA_CTRL_CMD_SOC_HWPM_ALLOC_PMA_STREAM IOCTL */
struct tegra_soc_hwpm_alloc_pma_stream {
	/*
	 * Inputs
	 */
	__u64 stream_buf_size;
	__u64 stream_buf_fd;	/* NvRmMemHandle */
	__u64 mem_bytes_buf_fd;	/* NvRmMemHandle  */

	/*
	 * Outputs
	 */
	__u64 stream_buf_pma_va; /* SMMU mapped VA */
};

/*
 * TEGRA_CTRL_CMD_SOC_HWPM_QUERY_WHITELIST IOCTL
 *
 * This IOCTL needs to be called twice. In the first IOCTL call userspace will
 * set the whitelist pointer to NULL and the driver will return whitelist_size.
 * Userspace will then allocate a buffer to hold whitelist_size entries. In the
 * 2nd IOCTL call, userspace will provide the whitelist buffer pointer, and the
 * driver will fill in the buffer.
 */
struct tegra_soc_hwpm_query_whitelist {
	/*
	 * Inputs and Outputs
	 */
	__u64 *whitelist;

	/*
	 * Outputs
	 */
	__u64 whitelist_size; /* The number of entires in the map */
};

/* Register read/write op */
struct tegra_soc_hwpm_reg_op {
	__u64 phys_addr;

	/*
	 * For 32 bit reg-ops this is the value used for reading/writing.
	 * For 64 bit reg-ops this is the value of the lower register (i.e. phys_addr).
	 */
	__u32 reg_val_lo;

	/*
	 * For 32 bit reg-ops this value is unused.
	 * For 64 bit reg-ops this is the value of the higher register (i.e. phys_addr + 4).
	 */
	__u32 reg_val_hi;

	/*
	 * For 32 bit reg-ops this is the mask used for writing.
	 * For 64 bit reg-ops this is the mask for the lower register (i.e. phys_addr).
	 */
	__u32 mask_lo;

	/*
	 * For 32 bit reg-ops this mask is unused.
	 * For 64 bit reg-ops this is the mask for the higher register (i.e. phys_addr + 4).
	 */
	__u32 mask_hi;

/*
 * INVALID is first so the default value of 0 is not a valid value.
 * User is forced to pick a real value
 */
#define TEGRA_SOC_HWPM_REG_OP_CMD_INVALID	0
#define TEGRA_SOC_HWPM_REG_OP_CMD_RD32		1
#define TEGRA_SOC_HWPM_REG_OP_CMD_RD64		2
#define TEGRA_SOC_HWPM_REG_OP_CMD_WR32		3
#define TEGRA_SOC_HWPM_REG_OP_CMD_WR64		4
	__u8 cmd;

/*
 * Return RegOps status
 */
#define TEGRA_SOC_HWPM_REG_OP_STATUS_SUCCESS			0
#define TEGRA_SOC_HWPM_REG_OP_STATUS_INVALID_CMD		1
 /* FIXME: What is this error code for? */
#define TEGRA_SOC_HWPM_REG_OP_STATUS_INVALID_ADDR		2
 /* not in whitelist */
#define TEGRA_SOC_HWPM_REG_OP_STATUS_INSUFFICIENT_PERMISSIONS	3
#define TEGRA_SOC_HWPM_REG_OP_STATUS_WR_FAILED			4
	__u8 status;

	/* Explicit padding for 8 Byte alignment */
	__u8 reserved[6];
};

/* TEGRA_CTRL_CMD_SOC_HWPM_EXEC_REG_OPS IOCTL */
struct tegra_soc_hwpm_exec_reg_ops {
	/*
	 * Inputs and Outputs
	 */
#define TEGRA_SOC_HWPM_REG_OPS_SIZE	127
	struct tegra_soc_hwpm_reg_op ops[TEGRA_SOC_HWPM_REG_OPS_SIZE];
	__u32 op_count;

	/* Inputs */
/*
 * INVALID is first so the default value of 0 is not a valid value.
 * User is forced to pick a real value
 */
#define TEGRA_SOC_HWPM_REG_OP_MODE_INVALID		0
/* Fail the entire IOCTL on the first malformed REG_OP */
#define TEGRA_SOC_HWPM_REG_OP_MODE_FAIL_ON_FIRST	1
/*
 * If a malformed REG_OP is encountered, set the status in
 * struct tegra_soc_hwpm_reg_op.status. Then continue processing the remaining
 * REG_OPs.
 */
#define TEGRA_SOC_HWPM_REG_OP_MODE_CONT_ON_ERR		2
	__u8 mode;

	/* Output */
	__u8 b_all_reg_ops_passed;
};

/* TEGRA_CTRL_CMD_SOC_HWPM_UPDATE_GET_PUT IOCTL */
#define TEGRA_SOC_HWPM_MEM_BYTES_INVALID	0xffffffff
struct tegra_soc_hwpm_update_get_put {
	/*
	 * Inputs
	 */
	__u64 mem_bump; /* Increase SW get pointer by mem_bump bytes */
	__u8 b_stream_mem_bytes; /* Stream MEM_BYTES value to MEM_BYTES buffer */
	/*
	 * FIXME: This is not needed right now
	 * __u8 b_wait_for_stream;
	 */
	__u8 b_read_mem_head;
	__u8 b_check_overflow;

	/*
	 * Outputs
	 */
	__u64 mem_head; /* HW put pointer value */
	/*
	 * FIXME: This is not needed right now
	 * __u64 mem_bytes;
	 */
	__u8 b_overflowed;
};

/* IOCTL enum */
enum tegra_soc_hwpm_ioctl_num {
	TEGRA_SOC_HWPM_IOCTL_DEVICE_INFO,
	TEGRA_SOC_HWPM_IOCTL_FLOORSWEEP_INFO,
	TEGRA_SOC_HWPM_IOCTL_GET_GPU_CPU_TIME_CORRELATION_INFO,
	TEGRA_SOC_HWPM_IOCTL_RESERVE_RESOURCE,
	TEGRA_SOC_HWPM_IOCTL_ALLOC_PMA_STREAM,
	TEGRA_SOC_HWPM_IOCTL_BIND,
	TEGRA_SOC_HWPM_IOCTL_QUERY_WHITELIST,
	TEGRA_SOC_HWPM_IOCTL_EXEC_REG_OPS,
	TEGRA_SOC_HWPM_IOCTL_UPDATE_GET_PUT,
	TERGA_SOC_HWPM_NUM_IOCTLS
};

/* FIXME: Does this have to be unique? */
#define TEGRA_SOC_HWPM_IOC_MAGIC	  'P'

/*
 * IOCTL for finding which physical instances of an IP are present in the chip
 *
 * FIXME: Convert to batch mode - refer to NV2080_CTRL_CMD_FB_GET_FS_INFO
 */
#define TEGRA_CTRL_CMD_SOC_HWPM_DEVICE_INFO				\
		_IOWR(TEGRA_SOC_HWPM_IOC_MAGIC,				\
			TEGRA_SOC_HWPM_IOCTL_DEVICE_INFO,		\
			struct tegra_soc_hwpm_device_info)

/*
 * IOCTL for finding the relationship between the CPU and GPU timers
 */
#define	TEGRA_CTRL_CMD_SOC_HWPM_GET_GPU_CPU_TIME_CORRELATION_INFO		\
		_IOWR(TEGRA_SOC_HWPM_IOC_MAGIC,					\
			TEGRA_SOC_HWPM_IOCTL_GET_GPU_CPU_TIME_CORRELATION_INFO,	\
			struct tegra_soc_hwpm_timer_relation)

/*
 * IOCTL for reserving a resource for profiling
 *
 * This IOCTL can only be called before BIND.
 *
 * FIXME: Convert to batch mode - refer to NV2080_CTRL_CMD_FB_GET_FS_INFO
 */
#define	TEGRA_CTRL_CMD_SOC_HWPM_RESERVE_RESOURCE			\
			_IOW(TEGRA_SOC_HWPM_IOC_MAGIC,			\
				TEGRA_SOC_HWPM_IOCTL_RESERVE_RESOURCE,	\
				struct tegra_soc_hwpm_reserve_resource)

/*
 * IOCTL for allocating the PMA output stream
 *
 * This IOCTL can only be called before BIND.
 */
#define TEGRA_CTRL_CMD_SOC_HWPM_ALLOC_PMA_STREAM			\
			_IOWR(TEGRA_SOC_HWPM_IOC_MAGIC,			\
				TEGRA_SOC_HWPM_IOCTL_ALLOC_PMA_STREAM,	\
				struct tegra_soc_hwpm_alloc_pma_stream)

/*
 * IOCTL for finalizing the initialization
 *
 * SOC HWPM driver will program the HW to finalize the initialiation.
 *
 * The following IOCTLs can only be called before BIND:
 *    - RESERVE_RESOURCE
 *    - ALLOC_PMA_STREAM
 *
 * The following IOCTLs can only be called after BIND:
 *    - QUERY_WHITELIST
 *    - EXEC_REG_OPS
 *    - UPDATE_GET_PUT
 */
#define TEGRA_CTRL_CMD_BIND						\
			_IO(TEGRA_SOC_HWPM_IOC_MAGIC,			\
				TEGRA_SOC_HWPM_IOCTL_BIND)

/*
 * IOCTL for requesting the driver's whitelist map
 *
 * This IOCTL can only be called after the BIND IOCTL
 *
 * FIXME: "WHITELIST" might have to be renamed to "ALLOWLIST"
 */
#define	TEGRA_CTRL_CMD_SOC_HWPM_QUERY_WHITELIST				\
			_IOWR(TEGRA_SOC_HWPM_IOC_MAGIC,			\
				TEGRA_SOC_HWPM_IOCTL_QUERY_WHITELIST,	\
				struct tegra_soc_hwpm_query_whitelist)

/*
 * IOCTL for executing register read/write operations
 *
 * This IOCTL can only be called after the BIND IOCTL
 */
#define	TEGRA_CTRL_CMD_SOC_HWPM_EXEC_REG_OPS				\
			_IOWR(TEGRA_SOC_HWPM_IOC_MAGIC,			\
				TEGRA_SOC_HWPM_IOCTL_EXEC_REG_OPS,	\
				struct tegra_soc_hwpm_exec_reg_ops)

/*
 * IOCTL for updating get/put pointers for PMA streamout buffer
 *
 * This IOCTL can only be called after the BIND IOCTL
 */
#define	TEGRA_CTRL_CMD_SOC_HWPM_UPDATE_GET_PUT				\
			_IOWR(TEGRA_SOC_HWPM_IOC_MAGIC,			\
				TEGRA_SOC_HWPM_IOCTL_UPDATE_GET_PUT,	\
				struct tegra_soc_hwpm_update_get_put)

/*
 * IOCTL for querying IP instance info
 */
#define TEGRA_CTRL_CMD_SOC_HWPM_IP_FLOORSWEEP_INFO		\
		_IOWR(TEGRA_SOC_HWPM_IOC_MAGIC,			\
			TEGRA_SOC_HWPM_IOCTL_FLOORSWEEP_INFO,	\
			struct tegra_soc_hwpm_ip_floorsweep_info)

#endif /* TEGRA_SOC_HWPM_UAPI_H */