/*
 * nvlink-mods.h:
 * This header contains the structures and variables needed for
 * the NVLINK MODs APIs exported by the Tegra NVLINK endpoint driver.
 *
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

/* TEGRA_CTRL_CMD_NVLINK_GET_NVLINK_CAPS */

#define NVLINK_VERSION_10				0x00000001
#define NVLINK_VERSION_20				0x00000002
#define NVLINK_VERSION_22				0x00000004

#define TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_INVALID	(0x00000000)
#define TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_1_0	(0x00000001)
#define TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_2_0	(0x00000002)
#define TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_2_2	(0x00000004)

#define TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_INVALID	(0x00000000)
#define TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_1_0		(0x00000001)
#define TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_2_0		(0x00000002)
#define TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_2_2		(0x00000004)

#define TEGRA_CTRL_NVLINK_CAPS_SUPPORTED		BIT(0)
#define TEGRA_CTRL_NVLINK_CAPS_P2P_SUPPORTED		BIT(1)
#define TEGRA_CTRL_NVLINK_CAPS_SYSMEM_ACCESS		BIT(2)
#define TEGRA_CTRL_NVLINK_CAPS_P2P_ATOMICS		BIT(3)
#define TEGRA_CTRL_NVLINK_CAPS_SYSMEM_ATOMICS		BIT(4)
#define TEGRA_CTRL_NVLINK_CAPS_PEX_TUNNELING		BIT(5)
#define TEGRA_CTRL_NVLINK_CAPS_SLI_BRIDGE		BIT(6)
#define TEGRA_CTRL_NVLINK_CAPS_SLI_BRIDGE_SENSABLE	BIT(7)
#define TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L0		BIT(8)
#define TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L1		BIT(9)
#define TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L2		BIT(10)
#define TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L3		BIT(11)
#define TEGRA_CTRL_NVLINK_CAPS_VALID			BIT(12)

struct nvlink_caps {
	__u16 nvlink_caps;

	__u8 lowest_nvlink_version;
	__u8 highest_nvlink_version;
	__u8 lowest_nci_version;
	__u8 highest_nci_version;

	__u32 discovered_link_mask;
	__u32 enabled_link_mask;
};

/* TEGRA_CTRL_CMD_NVLINK_GET_NVLINK_STATUS */

/* NVLink link states */
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_INIT		(0x00000000)
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_HWCFG		(0x00000001)
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_SWCFG		(0x00000002)
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_ACTIVE		(0x00000003)
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_FAULT		(0x00000004)
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_RECOVERY		(0x00000006)
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_INVALID		(0xFFFFFFFF)

/* NVLink Tx sublink states */
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_RX_STATE_HIGH_SPEED_1	(0x00000000)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_RX_STATE_SINGLE_LANE	(0x00000004)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_RX_STATE_TRAINING	(0x00000005)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_RX_STATE_SAFE_MODE	(0x00000006)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_RX_STATE_OFF		(0x00000007)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_RX_STATE_INVALID	(0x000000FF)

/* NVLink Rx sublink states */
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_TX_STATE_HIGH_SPEED_1	(0x00000000)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_TX_STATE_SINGLE_LANE	(0x00000004)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_TX_STATE_TRAINING	(0x00000005)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_TX_STATE_SAFE_MODE	(0x00000006)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_TX_STATE_OFF		(0x00000007)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_TX_STATE_INVALID	(0x000000FF)

#define TEGRA_CTRL_NVLINK_STATUS_PHY_NVHS			(0x00000001)
#define TEGRA_CTRL_NVLINK_STATUS_PHY_GRS			(0x00000002)
#define TEGRA_CTRL_NVLINK_STATUS_PHY_INVALID			(0x000000FF)

/* Version information */
#define TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_1_0		(0x00000001)
#define TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_2_0		(0x00000002)
#define TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_2_2		(0x00000004)
#define TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_INVALID	(0x000000FF)

#define TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_1_0		(0x00000001)
#define TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_2_0		(0x00000002)
#define TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_2_2		(0x00000004)
#define TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_INVALID		(0x000000FF)

#define TEGRA_CTRL_NVLINK_STATUS_NVHS_VERSION_1_0		(0x00000001)
#define TEGRA_CTRL_NVLINK_STATUS_NVHS_VERSION_INVALID		(0x000000FF)

#define TEGRA_CTRL_NVLINK_STATUS_GRS_VERSION_1_0		(0x00000001)
#define TEGRA_CTRL_NVLINK_STATUS_GRS_VERSION_INVALID		(0x000000FF)

/* Connection properties */
#define TEGRA_CTRL_NVLINK_STATUS_CONNECTED_TRUE		(0x00000001)
#define TEGRA_CTRL_NVLINK_STATUS_CONNECTED_FALSE		(0x00000000)

#define TEGRA_CTRL_NVLINK_STATUS_LOOP_PROPERTY_LOOPBACK	(0x00000001)
#define TEGRA_CTRL_NVLINK_STATUS_LOOP_PROPERTY_LOOPOUT		(0x00000002)
#define TEGRA_CTRL_NVLINK_STATUS_LOOP_PROPERTY_NONE		(0x00000000)

#define TEGRA_CTRL_NVLINK_STATUS_REMOTE_LINK_NUMBER_INVALID	(0x000000FF)

/* NVLink REFCLK types */
#define TEGRA_CTRL_NVLINK_REFCLK_TYPE_INVALID			(0x00)
#define TEGRA_CTRL_NVLINK_REFCLK_TYPE_NVHS			(0x01)
#define TEGRA_CTRL_NVLINK_REFCLK_TYPE_PEX			(0x02)

#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_ID_FLAGS_NONE	(0x00000000)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_ID_FLAGS_PCI	(0x00000001)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_ID_FLAGS_UUID	(0x00000002)

#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_EBRIDGE	(0x00000000)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_NPU		(0x00000001)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_GPU		(0x00000002)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_SWITCH	(0x00000003)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_TEGRA	(0x00000004)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_NONE		(0x000000FF)

#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_UUID_INVALID	(0xFFFFFFFF)

struct nvlink_device_info {
	/* ID Flags */
	__u32 device_id_flags;

	/* PCI Information */
	__u16 domain;
	__u16 bus;
	__u16 device;
	__u16 function;
	__u32 pci_device_id;

	/* Device Type */
	__u64 device_type;

	/* Device UUID */
	__u8 device_uuid[16];
};

struct nvlink_link_status_info {
	/* Top level capablilites */
	__u16 caps;

	__u8 phy_type;
	__u8 sublink_width;

	/* Link and sublink states */
	__u32 link_state;
	__u8 rx_sublink_status;
	__u8 tx_sublink_status;

	/* Indicates that lane reveral is in effect on this link */
	bool bLane_reversal;

	__u8 nvlink_version;
	__u8 nci_version;
	__u8 phy_version;

	/* Clock information */
	__u32 nvlink_link_clockKHz;
	__u32 nvlink_common_clock_speedKHz;
	__u32 nvlink_ref_clk_speedKHz;
	__u8 nvlink_ref_clk_type;

	__u32 nvlink_link_clockMhz;
	__u32 nvlink_common_clock_speedMhz;
	__u32 nvlink_ref_clk_speedMhz;

	/* Connection information */
	bool connected;
	__u8 loop_property;
	__u8 remote_device_link_number;
	__u8 local_device_link_number;

	struct nvlink_device_info remote_device_info;
	struct nvlink_device_info local_device_info;
};

struct nvlink_status {
	__u32 enabled_link_mask;
	struct nvlink_link_status_info link_info;
};

/* TODO: choose a unique MAGIC number for ioctl implementation */
#define TEGRA_NVLINK_IOC_MAGIC	  'T'
#define	TEGRA_CTRL_CMD_NVLINK_GET_NVLINK_CAPS		\
		_IOWR(TEGRA_NVLINK_IOC_MAGIC,  1, struct nvlink_caps)
#define TEGRA_CTRL_CMD_NVLINK_GET_NVLINK_STATUS		\
		_IOWR(TEGRA_NVLINK_IOC_MAGIC,  2, struct nvlink_status)
