# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2017-2023, NVIDIA CORPORATION.  All rights reserved.

ifeq ($(origin srctree.nvidia), undefined)
srctree.nvidia := $(srctree.nvidia-oot)
endif

ccflags-y += -I$(srctree.nvidia)/include
ccflags-y += -DMODS_HAS_TEGRA

obj-m += mods.o

mods-y                             := mods_irq.o
mods-y                             += mods_krnl.o
mods-y                             += mods_mem.o

mods-$(CONFIG_ACPI)                += mods_acpi.o
mods-$(CONFIG_TEGRA_NVADSP)        += mods_adsp.o
mods-$(CONFIG_ARM_FFA_TRANSPORT)   += mods_arm_ffa.o
mods-$(CONFIG_TEGRA_IVC)           += mods_bpmpipc.o
mods-$(CONFIG_COMMON_CLK)          += mods_clock.o
mods-$(CONFIG_DEBUG_FS)            += mods_debugfs.o
mods-$(CONFIG_DMA_ENGINE)          += mods_dma.o
mods-$(CONFIG_DMA_SHARED_BUFFER)   += mods_dmabuf.o
mods-$(CONFIG_ARCH_TEGRA)          += mods_ipi.o
mods-$(CONFIG_NET)                 += mods_netdevice.o
mods-$(CONFIG_ARCH_TEGRA)          += mods_oist.o
mods-$(CONFIG_OPTEE)               += mods_optee.o
mods-$(CONFIG_PCI)                 += mods_pci.o
mods-$(CONFIG_ARCH_TEGRA_19x_SOC)  += mods_ras.o
mods-$(CONFIG_ARCH_TEGRA)          += mods_smmu_drv.o
mods-$(CONFIG_ARCH_TEGRA)          += mods_tegraprod.o
mods-$(CONFIG_TRUSTY)              += mods_tz.o

mods-objs                          := mods.dtb.o
