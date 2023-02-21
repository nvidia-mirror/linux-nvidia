# SPDX-License-Identifier: GPL-2.0

ccflags-y += -I$(srctree.nvidia)/include
ccflags-y += -DMODS_HAS_TEGRA

LINUX_VERSION := $(shell expr $(VERSION) \* 256 + $(PATCHLEVEL))
LINUX_VERSION_6_2 := $(shell expr 6 \* 256 + 2)

ifeq ($(CONFIG_TEGRA_OOT_MODULE),m)
CONFIG_MODS := m
endif

obj-$(CONFIG_MODS)                 := mods.o

mods-y                             := mods_irq.o
mods-y                             += mods_krnl.o
mods-y                             += mods_mem.o

mods-$(CONFIG_ACPI)                += mods_acpi.o
mods-$(CONFIG_TEGRA_NVADSP)        += mods_adsp.o
# mods_arm_ffa and mods_bpmpipc are currently broken for Linux v6.2
# and so skip for Linux v6.2+
ifeq ($(shell test $(LINUX_VERSION) -lt $(LINUX_VERSION_6_2); echo $$?),0)
mods-$(CONFIG_ARM_FFA_TRANSPORT)   += mods_arm_ffa.o
mods-$(CONFIG_TEGRA_IVC)           += mods_bpmpipc.o
endif
mods-$(CONFIG_COMMON_CLK)          += mods_clock.o
mods-$(CONFIG_DEBUG_FS)            += mods_debugfs.o
mods-$(CONFIG_DMA_ENGINE)          += mods_dma.o
mods-$(CONFIG_ARCH_TEGRA)          += mods_ipi.o
mods-$(CONFIG_NET)                 += mods_netdevice.o
mods-$(CONFIG_ARCH_TEGRA)          += mods_oist.o
mods-$(CONFIG_OPTEE)               += mods_optee.o
mods-$(CONFIG_PCI)                 += mods_pci.o
mods-$(CONFIG_ARCH_TEGRA_19x_SOC)  += mods_ras.o
mods-$(CONFIG_ARCH_TEGRA)          += mods_smmu_drv.o
mods-$(CONFIG_TEGRA_DC)            += mods_tegradc.o
mods-$(CONFIG_TRUSTY)              += mods_tz.o

mods-objs                          := mods.dtb.o

ifneq ($(CONFIG_TEGRA_OOT_MODULE),m)
ccflags-y                          += -DMODS_HAS_DMABUF
ccflags-y                          += -DMODS_HAS_PROD
mods-$(CONFIG_DMA_SHARED_BUFFER)   += mods_dmabuf.o
mods-$(CONFIG_ARCH_TEGRA)          += mods_tegraprod.o
endif
