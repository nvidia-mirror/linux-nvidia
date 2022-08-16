#
# Tegra SOC HWPM
#

# SPDX-License-Identifier: GPL-2.0
GCOV_PROFILE := y

ccflags-y += -I$(srctree.nvidia)/include
ccflags-y += -I$(srctree.nvidia)/drivers/platform/tegra/hwpm/include
ccflags-y += -I$(srctree.nvidia)/drivers/platform/tegra/hwpm

# For OOT builds, set required config flags
ifeq ($(CONFIG_TEGRA_OOT_MODULE),m)
CONFIG_TEGRA_T234_HWPM = y
ccflags-y += -DCONFIG_TEGRA_T234_HWPM
NVHWPM_OBJ = m
else
NVHWPM_OBJ = y
endif

# Add required objects to nvhwpm object variable
# Include common files
include $(srctree.nvidia)/drivers/platform/tegra/hwpm/Makefile.common.sources
nvhwpm-objs += ${nvhwpm-common-objs}

# Include linux files
include $(srctree.nvidia)/drivers/platform/tegra/hwpm/Makefile.linux.sources
nvhwpm-objs += ${nvhwpm-linux-objs}

ifeq ($(CONFIG_TEGRA_T234_HWPM),y)
# Include T234 files
include $(srctree.nvidia)/drivers/platform/tegra/hwpm/Makefile.t234.sources
nvhwpm-objs += ${nvhwpm-t234-objs}
endif

obj-${NVHWPM_OBJ} += nvhwpm.o
