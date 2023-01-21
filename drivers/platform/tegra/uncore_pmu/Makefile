# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
# Uncore PMU driver code.

GCOV_PROFILE := y

ccflags-y += -I$(srctree)/drivers/platform/tegra/include \
							-I$(srctree.nvidia)/drivers/platform/tegra/

ccflags-y += -Werror

obj-$(CONFIG_ARCH_TEGRA_23x_SOC) += tegra23x_perf_uncore.o
obj-$(CONFIG_ARCH_TEGRA_23x_SOC) += arm_gicv3_pmu.o
