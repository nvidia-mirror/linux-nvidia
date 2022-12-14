#
# Platform Security Controller Driver code
#

GCOV_PROFILE := y

ifeq ($(CONFIG_TEGRA_OOT_MODULE),m)
CONFIG_TEGRA_PSC := m
endif

ccflags-y += -I$(srctree.nvidia)/drivers/platform/tegra/include

ccflags-y += -Werror

obj-$(CONFIG_TEGRA_PSC) += tegra23x_psc.o
tegra23x_psc-objs := tegra23x_psc_mailbox.o

tegra23x_psc-$(CONFIG_DEBUG_FS) += psc_debug.o
