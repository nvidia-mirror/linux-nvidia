ifeq ($(CONFIG_TEGRA_OOT_MODULE),m)
ccflags-y += -DCONFIG_TEGRA_GR_VIRTUALIZATION -DCONFIG_TEGRA_OOT_MODULE
obj-m = tegra_gr_comm.o
else
obj-$(CONFIG_TEGRA_GR_VIRTUALIZATION) += tegra_gr_comm.o
endif
