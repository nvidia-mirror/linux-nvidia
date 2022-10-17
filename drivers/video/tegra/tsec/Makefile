#
# Tsec Driver code.
#

GCOV_PROFILE := y

# Set config to build as module for OOT build
ifeq ($(CONFIG_TEGRA_OOT_MODULE),m)
CONFIG_TEGRA_TSEC := m
endif

obj-$(CONFIG_TEGRA_TSEC) += tsecriscv.o
tsecriscv-y := tsec_comms/tsec_comms.o tsec_boot.o tsec.o
