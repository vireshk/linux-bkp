# SPDX-License-Identifier: GPL-2.0-only
ffa-bus-y = bus.o
ffa-driver-y = driver.o
ffa-transport-$(CONFIG_ARM_FFA_SMCCC) += smccc.o
ffa-core-objs := $(ffa-bus-y)
ffa-module-objs := $(ffa-driver-y) $(ffa-transport-y)
obj-$(CONFIG_ARM_FFA_TRANSPORT)  = ffa-core.o
obj-$(CONFIG_ARM_FFA_TRANSPORT) += ffa-module.o
ffa-acs-y += acs.o
ffa-acs-objs := $(ffa-acs-y)
obj-$(CONFIG_ARM_FFA_ACS_USER) += ffa-acs.o
