# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_VIRTIO) += virtio.o virtio_ring.o
obj-$(CONFIG_VIRTIO_ANCHOR) += virtio_anchor.o
obj-$(CONFIG_VIRTIO_PCI_LIB) += virtio_pci_modern_dev.o
obj-$(CONFIG_VIRTIO_PCI_LIB_LEGACY) += virtio_pci_legacy_dev.o
obj-$(CONFIG_VIRTIO_MMIO) += virtio_mmio.o
obj-$(CONFIG_VIRTIO_MSG_TRANSPORT) += virtio_msg_transport.o
obj-$(CONFIG_VIRTIO_MSG_BUS_QUEUE_HELPER) += virtio_msg_bus_queue_helper.o
obj-$(CONFIG_VIRTIO_MSG_BUS_DMA_DRIVER_HELPER) += virtio_msg_bus_dma_driver_helper.o
obj-$(CONFIG_VIRTIO_MSG_BRIDGE) += virtio_msg_bus_bridge_device.o
obj-$(CONFIG_VIRTIO_MSG_BUS_DMA_DEVICE_HELPER) += virtio_msg_bus_dma_device_helper.o
obj-$(CONFIG_VIRTIO_MSG_LOOPBACK) += virtio_msg_loopback.o
virtio_msg_loopback-y := virtio_msg_loopback_transport.o \
			 virtio_msg_loopback_bridge.o
virtio_msg_bus_ffa_common-y := virtio_msg_bus_ffa_common.o
virtio_msg_bus_ffa_common-$(CONFIG_VIRTIO_MSG_FFA_XFER_INDIRECT) += \
			 virtio_msg_bus_ffa_xfer_indirect.o
virtio_msg_bus_ffa_common-$(CONFIG_VIRTIO_MSG_FFA_XFER_FIFO) += \
			 virtio_msg_bus_ffa_xfer_fifo.o
obj-$(CONFIG_VIRTIO_MSG_FFA_COMMON) += virtio_msg_bus_ffa_common_module.o
virtio_msg_bus_ffa_common_module-y := $(virtio_msg_bus_ffa_common-y)

virtio_msg_bus_ffa_driver-y := virtio_msg_bus_ffa_driver.o \
			 virtio_msg_bus_ffa_driver_area.o \
			 virtio_msg_bus_ffa_driver_topology.o
virtio_msg_bus_ffa_driver-$(CONFIG_VIRTIO_MSG_FFA_XFER_INDIRECT) += \
			 virtio_msg_bus_ffa_xfer_indirect_driver.o
virtio_msg_bus_ffa_driver-$(CONFIG_VIRTIO_MSG_FFA_XFER_FIFO) += \
			 virtio_msg_bus_ffa_xfer_fifo_driver.o
obj-$(CONFIG_VIRTIO_MSG_FFA_DRIVER_BINDING) += virtio_msg_bus_ffa_driver_module.o
virtio_msg_bus_ffa_driver_module-y := $(virtio_msg_bus_ffa_driver-y)

obj-$(CONFIG_VIRTIO_PCI) += virtio_pci.o
virtio_pci-y := virtio_pci_modern.o virtio_pci_common.o
virtio_pci-$(CONFIG_VIRTIO_PCI_LEGACY) += virtio_pci_legacy.o
virtio_pci-$(CONFIG_VIRTIO_PCI_ADMIN_LEGACY) += virtio_pci_admin_legacy_io.o
obj-$(CONFIG_VIRTIO_BALLOON) += virtio_balloon.o
obj-$(CONFIG_VIRTIO_INPUT) += virtio_input.o
obj-$(CONFIG_VIRTIO_VDPA) += virtio_vdpa.o
obj-$(CONFIG_VIRTIO_MEM) += virtio_mem.o
obj-$(CONFIG_VIRTIO_DMA_SHARED_BUFFER) += virtio_dma_buf.o
obj-$(CONFIG_VIRTIO_DEBUG) += virtio_debug.o
obj-$(CONFIG_VIRTIO_RTC) += virtio_rtc.o
virtio_rtc-y := virtio_rtc_driver.o
virtio_rtc-$(CONFIG_VIRTIO_RTC_PTP) += virtio_rtc_ptp.o
virtio_rtc-$(CONFIG_VIRTIO_RTC_ARM) += virtio_rtc_arm.o
virtio_rtc-$(CONFIG_VIRTIO_RTC_CLASS) += virtio_rtc_class.o
