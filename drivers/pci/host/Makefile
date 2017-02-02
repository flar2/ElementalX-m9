obj-$(CONFIG_PCI_MSM) += pci-msm.o

# HTC_WIFI_START
# [HPKB#28650] Reduce log size on non-debug ROM
ifdef HTC_CFLAG_DEBUG
ccflags-y += -DHTC_DEBUG_FLAG
endif
# HTC_WIFI_END
