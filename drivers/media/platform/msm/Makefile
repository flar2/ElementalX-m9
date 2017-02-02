#
# Makefile for the QCOM spcific video device drivers
# based on V4L2.
#

obj-$(CONFIG_MSM_VIDC_V4L2)     += vidc/
obj-$(CONFIG_MSM_WFD) += wfd/
obj-y += broadcast/
obj-$(CONFIG_DVB_MPQ) += dvb/
#HTC_START
ifneq ($(findstring M60,$(PRIVATE_SKU_NAME)),)
#ifeq ($(CONFIG_CAMERA_DRIVER_VER_M),y)
obj-$(CONFIG_MSMB_CAMERA) += camera_v2_m/
else
obj-$(CONFIG_MSMB_CAMERA) += camera_v2/
endif
#HTC_END
obj-$(CONFIG_MSM_VPU) += vpu/
