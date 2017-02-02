
#
# color annotations to use instead of leading newline chars
ccflags-y += -DANSI_COLORS

ccflags-y += -DBUILD_NUM_STRING=\"xxx\"
ccflags-y += -DMHL_PRODUCT_NUM=8620
ccflags-y += -DMHL_DRIVER_NAME=\"sii8620drv\"
ccflags-y += -DMHL_DEVICE_NAME=\"sii-8620\"

# Support Device Tree?
#ccflags-y += -DSIMG_USE_DTS

# Kernel level supported
ccflags-y += -DLINUX_KERNEL_VER=310
#
# FORCE_OCBUS_FOR_ECTS is used to identify code added for ECTS temporary fix that prohibits use of eCBUS.
# in addition module parameter force_ocbus_for_ects needs to be set as 1 to achieve ECTS operation.
ccflags-y += -DFORCE_OCBUS_FOR_ECTS
#
# PC_MODE_VIDEO_TIMING_SUPPORT is for cases where no VIC is available from either AVIF or VSIF.
ccflags-y += -DPC_MODE_VIDEO_TIMING_SUPPORT
#
# MANUAL_INFO_FRAME_CLEAR_AT_HPD_DRIVEN_HIGH is to clear all infoframes 
#	upon driving HPD high instead of when SCDT goes low.
ccflags-y += -DMANUAL_INFO_FRAME_CLEAR_AT_HPD_DRIVEN_HIGH
#
# MEDIA_DATA_TUNNEL_SUPPORT
#	Default is enabled. Comment next line to disable.
ccflags-y += -DMEDIA_DATA_TUNNEL_SUPPORT
#
# Include REMOTE BUTTON PROTOCOL code or not
ccflags-y += -DINCLUDE_RBP=1

ccflags-y += -DINCLUDE_HID=0

ccflags-$(CONFIG_HTC_MHL_DETECTION_8620) += -DHTC_MHL_DETECTION_8620
ccflags-$(CONFIG_INTERNAL_CHARGING_SUPPORT_8620) += -DINTERNAL_CHARGING_SUPPORT_8620

# Example of use of SiI6031 is wrapped under the following definition
# It also illustrates how 8620 driver may be integrated into MSM platform
# the flag should be disabled if not building for MSM
ccflags-y += -DINCLUDE_SII6031=0
#
# MANUAL_EDID_FETCH uses DDC master directly, instead of h/w automated method.
ccflags-y += -DMANUAL_EDID_FETCH

# If CI2CA pin is pulled HIGH, you must define the following flag
#ccflags-y += -DALT_I2C_ADDR

# PRINT_DDC_ABORTS enables logging of all DDC_ABORTs. Default "disabled" - helps MHL2 hot plug.
# Enable only if you must for debugging. 
#ccflags-y += -DPRINT_DDC_ABORTS

# CoC_FSM_MONITORING exports CoC state machine to GPIO pins
# Enable only if you must for debugging.
#ccflags-y += -DCoC_FSM_MONITORING
#ccflags-y += -DGPIO_OSCILLOSCOPE_DEBUG

# Enable only if you must for debugging. 
#ccflags-y += -DBIST_MONITORING

#BIST_DONE_DEBUG adds register dump prior to RAP{CBUS_MODE_UP}
#ccflags-y += -DBIST_DONE_DEBUG

# For si_emsc_hid-mt.c
ccflags-y += -Idrivers/hid

# Optimzations and/or workaround
ccflags-y += -DDISABLE_SPI_DMA
ccflags-y += -DUSE_SPIOPTIMIZE
ccflags-y += -DGCS_QUIRKS_FOR_SIMG

ccflags-y += -DDEBUG

# Enable VBUS sense and related operations
#ccflags-y += -DENABLE_VBUS_SENSE

#support for DVI sources and sinks in MHL3 mode.
#ccflags-y += -DMHL3_DVI_SUPPORT

#add HDMI VSDB to upstream EDID when downstream sink is DVI
#ccflags-$(CONFIG_DEBUG_DRIVER) += -DMHL3_DVI_SUPPORT_FORCE_HDMI
#
# the next lines are optional - they enable greater verbosity in debug output
ccflags-$(CONFIG_MHL_DEBUG_DRIVER) += -DENABLE_EDID_DEBUG_PRINT
#ccflags-$(CONFIG_DEBUG_DRIVER) += -DENABLE_DUMP_INFOFRAME
#
# uncomment next line to prevent EDID parsing. Useful for debugging.
#ccflags-$(CONFIG_DEBUG_DRIVER) += -DEDID_PASSTHROUGH

obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8620) += platform.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8620) += mhl_linux_tx.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8620) += mhl_rcp_inputdev.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8620) += mhl_rbp_inputdev.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8620) += mhl_supp.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8620) += si_8620_drv.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8620) += si_mhl2_edid_3d.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8620) += si_mdt_inputdev.o
#obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8620) += si_emsc_hid.o
#obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8620) += si_emsc_hid-mt.o
