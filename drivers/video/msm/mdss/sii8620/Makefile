#
# Makefile for the Silicon Image 8620 MHL TX device driver
#

#
# color annotations to use instead of leading newline chars
ccflags-y += -DANSI_COLORS

ccflags-y += -DBUILD_NUM_STRING=\"$(BUILD_NUM)\"
ccflags-y += -DMHL_PRODUCT_NUM=8620
ccflags-y += -DMHL_DRIVER_NAME=\"sii8620drv\"
ccflags-y += -DMHL_DEVICE_NAME=\"sii-8620\"
#ccflags-y += -DINPUT_DEV_ACCESSORY
#
# RCP_INPUTDEV_SUPPORT supports android key mapping for incoming RCP keys.
# Disable this support if key mapping is not required.
ccflags-y += -DRCP_INPUTDEV_SUPPORT
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
# MEDIA_DATA_TUNNEL_SUPPORT is for MDT support in chip independent code
ccflags-y += -DMEDIA_DATA_TUNNEL_SUPPORT
#
# EARLY_HSIC enables hsic_init to configure the transmitter for USB host mode
#ccflags-y += -DEARLY_HSIC
#
# MANUAL_EDID_FETCH uses DDC master directly, instead of h/w automated method.
ccflags-y += -DMANUAL_EDID_FETCH

# PRINT_ALL_INTR enables additional verbosity in the main driver level interrupt handler.
#ccflags-y += -DPRINT_ALL_INTR

# PRINT_DDC_ABORTS enables logging of all DDC_ABORTs. Default "disabled" - helps MHL2 hot plug.
# Enable only if you must for debugging.
#ccflags-y += -DPRINT_DDC_ABORTS

ccflags-$(CONFIG_DEBUG_DRIVER) += -DDEBUG

# Enable VBUS sense and related operations
ccflags-$(CONFIG_DEBUG_DRIVER) += -DENABLE_VBUS_SENSE

#
# the next three lines are optional - they enable greater verbosity in debug output
ccflags-$(CONFIG_DEBUG_DRIVER)  += -DENABLE_EDID_INFO_PRINT
#ccflags-$(CONFIG_DEBUG_DRIVER) += -DENABLE_EDID_DEBUG_PRINT
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
obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8620) += si_emsc.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8620) += si_emsc_hid.o
