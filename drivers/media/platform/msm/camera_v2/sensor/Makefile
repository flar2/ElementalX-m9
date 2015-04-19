ccflags-y += -Idrivers/media/platform/msm/camera_v2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/msm_vb2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/camera
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/io
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/cci
obj-$(CONFIG_MSMB_CAMERA) += cci/ io/ csiphy/ csid/ actuator/ flash/ eeprom/ ois/
obj-$(CONFIG_MSM_CAMERA_SENSOR) += msm_sensor_init.o msm_sensor_driver.o msm_sensor.o
obj-$(CONFIG_MT9M114) += mt9m114.o
obj-$(CONFIG_OV5645) += ov5645.o
obj-$(CONFIG_T4KA7) += t4ka7.o
obj-$(CONFIG_IMX220) += imx220.o
obj-$(CONFIG_VD6869) += vd6869.o
obj-$(CONFIG_OV13850_FRONT) += ov13850_front.o
obj-$(CONFIG_OV4688_FRONT) += ov4688_front.o
#obj-$(CONFIG_OV2722) += ov2722.o
obj-$(CONFIG_OV2722) += ov2722-subcam.o
#HTC_CAMERA+
#obj-$(CONFIG_ARCH_MSM8994) += htc_awb_cal.o
obj-y += htc_awb_cal.o
#HTC_CAMERA-
