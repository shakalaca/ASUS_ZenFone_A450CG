#
# Makefile for sensor drivers
#

obj-$(CONFIG_VIDEO_IMX)        += imx/
obj-$(CONFIG_VIDEO_IMX219)     += imx219/
obj-$(CONFIG_VIDEO_IMXVB)      += imx135VB/

obj-$(CONFIG_VIDEO_MT9E013)    += mt9e013/

obj-$(CONFIG_VIDEO_MT9D113)    += mt9d113.o
CFFLAGS_mt9d113.o = -Werror

obj-$(CONFIG_VIDEO_MT9V113)    += mt9v113.o
CFFLAGS_mt9v113.o = -Werror

obj-$(CONFIG_VIDEO_OV5640)     += ov5640.o
CFFLAGS_ov5640.o = -Werror

obj-$(CONFIG_VIDEO_OV9724)     += ov9724.o
obj-$(CONFIG_VIDEO_OV2722)     += ov2722.o
obj-$(CONFIG_VIDEO_OV8830)     += ov8830.o

obj-$(CONFIG_VIDEO_S5K8AAY)    += s5k8aay.o
CFLAGS_s5k8aay.o += -Werror

obj-$(CONFIG_VIDEO_SPCA700XA)  += spca700xa.o
obj-$(CONFIG_VIDEO_MT9M114)    += mt9m114.o
obj-$(CONFIG_VIDEO_OV5693)     += ov5693.o
CFFLAGS_ov5693.o = -Werror
obj-$(CONFIG_VIDEO_AR0543)     += ar0543.o
CFFLAGS_ar0543.o = -Werror
obj-$(CONFIG_VIDEO_AR0543_RAW)     += ar0543_raw.o
CFFLAGS_ar0543_raw.o = -Werror
obj-$(CONFIG_VIDEO_IMX111_RAW)     += imx111_raw.o
CFFLAGS_imx111_raw.o = -Werror
obj-$(CONFIG_VIDEO_MT9M114_RAW)     += mt9m114_raw.o
CFFLAGS_mt9m114_raw.o = -Werror
obj-$(CONFIG_VIDEO_GC0339_RAW) += gc0339_raw.o
CFFLAGS_gc0339_raw.o = -Werror
obj-$(CONFIG_VIDEO_HM2056_RAW) += hm2056_raw.o
CFFLAGS_hm2056_raw.o = -Werror


obj-$(CONFIG_VIDEO_MSRLIST_HELPER) += libmsrlisthelper.o
CFLAGS_libmsrlisthelper.o += -Werror

#
# Makefile for flash drivers
#

obj-$(CONFIG_VIDEO_LM3554) += lm3554.o
obj-$(CONFIG_VIDEO_LM3559) += lm3559.o


