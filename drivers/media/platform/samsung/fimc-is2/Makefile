fimc-is-objs	:= fimc-is-fpsimd.o \
		fimc-is-core.o \
		fimc-is-spi.o \
		fimc-is-i2c.o \
		fimc-is-debug.o \
		fimc-is-mem.o \
		fimc-is-pipe.o \
		fimc-is-framemgr.o \
		fimc-is-groupmgr.o \
		fimc-is-resourcemgr.o \
		fimc-is-devicemgr.o \
		fimc-is-video.o \
		fimc-is-video-sensor.o \
		fimc-is-video-3aa.o \
		fimc-is-video-3aac.o \
		fimc-is-video-3aap.o \
		fimc-is-video-isp.o \
		fimc-is-video-ispc.o \
		fimc-is-video-ispp.o \
		fimc-is-video-dis.o \
		fimc-is-video-disc.o \
		fimc-is-video-scc.o \
		fimc-is-video-scp.o \
		fimc-is-video-mcs.o \
		fimc-is-video-mcsp.o \
		fimc-is-video-vra.o \
		fimc-is-video-ssvc0.o \
		fimc-is-video-ssvc1.o \
		fimc-is-video-ssvc2.o \
		fimc-is-video-ssvc3.o \
		fimc-is-subdev-ctrl.o \
		fimc-is-device-flite.o \
		fimc-is-device-ischain.o \
		fimc-is-time.o \
		fimc-is-dvfs.o \
		fimc-is-dt.o \
		fimc-is-clk-gate.o \
		fimc-is-binary.o

obj-$(CONFIG_COMPANION_USE) += fimc-is-device-preprocessor.o \
			       fimc-is-video-preprocessor.o

ifeq ($(CONFIG_USE_DIRECT_IS_CONTROL),y)
fimc-is-objs += fimc-is-interface-wrap.o
else
fimc-is-objs += fimc-is-interface-wrap-fw.o
fimc-is-objs += fimc-is-interface.o
endif

ifeq ($(CONFIG_USE_SENSOR_GROUP),y)
fimc-is-objs += fimc-is-device-sensor_v2.o
else
fimc-is-objs += fimc-is-device-sensor.o
endif

obj-$(CONFIG_EXYNOS_DEVICE_MIPI_CSIS) += fimc-is-device-csi.o
obj-$(CONFIG_EXYNOS_DEVICE_MIPI_CSIS_VER2) += fimc-is-device-csi_v2.o
obj-$(CONFIG_EXYNOS_DEVICE_MIPI_CSIS_VER3) += fimc-is-device-csi_v3.o

obj-$(CONFIG_VIDEO_EXYNOS_FIMC_IS2) += fimc-is.o
obj-$(CONFIG_VIDEO_EXYNOS_FIMC_IS2) += sensor/
obj-$(CONFIG_VIDEO_EXYNOS_FIMC_IS2) += ischain/
obj-$(CONFIG_VIDEO_EXYNOS_FIMC_IS2) += vendor/
obj-$(CONFIG_VIDEO_EXYNOS_FIMC_IS2) += hardware/
obj-$(CONFIG_VIDEO_EXYNOS_FIMC_IS2) += interface/
obj-$(CONFIG_USE_DIRECT_IS_CONTROL) += hardware/
obj-$(CONFIG_USE_DIRECT_IS_CONTROL) += interface/

subdir-ccflags-y := -I$(srctree)/drivers/media/platform/samsung/fimc-is2
subdir-ccflags-y += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/include
subdir-ccflags-y += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/interface
subdir-ccflags-y += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/hardware
subdir-ccflags-y += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/hardware/api
subdir-ccflags-y += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/vendor
subdir-ccflags-y += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/sensor/csi
subdir-ccflags-y += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/sensor/module_framework
subdir-ccflags-y += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/vendor/mcd/preprocessor
subdir-ccflags-$(CONFIG_FIMC_IS_V3_1_1) += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/ischain/fimc-is-v3_1_1
subdir-ccflags-$(CONFIG_FIMC_IS_V3_11_0) += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/ischain/fimc-is-v3_11_0
subdir-ccflags-$(CONFIG_FIMC_IS_V3_20_0) += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/ischain/fimc-is-v3_20_0
subdir-ccflags-$(CONFIG_FIMC_IS_V4_0_0) += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/ischain/fimc-is-v4_0_0
subdir-ccflags-$(CONFIG_FIMC_IS_V4_3_0) += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/ischain/fimc-is-v4_3_0
subdir-ccflags-$(CONFIG_FIMC_IS_V5_10_0) += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/ischain/fimc-is-v5_10_0
subdir-ccflags-$(CONFIG_FIMC_IS_V5_2_0) += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/ischain/fimc-is-v5_2_0

# The transplant keeps stock's #if CONFIG_SOC_EXYNOS8895 blocks (stock
# CONFIG_SOC_EXYNOS8895=y, IKC:339); 6.18 has no such SoC symbol, so v5.2.0
# (the 8895 chain, ischain/fimc-is-v5_2_0/Kconfig) defines it for this tree.
subdir-ccflags-$(CONFIG_FIMC_IS_V5_2_0) += -DCONFIG_SOC_EXYNOS8895=1
# Stock built with -DANDROID_VERSION=$(PLATFORM_VERSION_NUMBER) (S:Makefile:483-488);
# the Pie build: scripts/android-version.sh "9" = 90000 (vendor-config_great.h:82).
subdir-ccflags-y += -DANDROID_VERSION=90000
# Stock user build: CONFIG_SAMSUNG_PRODUCT_SHIP=y (IKC:575): the libraries are not
# read from /data/media/0/ first (include/fimc-is-binary.h:33-36, spec L6B 5.6.8)
subdir-ccflags-y += -DCONFIG_SAMSUNG_PRODUCT_SHIP=1
# 4.4 API names of the stock sources (include/fimc-is-compat.h)
subdir-ccflags-y += -include $(srctree)/drivers/media/platform/samsung/fimc-is2/include/fimc-is-compat.h

ifeq ($(CONFIG_VENDER_MCD),y)
subdir-ccflags-y += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/vendor/mcd
else
subdir-ccflags-y += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/vendor/default
endif
