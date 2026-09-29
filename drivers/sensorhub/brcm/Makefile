#
# Makefile for the sensor drivers.
#
# L6C 5.3: the objects stock built for this board (S:drivers/sensorhub/brcm/Makefile with
# IKC:4748-4824); ssp_iio_trigger.o (never called) and ssp_motorcallback.o (D-14) are not
# built, ssp_iio.o is the Pie front end stock chose for ANDROID_MAJOR_VERSION != n (:56-60).

# L6C 5.0 G-23: stylistic stock constructs kept verbatim (non-static cross-file functions without a
# header prototype, ...) are silenced once per transplant directory
ccflags-y += -Wno-duplicate-decl-specifier -Wno-enum-conversion -Wno-missing-prototypes \
	     -Wno-missing-declarations -Wno-logical-not-parentheses

# stock -DANDROID_VERSION=$(PLATFORM_VERSION_NUMBER) of the Pie build (S:Makefile:486-489)
ccflags-y += -DANDROID_VERSION=90000

obj-$(CONFIG_SENSORS_SSP)		+= ssp_dev.o ssp_i2c.o ssp_data.o ssp_sysfs.o\
						ssp_firmware.o ssp_debug.o ssp_dump.o ssp_iio.o
obj-$(CONFIG_SENSORS_SYSFS)		+= sensors_core.o

# stock obj-$(CONFIG_IIO): the directory is also entered for the grip driver without the hub
obj-$(CONFIG_SENSORS_SSP)		+= ssp_iio_ring.o

obj-$(CONFIG_SENSORS_SSP_LSM6DSL)	+= factory/accel_lsm6dsl.o factory/gyro_lsm6dsl.o
obj-$(CONFIG_SENSORS_SSP_MAGNETIC_COMMON)	+= factory/magnetic_common.o
obj-$(CONFIG_SENSORS_SSP_TMD4906)	+= factory/light_tmg399x.o
obj-$(CONFIG_SENSORS_SSP_PROX_AUTOCAL_AMS)	+= factory/prox_autocal_ams.o
obj-$(CONFIG_SENSORS_SSP_LIGHT_COLORID) += factory/light_colorid.o
obj-$(CONFIG_SENSORS_SSP_LPS22H)	+= factory/pressure_lps22h.o
obj-$(CONFIG_SENSORS_SSP_MOBEAM)	 += factory/barcode_emul_tmg3992.o

obj-$(CONFIG_SENSORS_SSP_SENSORHUB)	+= ssp_sensorhub.o ssp_misc.o
obj-$(CONFIG_SENSORS_SX9320)		+= sx9320.o

obj-$(CONFIG_SENSORS_SSP_BBD)           += ssp_bbd.o factory/mcu_bcm4774.o
obj-$(CONFIG_SENSORS_SSP_BBD)           += bbdpl/
