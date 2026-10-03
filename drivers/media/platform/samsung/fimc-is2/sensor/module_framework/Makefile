obj-$(CONFIG_USE_DIRECT_IS_CONTROL)	+= fimc-is-interface-sensor.o \
				       fimc-is-interface-actuator.o \
				       fimc-is-control-sensor.o \
				       fimc-is-control-actuator.o \
				       fimc-is-device-sensor-peri.o

EXTRA_CFLAGS += -I$(srctree)/drivers/media/platform/samsung/fimc-is2
EXTRA_CFLAGS += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/include
ifeq ($(CONFIG_COMPANION_DIRECT_USE),y)
EXTRA_CFLAGS += -I$(srctree)/drivers/media/platform/samsung/fimc-is2/vendor/mcd/preprocessor
endif
