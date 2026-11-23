# Makefile of modem_v1

ccflags-y += -I$(src)
# Style warnings in the vendor code, fatal with CONFIG_WERROR
ccflags-y += -Wno-duplicate-decl-specifier -Wno-enum-conversion -Wno-missing-prototypes \
	     -Wno-missing-declarations -Wno-logical-not-parentheses

obj-y += modem_main.o modem_variation.o modem_io_device.o modem_utils.o modem_dump.o

obj-$(CONFIG_LINK_DEVICE_SHMEM) += link_device_shmem.o \
				   link_device_memory_snapshot.o \
				   link_device_memory_flow_control.o \
				   link_device_memory_debug.o \
				   link_device_memory_sbd.o \
				   modem_notifier.o

obj-$(CONFIG_UMTS_MODEM_SS310AP) += modem_ctrl_ss310ap.o
