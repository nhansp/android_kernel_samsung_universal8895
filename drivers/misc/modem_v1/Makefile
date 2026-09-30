# Makefile of modem_v1
# L6C 5.1.0: the objects stock built for this board (S:drivers/misc/modem_v1/Makefile:6, :13-18,
# :20 with IKC:1333, :1351); CONFIG_OPTION_REGION is a Kconfig string (stock -D, :3-4).
# The G-23 flags silence stylistic stock constructs kept verbatim (L6C 5.0 G-23).

ccflags-y += -I$(src)
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
