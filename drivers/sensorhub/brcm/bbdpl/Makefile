# L6C 5.2: stock S:drivers/sensorhub/brcm/bbdpl/Makefile:3, :5 (the TTY/I2C/hostwake
# transports are not built on this board)
# L6C 5.0 G-23: stylistic stock constructs kept verbatim (non-static cross-file functions without a
# header prototype, ...) are silenced once per transplant directory
ccflags-y += -Wno-duplicate-decl-specifier -Wno-enum-conversion -Wno-missing-prototypes \
	     -Wno-missing-declarations -Wno-logical-not-parentheses

obj-$(CONFIG_BCM_GPS_SPI_DRIVER) += bcm_gps_spi.o
obj-$(CONFIG_SENSORS_SSP_BBD) += bbd_rpc_lh.o bbd.o
