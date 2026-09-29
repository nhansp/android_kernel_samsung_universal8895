#
# Makefile for the optics drivers.
#
# L6C 5.10: stock S:drivers/optics/Makefile:12-14 for the two objects this board builds

# L6C 5.0 G-23: stylistic stock constructs kept verbatim (non-static cross-file functions without a
# header prototype, ...) are silenced once per transplant directory
ccflags-y += -Wno-duplicate-decl-specifier -Wno-enum-conversion -Wno-missing-prototypes \
	     -Wno-missing-declarations -Wno-logical-not-parentheses

obj-$(CONFIG_SENSORS_HRM_MAX86902)	+= hrm_max86902.o
obj-$(CONFIG_SENSORS_HRMSENSOR)		+= hrmsensor.o
