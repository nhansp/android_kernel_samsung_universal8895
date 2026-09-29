#
# Makefile for Secure OS booster API
#
obj-$(CONFIG_SECURE_OS_BOOSTER_API) += sec_os_booster.o

# Includes MobiCore kernel driver ($(srctree): the O= build, L6T 5.8 item 6;
# secos_booster.h comes from <soc/samsung/secos_booster.h>)
ccflags-y += -I$(srctree)/drivers/gud/gud-exynos8895_kinibi400/MobiCoreDriver
ccflags-y += -I$(srctree)/drivers/gud/gud-exynos8895_kinibi400/MobiCoreDriver/public
