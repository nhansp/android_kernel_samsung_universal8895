# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_B43)		+= b43/
obj-$(CONFIG_B43LEGACY)		+= b43legacy/

obj-$(CONFIG_BRCMFMAC)	+= brcm80211/
obj-$(CONFIG_BRCMSMAC)	+= brcm80211/

# L6D K-F4z: bcmdhd4361 Wi-Fi (L6D 6.2, stock S:drivers/net/wireless/Makefile:59-61)
obj-$(CONFIG_BCM4361)			+= bcmdhd4361/
obj-$(CONFIG_BCMDHD_EXYNOS8895)	+= bcmdhd4361/dhd_custom_memprealloc.o bcmdhd4361/dhd_custom_exynos8895.o
obj-$(CONFIG_BCMDHD_EXYNOS8895_GLUE_ONLY) += bcmdhd4361/dhd_custom_exynos8895.o	# (final, R3-03) T-F1 only
