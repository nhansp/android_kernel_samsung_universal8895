# SPDX-License-Identifier: GPL-2.0

obj-$(CONFIG_EXYNOS_ASV_ARM)	+= exynos5422-asv.o
obj-$(CONFIG_EXYNOS_CHIPID)	+= exynos_chipid.o
exynos_chipid-y			+= exynos-chipid.o exynos-asv.o

obj-$(CONFIG_EXYNOS_USI)	+= exynos-usi.o

obj-$(CONFIG_EXYNOS_PMU)	+= exynos-pmu.o exynos8895-pmu.o

obj-$(CONFIG_EXYNOS_PMU_ARM_DRIVERS)	+= exynos3250-pmu.o exynos4-pmu.o \
					exynos5250-pmu.o exynos5420-pmu.o
obj-$(CONFIG_EXYNOS_REGULATOR_COUPLER) += exynos-regulator-coupler.o

obj-$(CONFIG_SAMSUNG_PM_CHECK)	+= s3c-pm-check.o

obj-$(CONFIG_GREATLTE_HB) += greatlte-hb.o
obj-$(CONFIG_GREATLTE_HS) += greatlte-hs.o
obj-$(CONFIG_GREATLTE_L1_SELFTEST) += greatlte-l1-selftest.o
obj-$(CONFIG_GREATLTE_HB) += greatlte-hb-l2.o
obj-$(CONFIG_GREATLTE_USB_DIAG) += greatlte-hb-l3.o
obj-$(CONFIG_GREATLTE_HB) += greatlte-hb-l4.o
obj-$(CONFIG_GREATLTE_HB_L5) += greatlte-hb-l5.o
obj-$(CONFIG_SAMSUNG_SEC_CLASS)		+= samsung-sec-class.o
obj-$(CONFIG_GREATLTE_HB_L6F)		+= greatlte-hb-l6f.o
# L6T (Trustonic TEE)
obj-$(CONFIG_EXYNOS8895_SMC)		+= exynos8895-smc.o
obj-$(CONFIG_EXYNOS_UFS_SRPMB)		+= exynos-srpmb.o
obj-$(CONFIG_EXYNOS_SECURE_LOG)		+= exynos-seclog.o
obj-$(CONFIG_GREATLTE_HB_L6T)		+= greatlte-hb-l6t.o
# L6A slice ion
obj-$(CONFIG_GREATLTE_HB_L6A_ION)	+= greatlte-hb-l6a-ion.o

# L6A S3CFB front-end
obj-$(CONFIG_GREATLTE_HB_L6A_S3CFB)	+= greatlte-hb-l6a-s3cfb.o

# L6A slice gpu
obj-$(CONFIG_GREATLTE_HB_L6A_GPU)	+= greatlte-hb-l6a-gpu.o

# L6D PCIe RC0 heartbeat
obj-$(CONFIG_GREATLTE_HB_L6D_PCIE)	+= greatlte-hb-l6d-pcie.o
# L6D Wi-Fi heartbeat
obj-$(CONFIG_GREATLTE_HB_L6D_WIFI)	+= greatlte-hb-l6d-wifi.o

# L6D slice bt
obj-$(CONFIG_GREATLTE_HB_L6D_BT)	+= greatlte-hb-l6d-bt.o

# L7 (power management)
obj-$(CONFIG_EXYNOS8895_PM)		+= exynos8895-pm.o
# L7 heartbeat printers
obj-$(CONFIG_GREATLTE_HB_L7)		+= greatlte-hb-l7.o
