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

# DEBUG: CoreSight PC sampling at panic
obj-$(CONFIG_GREATLTE_CSPC)		+= greatlte-cspc.o
# L6F slice fp (WP-6F)
obj-$(CONFIG_GREATLTE_HB_L6F_FP)	+= greatlte-hb-l6f-fp.o
# greatlte WP-6C: CP power control (L6C 5.1.4; stock S:drivers/soc/samsung/Makefile:40)
obj-$(CONFIG_SEC_SIPC_MODEM_IF)	+= exynos8895-pmu-cp.o

# L6C modem + sensor hub heartbeat
obj-$(CONFIG_GREATLTE_HB_L6C)	+= greatlte-hb-l6c.o
# L6M SMFC/MSCL heartbeat (WP l6m, DEBUG)
obj-$(CONFIG_GREATLTE_HB_L6M)	+= greatlte-hb-l6m.o# L7 (power management)
obj-$(CONFIG_EXYNOS8895_PM)		+= exynos8895-pm.o
# L7 heartbeat printers
obj-$(CONFIG_GREATLTE_HB_L7)		+= greatlte-hb-l7.o
# L7 s2idle SICD heartbeat rows (WP l7sicd, DEBUG); same bool as the L7 printers
obj-$(CONFIG_GREATLTE_HB_L7)		+= greatlte-hb-l7sicd.o
# L2b bus DVFS heartbeat
obj-$(CONFIG_GREATLTE_HB_L2B)		+= greatlte-hb-l2b.o
# L2b: Exynos8895 bus traffic shaper
obj-$(CONFIG_EXYNOS8895_BTS_BW)		+= bts/
# greatlte WP-L4b: FMP/TRNG heartbeat rows and FMP known-answer test (DEBUG, spec L4b 5.5-5.6)
obj-$(CONFIG_GREATLTE_HB_L4B)		+= greatlte-hb-l4b.o
# L6K DRAM temperature trip (WP l6ki)
obj-$(CONFIG_EXYNOS8895_MCINFO)	+= exynos8895-mcinfo.o
# L6K heartbeat rows (WP l6ki, DEBUG)
obj-$(CONFIG_GREATLTE_HB_L6K)	+= greatlte-hb-l6k.o
# L6I heartbeat rows (WP l6ki, DEBUG)
obj-$(CONFIG_GREATLTE_HB_L6I)	+= greatlte-hb-l6i.o
# L6E audio heartbeat
obj-$(CONFIG_GREATLTE_HB_L6E)	+= greatlte-hb-l6e.o
# L6L (G2D blitter) heartbeat rows (WP l6l, DEBUG)
obj-$(CONFIG_GREATLTE_HB_L6L)	+= greatlte-hb-l6l.o
# L6G Type-C heartbeat rows (WP l6gtc, DEBUG)
obj-$(CONFIG_GREATLTE_HB_L6G)	+= greatlte-hb-l6g.o
# L6B multimedia heartbeat (l6b2)
obj-$(CONFIG_GREATLTE_HB_L6B)		+= greatlte-hb-l6b.o

# L6B multimedia QoS shim
obj-$(CONFIG_EXYNOS8895_MM_QOS)		+= exynos8895-mm-qos.o
