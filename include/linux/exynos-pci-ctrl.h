/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2015 Samsung Electronics Co.Ltd
 * http://www.samsung.com
 *
 * Exynos PCIe link power control
 */

#ifndef __EXYNOS_PCI_CTRL_H
#define __EXYNOS_PCI_CTRL_H

/* Clients that may hold L1 substates off, one bit each */
#define PCIE_L1SS_CTRL_ARGOS		(0x1 << 0)
#define PCIE_L1SS_CTRL_BOOT		(0x1 << 1)
#define PCIE_L1SS_CTRL_CAMERA		(0x1 << 2)
#define PCIE_L1SS_CTRL_MODEM_IF		(0x1 << 3)
#define PCIE_L1SS_CTRL_SYSFS		(0x1 << 5)
#define PCIE_L1SS_CTRL_WIFI		(0x1 << 7)

/* No inline stubs: the modem driver defines a weak exynos_pcie_l1ss_ctrl() */
int exynos_pcie_l1ss_ctrl(int enable, int id);
int exynos_pcie_pm_resume(int ch_num);
void exynos_pcie_pm_suspend(int ch_num);
int exynos_pcie_poweron(int ch_num);
void exynos_pcie_poweroff(int ch_num);
void exynos_pcie_register_dump(int ch_num);
int exynos_pcie_dump_link_down_status(int ch_num);

#endif /* __EXYNOS_PCI_CTRL_H */
