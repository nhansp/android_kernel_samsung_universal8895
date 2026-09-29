/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Samsung sec_key interface.
 * Spec: L6F 5.14 (lines 2490-2498).
 */

#ifndef _LINUX_INPUT_SEC_KEY_H
#define _LINUX_INPUT_SEC_KEY_H

#include <linux/types.h>

#if IS_ENABLED(CONFIG_INPUT_SAMSUNG_SEC_KEY)
bool sec_key_hall_closed(void);	/* true = flip cover closed (SW_LID 1) = hall_detect "CLOSE" */
#else
static inline bool sec_key_hall_closed(void) { return false; }	/* = "OPEN", stock's state before any event */
#endif

#endif /* _LINUX_INPUT_SEC_KEY_H */
