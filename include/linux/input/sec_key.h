/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_INPUT_SEC_KEY_H
#define _LINUX_INPUT_SEC_KEY_H

#include <linux/types.h>

#if IS_ENABLED(CONFIG_INPUT_SAMSUNG_SEC_KEY)
/* true when the flip cover is closed (SW_LID 1) */
bool sec_key_hall_closed(void);
#else
static inline bool sec_key_hall_closed(void) { return false; }
#endif

#endif /* _LINUX_INPUT_SEC_KEY_H */
