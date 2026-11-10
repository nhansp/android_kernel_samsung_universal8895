/* SPDX-License-Identifier: GPL-2.0 */
#ifndef FIMC_IS_INTERFACE_JOPP_H
#define FIMC_IS_INTERFACE_JOPP_H

#include <linux/compiler.h>
#include <linux/stringify.h>

/*
 * The stock DDK, RTA and VRA libraries are built for CONFIG_RKP_CFP_JOPP: before every indirect
 * call into the kernel they check that the word preceding the target is
 * CONFIG_RKP_CFP_JOPP_MAGIC and execute 0xdeadc0de (undefined) otherwise. The stock kernel put the
 * magic in front of every function (scripts/rkp_cfp/instrument.py); here each kernel function
 * handed to a library goes through a local entry that carries it. "hint 34" is BTI C.
 */
#define FIMC_IS_JOPP_MAGIC	0x00be7bad

#define FIMC_IS_JOPP_ENTRY(fn)							\
	static void *const __fimc_is_jopp_target_##fn __used = (void *)(fn);	\
	extern char fn##_jopp[];						\
	asm(".pushsection .text, \"ax\", %progbits\n"				\
	    ".balign 4\n"							\
	    ".word " __stringify(FIMC_IS_JOPP_MAGIC) "\n"			\
	    ".type " #fn "_jopp, %function\n"					\
	    #fn "_jopp:\n"							\
	    "hint 34\n"								\
	    "b " #fn "\n"							\
	    ".size " #fn "_jopp, . - " #fn "_jopp\n"				\
	    ".popsection")

#define FIMC_IS_JOPP_ADDR(fn)	((void *)fn##_jopp)
#define FIMC_IS_JOPP(fn)	((typeof(&(fn)))FIMC_IS_JOPP_ADDR(fn))

#endif
