#
# Makefile for the Kinibi core driver
#

# add our modules to kernel.
obj-$(CONFIG_TRUSTONIC_TEE) += mcDrvModule.o

mcDrvModule-y := \
	admin.o \
	client.o \
	clientlib.o \
	fastcall.o \
	iwp.o \
	logging.o \
	main.o \
	mcp.o \
	mmu.o \
	nq.o \
	scheduler.o \
	session.o \
	teeclientapi.o \
	user.o

# Release mode by default
ccflags-y += -DNDEBUG
ccflags-y += -Wno-declaration-after-statement

ccflags-$(CONFIG_TRUSTONIC_TEE_DEBUG) += -DDEBUG

# MobiCore Driver includes ($(srctree): the O= build, L6T 5.0)
ccflags-y += -I$(srctree)/drivers/gud/gud-exynos8895_kinibi400/MobiCoreDriver
