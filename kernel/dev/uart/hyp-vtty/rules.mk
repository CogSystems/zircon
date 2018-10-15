# Copyright (c) 2018 Cog Systems Pty Ltd

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS += \
	$(LOCAL_DIR)/uart.c

MODULE_DEPS += \
	kernel/dev/pdev \
	kernel/dev/pdev/uart \
	kernel/lib/microvisor \

include make/module.mk
