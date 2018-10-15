# Copyright (c) 2018 Cog Systems Pty Ltd

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS := \
	$(LOCAL_DIR)/empty.c \

MODULE_DEPS := \
	kernel/lib/okl4_environment \

include make/module.mk
