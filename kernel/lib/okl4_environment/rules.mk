# Copyright (c) 2018 Cog Systems Pty Ltd

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS := \
	$(LOCAL_DIR)/empty.c \

include make/module.mk
