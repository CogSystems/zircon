# Copyright (c) 2018 Cog Systems Pty Ltd

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_TYPE := userapp
MODULE_GROUP := misc

MODULE_SRCS += \
    $(LOCAL_DIR)/hyp-virq-test.c

MODULE_LIBS := \
    system/ulib/zircon \
    system/ulib/fdio \
    system/ulib/c

include make/module.mk
