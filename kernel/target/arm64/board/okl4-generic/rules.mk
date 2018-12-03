# Copyright 2018 Cog Systems Pty Ltd
#
# Use of this source code is governed by a MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT

ifeq ($(OKL4_GUEST),true)

LOCAL_DIR := $(GET_LOCAL_DIR)

PLATFORM_BOARD_NAME := okl4-generic
PLATFORM_USE_SHIM := true
PLATFORM_USE_GZIP := true
PLATFORM_USE_MKBOOTIMG := true
PLATFORM_DTB_TYPE := append
PLATFORM_MEMBASE := 0
PLATFORM_CMDLINE := "TERM=xterm kernel.oom.redline-mb=4"
PLATFORM_DTB_PATH := $(LOCAL_DIR)/device-tree.dtb
PLATFORM_KERNEL_OFFSET := 0x00008000

GLOBAL_COMPILEFLAGS += -DOKL4_GUEST -DDISABLE_KASLR

include make/board.mk

endif
