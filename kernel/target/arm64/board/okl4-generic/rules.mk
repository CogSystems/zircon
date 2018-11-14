# Copyright 2018 Cog Systems Pty Ltd
#
# Use of this source code is governed by a MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT

ifeq ($(OKL4_GUEST),true)

LOCAL_DIR := $(GET_LOCAL_DIR)

PLATFORM_BOARD_NAME := okl4-generic
PLATFORM_USE_SHIM := true
PLATFORM_USE_GZIP := false

GLOBAL_COMPILEFLAGS += -DOKL4_GUEST -DDISABLE_KASLR

include make/board.mk

endif
