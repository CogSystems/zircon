# Copyright 2018 The Fuchsia Authors
#
# Use of this source code is governed by a MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT

LOCAL_DIR := $(GET_LOCAL_DIR)

PLATFORM := generic-arm

# include rules for our various arm64 boards
ifeq ($(OKL4_GUEST),true)
include $(LOCAL_DIR)/board/okl4-generic/rules.mk
else
include $(LOCAL_DIR)/board/*/rules.mk
endif
