/*
 * Copyright 2018 Cog Systems Pty Ltd. All rights reserved.
 *
 * Use of this source code is governed by the 3-Clause BSD license:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <threads.h>
#include <unistd.h>

#include <ddk/binding.h>
#include <ddk/debug.h>
#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/platform-defs.h>
#include <ddk/protocol/platform-bus.h>

#include <zircon/boot/driver-config.h>

#define MAX_LINK_SHBUF  8

static zx_protocol_device_t okl4_device_protocol = {
    .version = DEVICE_OPS_VERSION,
};

static zx_status_t okl4_bind(void* ctx, zx_device_t* parent) {
    zx_status_t status;
    pbus_protocol_t pbus;

    status = device_get_protocol(parent, ZX_PROTOCOL_PBUS, &pbus);
    if (status != ZX_OK) {
        return status;
    }

    // Add dummy board driver
    device_add_args_t args = {
        .version = DEVICE_ADD_ARGS_VERSION,
        .name = "okl4",
        .ctx = NULL,
        .ops = &okl4_device_protocol,
        .flags = DEVICE_ADD_NON_BINDABLE,
    };

    status = device_add(parent, &args, NULL);
    if (status != ZX_OK) {
        return status;
    }

    // Attempt to add link shbuf devices
    // Note: if this is slow, spin off a thread
    for (uint32_t i = 0; i < MAX_LINK_SHBUF; i++) {
        pbus_boot_metadata_t metadata = {
            .zbi_type = LINK_SHBUF_METADATA,
            .zbi_extra = i,
        };
        pbus_dev_t shbuf_dev = {
            .name = "link-shbuf",
            .vid = PDEV_VID_OKL4,
            .pid = PDEV_PID_OKL4,
            .did = PDEV_DID_LINK_SHBUF,
            .boot_metadata_list = &metadata,
            .boot_metadata_count = 1,
        };
        status = pbus_device_add(&pbus, &shbuf_dev);
        if (status != ZX_OK) {
            break;
        }
    }

    return ZX_OK;
}

static zx_driver_ops_t okl4_driver_ops = {
    .version = DRIVER_OPS_VERSION,
    .bind = okl4_bind,
};

ZIRCON_DRIVER_BEGIN(okl4, okl4_driver_ops, "zircon", "0.1", 3)
    BI_ABORT_IF(NE, BIND_PROTOCOL, ZX_PROTOCOL_PBUS),
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_VID, PDEV_VID_OKL4),
    BI_MATCH_IF(EQ, BIND_PLATFORM_DEV_PID, PDEV_PID_OKL4),
ZIRCON_DRIVER_END(okl4)
