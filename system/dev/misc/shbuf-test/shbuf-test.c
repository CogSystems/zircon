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

#include <ddk/binding.h>
#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/mmio-buffer.h>
#include <ddk/platform-defs.h>

#include <zircon/syscalls.h>
#include <zircon/types.h>
#include <zircon/device/link-shbuf.h>
#include <zircon/device/vfs.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

static int dev_index = 0;

static zx_status_t shbuf_test_read(void* ctx, void* buf,
        size_t count, zx_off_t off, size_t* actual) {
    zx_device_t *parent = ctx;
    return device_read(parent, buf, count, off, actual);
}

static zx_status_t shbuf_test_write(void* ctx, const void* buf,
        size_t count, zx_off_t off, size_t* actual) {
    zx_device_t *parent = ctx;
    return device_write(parent, buf, count, off, actual);
}

static zx_off_t shbuf_test_get_size(void* ctx) {
    zx_device_t *parent = ctx;
    return device_get_size(parent);
}

static zx_status_t shbuf_test_ioctl(void* ctx, uint32_t op, const void* in_buf,
        size_t in_len, void* out_buf, size_t out_len, size_t* out_actual) {
    zx_device_t *parent = ctx;
    return device_ioctl(parent, op, in_buf, in_len, out_buf, out_len, out_actual);
}

static zx_protocol_device_t shbuf_test_protocol = {
    .version = DEVICE_OPS_VERSION,
    .read = shbuf_test_read,
    .write = shbuf_test_write,
    .get_size = shbuf_test_get_size,
    .ioctl = shbuf_test_ioctl,
};

static zx_status_t shbuf_test_bind(void* ctx, zx_device_t* parent) {
    char devname[32];
    zx_device_t *shbuf_dev;

    sprintf(devname, "shbuf-test%d", dev_index);

    device_add_args_t args = {
        .version = DEVICE_ADD_ARGS_VERSION,
        .name = devname,
        .ctx = parent,
        .proto_id = ZX_PROTOCOL_SHBUF_TEST,
        .ops = &shbuf_test_protocol,
    };

    return device_add(parent, &args, &shbuf_dev);
}

static zx_driver_ops_t shbuf_test_driver_ops = {
    .version = DRIVER_OPS_VERSION,
    .bind = shbuf_test_bind,
};

ZIRCON_DRIVER_BEGIN(shbuf_test, shbuf_test_driver_ops, "zircon", "0.1", 2)
    BI_ABORT_IF(NE, BIND_PROTOCOL, ZX_PROTOCOL_LINK_SHBUF),
    BI_MATCH_IF(EQ, BIND_LINK_SHBUF_TYPE, LINK_SHBUF_TYPE_TEST),
ZIRCON_DRIVER_END(shbuf_test)
