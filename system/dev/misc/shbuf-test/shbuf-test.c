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
#include <threads.h>

#define IRQ_CLR_SIGNAL  ZX_USER_SIGNAL_0

struct shbuf_data {
    zx_device_t *dev;
    zx_handle_t event;
};

static int event_wait(void* arg) {
    struct shbuf_data *data = arg;
    zx_status_t status;
    zx_device_t *parent = device_get_parent(data->dev);

    for (;;) {
        status = device_state_wait(parent, DEV_STATE_OOB,
                ZX_TIME_INFINITE, NULL);
        if (status != ZX_OK) {
            break;
        }

        device_state_set(data->dev, DEV_STATE_OOB);

        status = zx_object_wait_one(data->event, IRQ_CLR_SIGNAL,
                ZX_TIME_INFINITE, NULL);
        if (status != ZX_OK) {
            break;
        }

        device_state_clr(data->dev, DEV_STATE_OOB);
        zx_object_signal(data->event, IRQ_CLR_SIGNAL, 0);
    }

    return 0;
}

static zx_status_t shbuf_test_read(void* ctx, void* buf,
        size_t count, zx_off_t off, size_t* actual) {
    struct shbuf_data *data = ctx;
    zx_device_t *parent = device_get_parent(data->dev);
    return device_read(parent, buf, count, off, actual);
}

static zx_status_t shbuf_test_write(void* ctx, const void* buf,
        size_t count, zx_off_t off, size_t* actual) {
    struct shbuf_data *data = ctx;
    zx_device_t *parent = device_get_parent(data->dev);
    return device_write(parent, buf, count, off, actual);
}

static zx_off_t shbuf_test_get_size(void* ctx) {
    struct shbuf_data *data = ctx;
    zx_device_t *parent = device_get_parent(data->dev);
    return device_get_size(parent);
}

static zx_status_t shbuf_test_ioctl(void* ctx, uint32_t op, const void* in_buf,
        size_t in_len, void* out_buf, size_t out_len, size_t* out_actual) {
    struct shbuf_data *data = ctx;
    zx_device_t *parent = device_get_parent(data->dev);
    zx_status_t status;

    status = device_ioctl(parent, op, in_buf, in_len, out_buf, out_len, out_actual);

    if (status == ZX_OK && op == IOCTL_LINK_SHBUF_IRQ_CLR) {
        zx_object_signal(data->event, 0, IRQ_CLR_SIGNAL);
    }

    return ZX_OK;
}

static void shbuf_test_unbind(void *ctx) {
    struct shbuf_data *data = ctx;
    device_remove(data->dev);
}

static void shbuf_test_release(void *ctx) {
    struct shbuf_data *data = ctx;
    zx_handle_close(data->event);
    free(data);
}

static zx_protocol_device_t shbuf_test_protocol = {
    .version = DEVICE_OPS_VERSION,
    .read = shbuf_test_read,
    .write = shbuf_test_write,
    .get_size = shbuf_test_get_size,
    .ioctl = shbuf_test_ioctl,
    .unbind = shbuf_test_unbind,
    .release = shbuf_test_release,
};

static zx_status_t shbuf_test_bind(void* ctx, zx_device_t* parent) {
    zx_status_t status;
    thrd_t thrd;
    struct shbuf_data *data = calloc(1, sizeof(struct shbuf_data));

    if (data == NULL) {
        return ZX_ERR_NO_MEMORY;
    }

    status = zx_event_create(0, &data->event);
    if (status != ZX_OK) {
        return status;
    }

    device_add_args_t args = {
        .version = DEVICE_ADD_ARGS_VERSION,
        .name = "shbuf-test",
        .ctx = data,
        .proto_id = ZX_PROTOCOL_SHBUF_TEST,
        .ops = &shbuf_test_protocol,
    };

    status = device_add(parent, &args, &data->dev);
    if (status != ZX_OK) {
        zx_handle_close(data->event);
    }

    int ret = thrd_create(&thrd, event_wait, data);
    if (ret != thrd_success) {
        device_remove(data->dev);
        return ZX_ERR_NO_MEMORY;
    }

    return status;
}

static zx_driver_ops_t shbuf_test_driver_ops = {
    .version = DRIVER_OPS_VERSION,
    .bind = shbuf_test_bind,
};

ZIRCON_DRIVER_BEGIN(shbuf_test, shbuf_test_driver_ops, "zircon", "0.1", 2)
    BI_ABORT_IF(NE, BIND_PROTOCOL, ZX_PROTOCOL_LINK_SHBUF),
    BI_MATCH_IF(EQ, BIND_LINK_SHBUF_TYPE, LINK_SHBUF_TYPE_TEST),
ZIRCON_DRIVER_END(shbuf_test)
