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
#include <ddk/platform-defs.h>

#include <zircon/syscalls.h>
#include <zircon/syscalls/hyp_sys.h>
#include <zircon/types.h>
#include <zircon/device/vfs.h>
#include <zircon/boot/driver-config.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <threads.h>
#include <stdatomic.h>
#include <fcntl.h>

struct hyp_virq_data {
    zx_device_t *dev;
    thrd_t virq_thrd;
    zx_handle_t rsrc;

    uint32_t virqline;

    uint32_t virq;
    zx_handle_t virq_handle;
    atomic_uint_fast64_t payload;
    bool raised;
};

static int virq_handler(void* arg) {
    struct hyp_virq_data *virq_data = arg;
    zx_status_t status;
    uint64_t payload;

    for (;;) {
        status = zx_interrupt_wait(virq_data->virq_handle, NULL);
        if (status == ZX_ERR_CANCELED) {
            return 0;
        } else if (status != ZX_OK) {
            printf("hyp-virq: interrupt wait failed: %d\n", status);
            return status;
        }

        status = zx_hyp_virq_get_payload(virq_data->rsrc, virq_data->virq, &payload);
        if (status != ZX_OK) {
            printf("hyp-virq: virq get payload failed: %d\n", status);
            return status;
        }

        atomic_fetch_or_explicit(&virq_data->payload, payload, memory_order_acquire);
        device_state_set(virq_data->dev, DEV_STATE_READABLE);
        virq_data->raised = true;
    }

    return 0;
}

zx_status_t hyp_virq_open(void* ctx, zx_device_t** dev_out, uint32_t flags) {
    struct hyp_virq_data *virq_data = ctx;

    if ((flags & ZX_FS_RIGHT_READABLE) && (virq_data->virq == 0)) {
        return ZX_ERR_ACCESS_DENIED;
    }

    if ((flags & ZX_FS_RIGHT_WRITABLE) && (virq_data->virqline == 0)) {
        return ZX_ERR_ACCESS_DENIED;
    }

    return ZX_OK;
}

static zx_status_t hyp_virq_read(void* ctx, void* buf,
        size_t count, zx_off_t off, size_t* actual) {
    struct hyp_virq_data *virq_data = ctx;
    uint64_t payload;

    if (virq_data->virq == 0) {
        return ZX_ERR_UNAVAILABLE;
    }

    if (!virq_data->raised) {
        return ZX_ERR_SHOULD_WAIT;
    }

    virq_data->raised = false;
    device_state_clr(virq_data->dev, DEV_STATE_READABLE);
    payload = atomic_exchange_explicit(&virq_data->payload, 0, memory_order_release);

    if (count >= sizeof(payload)) {
        memcpy(buf, &payload, sizeof(payload));
        *actual = sizeof(payload);
    } else if (count == 0) {
        *actual = 0;
    } else {
        return ZX_ERR_IO;
    }

    return ZX_OK;
}

static zx_status_t hyp_virq_write(void* ctx, const void* buf,
        size_t count, zx_off_t off, size_t* actual) {
    struct hyp_virq_data *virq_data = ctx;
    zx_status_t status;
    uint64_t payload;

    if (virq_data->virqline == 0) {
        return ZX_ERR_UNAVAILABLE;
    }

    if (count == sizeof(payload)) {
        memcpy(&payload, buf, sizeof(payload));
    } else if (count == 0) {
        payload = 0;
    } else {
        return ZX_ERR_IO;
    }

    status = zx_hyp_virq_raise(virq_data->rsrc, virq_data->virqline, payload);
    if (status != ZX_OK) {
        printf("hyp-virq: virq raise failed: %d\n", status);
        return ZX_ERR_IO;
    }

    *actual = count;

    return ZX_OK;
}

static void hyp_virq_unbind(void* ctx) {
    struct hyp_virq_data *virq_data = ctx;

    device_remove(virq_data->dev);
}

static void hyp_virq_release(void* ctx) {
    struct hyp_virq_data *virq_data = ctx;

    if (virq_data->virq_handle != ZX_HANDLE_INVALID) {
        zx_interrupt_destroy(virq_data->virq_handle);
        zx_handle_close(virq_data->virq_handle);
        thrd_join(virq_data->virq_thrd, NULL);
    }

    free(virq_data);
}

static zx_protocol_device_t hyp_virq_protocol = {
    .version = DEVICE_OPS_VERSION,
    .open = hyp_virq_open,
    .read = hyp_virq_read,
    .write = hyp_virq_write,
    .unbind = hyp_virq_unbind,
    .release = hyp_virq_release,
};

static zx_status_t hyp_virq_bind(void* ctx, zx_device_t* parent) {
    zx_status_t status;
    hyp_virq_info_t info;
    size_t actual;
    struct hyp_virq_data *virq_data = calloc(1, sizeof(struct hyp_virq_data));

    if (virq_data == NULL) {
        return ZX_ERR_NO_MEMORY;
    }

    status = device_get_metadata(parent, HYP_VIRQ_METADATA, &info,
            sizeof(info), &actual);
    if ((status != ZX_OK) || (actual != sizeof(info))) {
        status = ZX_ERR_NOT_FOUND;
        goto get_metadata_fail;
    }

    virq_data->rsrc = get_root_resource();
    virq_data->virq = info.virq;
    virq_data->virqline = info.virqline;
    virq_data->payload = 0;
    virq_data->raised = false;

    if (virq_data->virq != 0) {
        status = zx_interrupt_create(virq_data->rsrc, virq_data->virq,
                ZX_INTERRUPT_MODE_EDGE_HIGH, &virq_data->virq_handle);
        if (status != ZX_OK) {
            printf("hyp-virq: irq create failed\n");
            goto irq_create_fail;
        }
    } else {
        virq_data->virq_handle = ZX_HANDLE_INVALID;
    }

    device_add_args_t args = {
        .version = DEVICE_ADD_ARGS_VERSION,
        .name = info.name,
        .ctx = virq_data,
        .proto_id = ZX_PROTOCOL_HYP_VIRQ,
        .ops = &hyp_virq_protocol,
    };

    status = device_add(parent, &args, &virq_data->dev);
    if (status != ZX_OK) {
        printf("hyp-virq: device add failed\n");
        goto device_add_fail;
    }

    if (virq_data->virq != 0) {
        int ret = thrd_create(&virq_data->virq_thrd, virq_handler, virq_data);
        if (ret != thrd_success) {
            printf("hyp-virq: thread create failed\n");
            device_remove(virq_data->dev);
            return ZX_ERR_NO_MEMORY;
        }
    }

    if (virq_data->virqline != 0) {
        device_state_set(virq_data->dev, DEV_STATE_WRITABLE);
    }

    return ZX_OK;

device_add_fail:
    zx_interrupt_destroy(virq_data->virq_handle);
    zx_handle_close(virq_data->virq_handle);
irq_create_fail:
get_metadata_fail:
    free(virq_data);

    return status;
}

static zx_driver_ops_t hyp_virq_driver_ops = {
    .version = DRIVER_OPS_VERSION,
    .bind = hyp_virq_bind,
};

ZIRCON_DRIVER_BEGIN(hyp_virq, hyp_virq_driver_ops, "zircon", "0.1", 3)
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_VID, PDEV_VID_OKL4),
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_PID, PDEV_PID_OKL4),
    BI_MATCH_IF(EQ, BIND_PLATFORM_DEV_DID, PDEV_DID_HYP_VIRQ),
ZIRCON_DRIVER_END(hyp_virq)
