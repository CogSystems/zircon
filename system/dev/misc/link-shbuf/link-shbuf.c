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
#include <zircon/syscalls/hyp_sys.h>
#include <zircon/types.h>
#include <zircon/device/link-shbuf.h>
#include <zircon/device/vfs.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <threads.h>
#include <stdatomic.h>
#include <fcntl.h>

struct link_shbuf_data {
    /* Device data */
    zx_device_t *dev;
    thrd_t virq_thrd;
    zx_handle_t rsrc;
    /* Outgoing virq */
    uint32_t virqline;
    /* Incoming virq */
    uint32_t virq;
    zx_handle_t virq_handle;
    atomic_uint_fast64_t virq_payload;
    /* Shared memory region */
    mmio_buffer_t buffer;
    uint32_t rwx;
};

static int virq_handler(void* arg) {
    struct link_shbuf_data *shbuf = arg;
    zx_status_t status;
    uint64_t payload;

    for (;;) {
        status = zx_interrupt_wait(shbuf->virq_handle, NULL);
        if (status == ZX_ERR_CANCELED) {
            /* Driver is being torn down, silently exit */
            return 0;
        } else if (status != ZX_OK) {
            printf("link-shbuf: interrupt wait failed: %d\n", status);
            return status;
        }

        status = zx_hyp_virq_get_payload(shbuf->rsrc, shbuf->virq, &payload);
        if (status != ZX_OK) {
            printf("link-shbuf: virq get payload failed: %d\n", status);
            return status;
        }

        (void)atomic_fetch_or(&shbuf->virq_payload, payload);

        /* DEV_STATE_OOB sets the POLLPRI flag */
        device_state_set(shbuf->dev, DEV_STATE_OOB);
    }

    return 0;
}

zx_status_t link_shbuf_open(void* ctx, zx_device_t** dev_out, uint32_t flags) {
    struct link_shbuf_data *shbuf = ctx;

    if ((flags & ZX_FS_RIGHT_READABLE) && !(shbuf->rwx & S_IROTH)) {
        return ZX_ERR_ACCESS_DENIED;
    }

    if ((flags & ZX_FS_RIGHT_WRITABLE) && !(shbuf->rwx & S_IWOTH)) {
        return ZX_ERR_ACCESS_DENIED;
    }

    return ZX_OK;
}

static bool link_shbuf_valid_access(size_t size, zx_off_t off, size_t count) {
    /* Disallow short reads and writes */
    return ((off < off + count) && (off + count <= size));
}

static zx_status_t link_shbuf_read(void* ctx, void* buf,
        size_t count, zx_off_t off, size_t* actual) {
    struct link_shbuf_data *shbuf = ctx;

    if (!link_shbuf_valid_access(shbuf->buffer.size, off, count)) {
        return ZX_ERR_OUT_OF_RANGE;
    }

    memcpy(buf, shbuf->buffer.vaddr + off, count);
    *actual = count;

    return ZX_OK;
}

static zx_status_t link_shbuf_write(void* ctx, const void* buf,
        size_t count, zx_off_t off, size_t* actual) {
    struct link_shbuf_data *shbuf = ctx;

    if (!link_shbuf_valid_access(shbuf->buffer.size, off, count)) {
        return ZX_ERR_OUT_OF_RANGE;
    }

    memcpy(shbuf->buffer.vaddr + off, buf, count);
    *actual = count;

    return ZX_OK;
}

static zx_off_t link_shbuf_get_size(void* ctx) {
    struct link_shbuf_data *shbuf = ctx;

    return shbuf->buffer.size;
}

static zx_status_t link_shbuf_ioctl_irq_tx(struct link_shbuf_data *shbuf,
        const void* in_buf, size_t in_len, size_t* out_actual) {
    uint64_t payload;
    zx_status_t status;

    if (shbuf->virqline == 0) {
        /* No outgoing virq */
        return ZX_ERR_UNAVAILABLE;
    }

    /* We expect in_buf to point to a payload */
    if (in_len < sizeof(uint64_t)) {
        return ZX_ERR_INVALID_ARGS;
    }
    payload = *(uint64_t *)in_buf;

    status = zx_hyp_virq_raise(shbuf->rsrc, shbuf->virqline, payload);
    if (status != ZX_OK) {
        printf("link-shbuf: virq raise failed: %d\n", status);
        return status;
    }

    *out_actual = 0;

    return ZX_OK;
}

static zx_status_t link_shbuf_ioctl_irq_clr(struct link_shbuf_data *shbuf,
        void* out_buf, size_t out_len, size_t* out_actual) {
    uint64_t payload;

    if (out_len < sizeof(uint64_t)) {
        return ZX_ERR_INVALID_ARGS;
    }

    device_state_clr(shbuf->dev, DEV_STATE_OOB);
    payload = atomic_exchange(&shbuf->virq_payload, 0);

    /* Write payload to out_buf */
    *(uint64_t *)out_buf = payload;
    *out_actual = sizeof(uint64_t);

    return ZX_OK;
}

static zx_status_t link_shbuf_ioctl_get_vmo(struct link_shbuf_data *shbuf,
        void* out_buf, size_t out_len, size_t* out_actual) {
    zx_handle_t vmo;
    zx_status_t status;
    zx_rights_t rights;

    if (out_len < sizeof(zx_handle_t)) {
        return ZX_ERR_INVALID_ARGS;
    }

    /*
     * We must give transfer rights so VMO can be received by caller.
     * Otherwise, we limit rights to the shbuf's rwx + map.
     */
    rights = ZX_RIGHT_TRANSFER | ZX_RIGHT_MAP;
    if (shbuf->rwx & S_IROTH) {
        rights |= ZX_RIGHT_READ;
    }
    if (shbuf->rwx & S_IWOTH) {
        rights |= ZX_RIGHT_WRITE;
    }
    if (shbuf->rwx & S_IXOTH) {
        rights |= ZX_RIGHT_EXECUTE;
    }

    status = zx_handle_duplicate(shbuf->buffer.vmo, rights, &vmo);
    if (status != ZX_OK) {
        return ZX_ERR_UNAVAILABLE;
    }

    *(zx_handle_t *)out_buf = vmo;
    *out_actual = sizeof(zx_handle_t);

    return ZX_OK;
}

static zx_status_t link_shbuf_ioctl(void* ctx, uint32_t op, const void* in_buf,
        size_t in_len, void* out_buf, size_t out_len, size_t* out_actual) {
    struct link_shbuf_data *shbuf = ctx;

    switch (op) {
    case IOCTL_LINK_SHBUF_IRQ_TX:
        return link_shbuf_ioctl_irq_tx(shbuf, in_buf, in_len, out_actual);
    case IOCTL_LINK_SHBUF_IRQ_CLR:
        return link_shbuf_ioctl_irq_clr(shbuf, out_buf, out_len, out_actual);
    case IOCTL_LINK_SHBUF_GET_VMO:
        return link_shbuf_ioctl_get_vmo(shbuf, out_buf, out_len, out_actual);
    default:
        return ZX_ERR_NOT_SUPPORTED;
    }
}

static void link_shbuf_unbind(void* ctx) {
    struct link_shbuf_data *shbuf = ctx;

    device_remove(shbuf->dev);
}

static void link_shbuf_release(void* ctx) {
    struct link_shbuf_data *shbuf = ctx;

    if (shbuf->virq_handle != ZX_HANDLE_INVALID) {
        /* Cancel the virq thread's irq wait */
        zx_interrupt_destroy(shbuf->virq_handle);
        zx_handle_close(shbuf->virq_handle);
        /* Wait for thread to exit */
        thrd_join(shbuf->virq_thrd, NULL);
    }

    mmio_buffer_release(&shbuf->buffer);
    free(shbuf);
}

static zx_protocol_device_t link_shbuf_protocol = {
    .version = DEVICE_OPS_VERSION,
    .open = link_shbuf_open,
    .read = link_shbuf_read,
    .write = link_shbuf_write,
    .get_size = link_shbuf_get_size,
    .ioctl = link_shbuf_ioctl,
    .unbind = link_shbuf_unbind,
    .release = link_shbuf_release,
};

static zx_status_t link_shbuf_bind(void* ctx, zx_device_t* parent) {
    link_shbuf_info_t info;
    zx_status_t status;
    size_t actual;
    struct link_shbuf_data *shbuf = calloc(1, sizeof(struct link_shbuf_data));

    if (shbuf == NULL) {
        return ZX_ERR_NO_MEMORY;
    }

    status = device_get_metadata(parent, LINK_SHBUF_METADATA, &info,
            sizeof(info), &actual);
    if ((status != ZX_OK) || (actual != sizeof(info))) {
        printf("link-shbuf: get metadata failed %x %lx\n", status, actual);
        status = ZX_ERR_NOT_FOUND;
        goto get_metadata_fail;
    }

    shbuf->rsrc = get_root_resource();
    shbuf->virq = info.virq;
    shbuf->virqline = info.virqline;
    shbuf->rwx = info.rwx;

    status = mmio_buffer_init_physical(&shbuf->buffer, info.paddr, info.size,
            shbuf->rsrc, ZX_CACHE_POLICY_CACHED);
    if (status != ZX_OK) {
        printf("link-shbuf: mmio init failed\n");
        goto mmio_init_fail;
    }

    if (shbuf->virq != 0) {
        status = zx_interrupt_create(shbuf->rsrc, shbuf->virq,
                ZX_INTERRUPT_MODE_EDGE_HIGH, &shbuf->virq_handle);
        if (status != ZX_OK) {
            printf("link-shbuf: irq create failed\n");
            goto irq_create_fail;
        }
    } else {
        /* No incoming virq */
        shbuf->virq_handle = ZX_HANDLE_INVALID;
    }

    zx_device_prop_t props[] = {
        {BIND_PROTOCOL, 0, ZX_PROTOCOL_LINK_SHBUF},
        {BIND_LINK_SHBUF_TYPE, 0, info.type},
    };

    device_add_args_t args = {
        .version = DEVICE_ADD_ARGS_VERSION,
        .name = info.name,
        .ctx = shbuf,
        .proto_id = ZX_PROTOCOL_LINK_SHBUF,
        .ops = &link_shbuf_protocol,
        .props = props,
        .prop_count = countof(props),
    };

    status = device_add(parent, &args, &shbuf->dev);
    if (status != ZX_OK) {
        printf("link-shbuf: device_add() failed\n");
        goto device_add_fail;
    }

    /* Shared memory is always readable & writable */
    device_state_set(shbuf->dev, (DEV_STATE_READABLE | DEV_STATE_WRITABLE));

    /* Create thread for handling incoming virqs */
    if (shbuf->virq_handle != ZX_HANDLE_INVALID) {
        int ret = thrd_create(&shbuf->virq_thrd, virq_handler, shbuf);
        if (ret != thrd_success) {
            printf("link-shbuf: thrd_create() failed\n");
            /* device_remove() will call release hook */
            device_remove(shbuf->dev);
            return ZX_ERR_NO_MEMORY;
        }
    }

    return ZX_OK;

device_add_fail:
    zx_interrupt_destroy(shbuf->virq_handle);
    zx_handle_close(shbuf->virq_handle);
irq_create_fail:
    mmio_buffer_release(&shbuf->buffer);
mmio_init_fail:
get_metadata_fail:
    free(shbuf);

    return status;
}

static zx_driver_ops_t link_shbuf_driver_ops = {
    .version = DRIVER_OPS_VERSION,
    .bind = link_shbuf_bind,
};

ZIRCON_DRIVER_BEGIN(link_shbuf, link_shbuf_driver_ops, "zircon", "0.1", 3)
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_VID, PDEV_VID_OKL4),
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_PID, PDEV_PID_OKL4),
    BI_MATCH_IF(EQ, BIND_PLATFORM_DEV_DID, PDEV_DID_LINK_SHBUF),
ZIRCON_DRIVER_END(link_shbuf)
