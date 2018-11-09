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
#include <zircon/device/vs-shbuf.h>
#include <zircon/device/vfs.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <threads.h>
#include <stdatomic.h>
#include <fcntl.h>

struct vs_shbuf_data {
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
    mmio_buffer_t tx_buffer;
    mmio_buffer_t rx_buffer;
    uint32_t tx_rwx;
    uint32_t rx_rwx;
    uint32_t rwx;
};

static int virq_handler(void* arg) {
    struct vs_shbuf_data *shbuf = arg;
    zx_status_t status;
    zx_hyp_sys_args_t args = {0};
    uint64_t payload;

    for (;;) {
        status = zx_interrupt_wait(shbuf->virq_handle, NULL);
        if (status == ZX_ERR_CANCELED) {
            /* Driver is being torn down, silently exit */
            return 0;
        } else if (status != ZX_OK) {
            printf("vs-shbuf: interrupt wait failed: %d\n", status);
            return status;
        }

        args.x0 = shbuf->virq;
        status = zx_hyp_syscall(shbuf->rsrc,
                HYP_SYSCALL_INTERRUPT_GET_PAYLOAD, &args);
        if ((status != ZX_OK) || (args.x0 != 0)) {
            printf("vs-shbuf: hyp syscall (get payload) failed: %d, %lu\n",
                    status, args.x0);
            return status;
        }
        payload = args.x1;

        (void)atomic_fetch_or(&shbuf->virq_payload, payload);

        /* DEV_STATE_OOB sets the POLLPRI flag */
        device_state_set(shbuf->dev, DEV_STATE_OOB);
    }

    return 0;
}

zx_status_t vs_shbuf_open(void* ctx, zx_device_t** dev_out, uint32_t flags) {
    struct vs_shbuf_data *shbuf = ctx;
    uint32_t mask = ZX_FS_RIGHT_READABLE |
            ZX_FS_RIGHT_WRITABLE | ZX_FS_FLAG_VNODE_REF_ONLY;

    if ((mask & flags) != flags) {
        return ZX_ERR_ACCESS_DENIED;
    }

    if ((flags & ZX_FS_RIGHT_READABLE) && !(shbuf->rwx & S_IROTH)) {
        return ZX_ERR_ACCESS_DENIED;
    }

    if ((flags & ZX_FS_RIGHT_WRITABLE) && !(shbuf->rwx & S_IWOTH)) {
        return ZX_ERR_ACCESS_DENIED;
    }

    return ZX_OK;
}

static bool vs_shbuf_valid_access(size_t size, zx_off_t off, size_t count) {
    /* Disallow short reads and writes */
    return ((off < off + count) && (off + count <= size));
}

static zx_status_t vs_shbuf_read(void* ctx, void* buf,
        size_t count, zx_off_t off, size_t* actual) {
    struct vs_shbuf_data *shbuf = ctx;

    if (!vs_shbuf_valid_access(shbuf->rx_buffer.size, off, count)) {
        return ZX_ERR_OUT_OF_RANGE;
    }

    memcpy(buf, shbuf->rx_buffer.vaddr + off, count);
    *actual = count;

    return ZX_OK;
}

static zx_status_t vs_shbuf_write(void* ctx, const void* buf,
        size_t count, zx_off_t off, size_t* actual) {
    struct vs_shbuf_data *shbuf = ctx;

    if (!vs_shbuf_valid_access(shbuf->tx_buffer.size, off, count)) {
        return ZX_ERR_OUT_OF_RANGE;
    }

    memcpy(shbuf->tx_buffer.vaddr + off, buf, count);
    *actual = count;

    return ZX_OK;
}

static zx_off_t vs_shbuf_get_size(void* ctx) {
    struct vs_shbuf_data *shbuf = ctx;

    /* NOTE: this assume tx and rx buffers are the same size */
    return shbuf->tx_buffer.size;
}

static zx_status_t vs_shbuf_ioctl_irq_tx(struct vs_shbuf_data *shbuf,
        const void* in_buf, size_t in_len, size_t* out_actual) {
    uint64_t payload;
    zx_status_t status;
    zx_hyp_sys_args_t args = {0};

    if (shbuf->virqline == 0) {
        /* No outgoing virq */
        return ZX_ERR_UNAVAILABLE;
    }

    /* We expect in_buf to point to a payload */
    if (in_len < sizeof(uint64_t)) {
        return ZX_ERR_INVALID_ARGS;
    }
    payload = *(uint64_t *)in_buf;

    args.x0 = shbuf->virqline;
    args.x1 = payload;
    status = zx_hyp_syscall(shbuf->rsrc, HYP_SYSCALL_VINTERRUPT_RAISE, &args);
    if ((status != ZX_OK) || (args.x0 != 0)) {
        printf("vs-shbuf: hyp syscall (virq raise) failed: %d, %lu\n",
                status, args.x0);
        return ZX_ERR_INVALID_ARGS;
    }

    *out_actual = 0;

    return ZX_OK;
}

static zx_status_t vs_shbuf_ioctl_irq_clr(struct vs_shbuf_data *shbuf,
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

static zx_status_t vs_shbuf_ioctl_get_vmo(struct vs_shbuf_data *shbuf,
        void* out_buf, size_t out_len, size_t* out_actual) {
    zx_handle_t *vmo;
    zx_status_t status;
    zx_rights_t rights;

    if (!out_buf || out_len < sizeof(zx_handle_t)) {
        return ZX_ERR_INVALID_ARGS;
    }

    vmo = out_buf;

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

    status = zx_handle_duplicate(shbuf->tx_buffer.vmo, rights, &vmo[0]);
    if (status != ZX_OK) {
        return ZX_ERR_UNAVAILABLE;
    }
    status = zx_handle_duplicate(shbuf->rx_buffer.vmo, rights, &vmo[1]);
    if (status != ZX_OK) {
        return ZX_ERR_UNAVAILABLE;
    }

    *out_actual = sizeof(*vmo) * 2;

    return ZX_OK;
}

static zx_status_t vs_shbuf_ioctl(void* ctx, uint32_t op, const void* in_buf,
        size_t in_len, void* out_buf, size_t out_len, size_t* out_actual) {
    struct vs_shbuf_data *shbuf = ctx;

    switch (op) {
    case IOCTL_VS_SHBUF_IRQ_TX:
        return vs_shbuf_ioctl_irq_tx(shbuf, in_buf, in_len, out_actual);
    case IOCTL_VS_SHBUF_IRQ_CLR:
        return vs_shbuf_ioctl_irq_clr(shbuf, out_buf, out_len, out_actual);
    case IOCTL_VS_SHBUF_GET_VMO:
        return vs_shbuf_ioctl_get_vmo(shbuf, out_buf, out_len, out_actual);
    default:
        return ZX_ERR_NOT_SUPPORTED;
    }
}

static void vs_shbuf_unbind(void* ctx) {
    struct vs_shbuf_data *shbuf = ctx;

    device_remove(shbuf->dev);
}

static void vs_shbuf_release(void* ctx) {
    struct vs_shbuf_data *shbuf = ctx;

    if (shbuf->virq_handle != ZX_HANDLE_INVALID) {
        /* Cancel the virq thread's irq wait */
        zx_interrupt_destroy(shbuf->virq_handle);
        zx_handle_close(shbuf->virq_handle);
        /* Wait for thread to exit */
        thrd_join(shbuf->virq_thrd, NULL);
    }

    mmio_buffer_release(&shbuf->rx_buffer);
    mmio_buffer_release(&shbuf->tx_buffer);
    free(shbuf);
}

static zx_protocol_device_t vs_shbuf_protocol = {
    .version = DEVICE_OPS_VERSION,
    .open = vs_shbuf_open,
    .read = vs_shbuf_read,
    .write = vs_shbuf_write,
    .get_size = vs_shbuf_get_size,
    .ioctl = vs_shbuf_ioctl,
    .unbind = vs_shbuf_unbind,
    .release = vs_shbuf_release,
};

static zx_status_t vs_shbuf_bind(void* ctx, zx_device_t* parent) {
    vs_shbuf_info_t info;
    zx_status_t status;
    size_t actual;
    struct vs_shbuf_data *shbuf = calloc(1, sizeof(struct vs_shbuf_data));

    if (shbuf == NULL) {
        return ZX_ERR_NO_MEMORY;
    }

    status = device_get_metadata(parent, VS_SHBUF_METADATA, &info,
            sizeof(info), &actual);
    if ((status != ZX_OK) || (actual != sizeof(info))) {
        printf("vs-shbuf: get metadata failed %x %lx\n", status, actual);
        status = ZX_ERR_NOT_FOUND;
        goto get_metadata_fail;
    }

    shbuf->rsrc = get_root_resource();
    shbuf->virq = info.virq;
    shbuf->virqline = info.virqline;
    shbuf->tx_rwx = info.tx_buf.rwx;
    shbuf->rx_rwx = info.rx_buf.rwx;
    shbuf->rwx = info.tx_buf.rwx | info.rx_buf.rwx;

    status = mmio_buffer_init_physical(&shbuf->tx_buffer, info.tx_buf.paddr,
            info.tx_buf.size, shbuf->rsrc, ZX_CACHE_POLICY_CACHED);
    if (status != ZX_OK) {
        printf("vs-shbuf: mmio init (tx) failed\n");
        goto mmio_init_tx_fail;
    }
    status = mmio_buffer_init_physical(&shbuf->rx_buffer, info.rx_buf.paddr,
            info.rx_buf.size, shbuf->rsrc, ZX_CACHE_POLICY_CACHED);
    if (status != ZX_OK) {
        printf("vs-shbuf: mmio init (rx) failed\n");
        goto mmio_init_rx_fail;
    }

    if (shbuf->virq != 0) {
        status = zx_interrupt_create(shbuf->rsrc, shbuf->virq,
                ZX_INTERRUPT_MODE_EDGE_HIGH, &shbuf->virq_handle);
        if (status != ZX_OK) {
            printf("vs-shbuf: irq create failed\n");
            goto irq_create_fail;
        }
    } else {
        /* No incoming virq */
        shbuf->virq_handle = ZX_HANDLE_INVALID;
    }

    zx_device_prop_t props[] = {
        {BIND_PROTOCOL, 0, ZX_PROTOCOL_VS_SHBUF},
        {BIND_VS_SHBUF_TYPE, 0, 0},
    };

    device_add_args_t args = {
        .version = DEVICE_ADD_ARGS_VERSION,
        .name = info.name,
        .ctx = shbuf,
        .proto_id = ZX_PROTOCOL_VS_SHBUF,
        .ops = &vs_shbuf_protocol,
        .props = props,
        .prop_count = countof(props),
    };

    status = device_add(parent, &args, &shbuf->dev);
    if (status != ZX_OK) {
        printf("vs-shbuf: device_add() failed\n");
        goto device_add_fail;
    }

    /* Shared memory is always readable & writable */
    device_state_set(shbuf->dev, (DEV_STATE_READABLE | DEV_STATE_WRITABLE));

    /* Create thread for handling incoming virqs */
    if (shbuf->virq_handle != ZX_HANDLE_INVALID) {
        int ret = thrd_create(&shbuf->virq_thrd, virq_handler, shbuf);
        if (ret != thrd_success) {
            printf("vs-shbuf: thrd_create() failed\n");
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
    mmio_buffer_release(&shbuf->rx_buffer);
mmio_init_rx_fail:
    mmio_buffer_release(&shbuf->tx_buffer);
mmio_init_tx_fail:
get_metadata_fail:
    free(shbuf);

    return status;
}

static zx_driver_ops_t vs_shbuf_driver_ops = {
    .version = DRIVER_OPS_VERSION,
    .bind = vs_shbuf_bind,
};

ZIRCON_DRIVER_BEGIN(vs_shbuf, vs_shbuf_driver_ops, "zircon", "0.1", 3)
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_VID, PDEV_VID_OKL4),
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_PID, PDEV_PID_OKL4),
    BI_MATCH_IF(EQ, BIND_PLATFORM_DEV_DID, PDEV_DID_VS_SHBUF),
ZIRCON_DRIVER_END(vs_shbuf)
