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
#include <zircon/syscalls/port.h>
#include <zircon/types.h>
#include <zircon/boot/driver-config.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <threads.h>
#include <assert.h>
#include <string.h>
#include <sys/param.h>

#define HYP_PIPE_TX_IRQ_KEY 0
#define HYP_PIPE_RX_IRQ_KEY 1

struct hyp_pipe_data {
    zx_device_t *dev;
    zx_handle_t rsrc;

    uint32_t tx_pipe;
    uint32_t rx_pipe;
    uint32_t tx_irq;
    uint32_t rx_irq;
    zx_handle_t tx_irq_handle;
    zx_handle_t rx_irq_handle;
    mtx_t pipe_lock;

    size_t max_msg_size;
    int ref_count;

    zx_handle_t port;
    thrd_t irq_thrd;

    bool reset;
    bool tx_maybe_avail;
    bool rx_maybe_avail;

    char *rx_buf;
    size_t rx_buf_count;
    zx_off_t rx_buf_off;

    char *tx_buf;
};

static void update_dev_state(struct hyp_pipe_data *pipe_data) {
    zx_signals_t state = 0;
    zx_signals_t mask = DEV_STATE_READABLE |
            DEV_STATE_WRITABLE | DEV_STATE_HANGUP;

    // Should be holding pipe lock!
    if (pipe_data->rx_maybe_avail)
        state |= DEV_STATE_READABLE;
    if (pipe_data->tx_maybe_avail)
        state |= DEV_STATE_WRITABLE;
    if (pipe_data->reset)
        state |= DEV_STATE_HANGUP;

    device_state_clr_set(pipe_data->dev, mask, state);
}

static int irq_handler(void* arg) {
    struct hyp_pipe_data *pipe_data = arg;
    zx_status_t status;
    zx_port_packet_t pkt;
    uint32_t irq, state;
    zx_handle_t irq_handle;

    for (;;) {
        status = zx_port_wait(pipe_data->port, ZX_TIME_INFINITE, &pkt);
        if (status != ZX_OK) {
            printf("hyp-pipe: port wait failed: %d\n", status);
            return status;
        }
        assert(pkt.type == ZX_PKT_TYPE_INTERRUPT);

        if (pkt.key == HYP_PIPE_TX_IRQ_KEY) {
            irq = pipe_data->tx_irq;
            irq_handle = pipe_data->tx_irq_handle;
        } else if (pkt.key == HYP_PIPE_RX_IRQ_KEY) {
            irq = pipe_data->rx_irq;
            irq_handle = pipe_data->rx_irq_handle;
        } else {
            printf("hyp-pipe: port packet has bad key %lu\n", pkt.key);
            return ZX_ERR_BAD_STATE;
        }

        status = zx_interrupt_ack(irq_handle);
        if (status != ZX_OK) {
            printf("hyp-pipe: irq ack failed: %d\n", status);
            return status;
        }

        status = zx_hyp_pipe_get_state(pipe_data->rsrc, irq, &state);
        if (status != ZX_OK) {
            printf("hyp-pipe: virq get payload failed: %d\n", status);
            return status;
        }

        mtx_lock(&pipe_data->pipe_lock);
        if (irq == pipe_data->tx_irq) {
            if (state & ZX_HYP_PIPE_STATE_RESET) {
                pipe_data->tx_maybe_avail = true;
                pipe_data->reset = true;
            }
            if (state & ZX_HYP_PIPE_STATE_TX_AVAIL) {
                pipe_data->tx_maybe_avail = true;
            }
        } else {
            if (state & ZX_HYP_PIPE_STATE_RESET) {
                pipe_data->rx_maybe_avail = true;
                pipe_data->reset = true;
            }
            if (state & ZX_HYP_PIPE_STATE_RX_AVAIL) {
                pipe_data->rx_maybe_avail = true;
            }
        }
        update_dev_state(pipe_data);
        mtx_unlock(&pipe_data->pipe_lock);
    }

    return 0;
}

zx_status_t hyp_pipe_open(void* ctx, zx_device_t** dev_out, uint32_t flags) {
    struct hyp_pipe_data *pipe_data = ctx;

    mtx_lock(&pipe_data->pipe_lock);
    if (!pipe_data->ref_count) {
        pipe_data->rx_buf_count = 0;

        pipe_data->reset = false;
        pipe_data->tx_maybe_avail = true;
        pipe_data->rx_maybe_avail = true;
        update_dev_state(pipe_data);

        zx_hyp_pipe_control(pipe_data->rsrc, pipe_data->tx_pipe,
                ZX_HYP_PIPE_CTL_TX_READY);
        zx_hyp_pipe_control(pipe_data->rsrc, pipe_data->rx_pipe,
                ZX_HYP_PIPE_CTL_RX_READY);
    }
    pipe_data->ref_count++;
    mtx_unlock(&pipe_data->pipe_lock);

    return ZX_OK;
}

zx_status_t hyp_pipe_close(void* ctx, uint32_t flags) {
    struct hyp_pipe_data *pipe_data = ctx;

    mtx_lock(&pipe_data->pipe_lock);
    pipe_data->ref_count--;
    if (!pipe_data->ref_count) {
        zx_hyp_pipe_control(pipe_data->rsrc, pipe_data->tx_pipe,
                ZX_HYP_PIPE_CTL_RESET);
        zx_hyp_pipe_control(pipe_data->rsrc, pipe_data->rx_pipe,
                ZX_HYP_PIPE_CTL_RESET);

        pipe_data->rx_buf_count = 0;
    }
    mtx_unlock(&pipe_data->pipe_lock);

    return ZX_OK;
}

static zx_status_t hyp_pipe_read(void* ctx, void* buf,
        size_t count, zx_off_t off, size_t* actual) {
    struct hyp_pipe_data *pipe_data = ctx;
    size_t size;
    zx_status_t status;

    *actual = 0;

    if (!count) {
        return ZX_OK;
    }

    mtx_lock(&pipe_data->pipe_lock);

    if (pipe_data->reset) {
        mtx_unlock(&pipe_data->pipe_lock);
        return ZX_ERR_UNAVAILABLE;
    }

    if (!pipe_data->rx_maybe_avail) {
        mtx_unlock(&pipe_data->pipe_lock);
        return ZX_ERR_SHOULD_WAIT;
    }

    // Recv buffered data first
    if (pipe_data->rx_buf_count) {
        size = MIN(pipe_data->rx_buf_count, count);
        memcpy(buf, pipe_data->rx_buf + pipe_data->rx_buf_off, size);

        pipe_data->rx_buf_count -= size;
        pipe_data->rx_buf_off += size;

        buf += size;
        count -= size;
        *actual = size;

        if (!count) {
            mtx_unlock(&pipe_data->pipe_lock);
            return ZX_OK;
        }
    }

    // Recv from pipe
    while (count) {
        status = zx_hyp_pipe_recv(pipe_data->rsrc,
                pipe_data->rx_pipe, pipe_data->rx_buf,
                pipe_data->max_msg_size + sizeof(uint32_t), NULL);
        if (status == ZX_ERR_SHOULD_WAIT) {
            pipe_data->rx_maybe_avail = false;
            update_dev_state(pipe_data);
            if (*actual > 0) {
                status = ZX_OK;
            }
            break;
        } else if (status != ZX_OK) {
            break;
        }

        size = *(uint32_t *)&pipe_data->rx_buf[0];
        if (size > pipe_data->max_msg_size) {
            status = (*actual > 0) ? ZX_OK : ZX_ERR_BAD_STATE;
            break;
        }

        if (size > count) {
            // Store info on leftover data
            pipe_data->rx_buf_count = size - count;
            pipe_data->rx_buf_off = count + sizeof(uint32_t);
            size = count;
        }

        memcpy(buf, pipe_data->rx_buf + sizeof(uint32_t), size);

        buf += size;
        count -= size;
        *actual += size;
    }

    mtx_unlock(&pipe_data->pipe_lock);

    return status;
}

static zx_status_t hyp_pipe_write(void* ctx, const void* buf,
        size_t count, zx_off_t off, size_t* actual) {
    struct hyp_pipe_data *pipe_data = ctx;
    size_t size, pipe_size;
    zx_status_t status;

    *actual = 0;

    if (!count) {
        return ZX_OK;
    }

    mtx_lock(&pipe_data->pipe_lock);

    if (pipe_data->reset) {
        mtx_unlock(&pipe_data->pipe_lock);
        return ZX_ERR_UNAVAILABLE;
    }

    if (!pipe_data->tx_maybe_avail) {
        mtx_unlock(&pipe_data->pipe_lock);
        return ZX_ERR_SHOULD_WAIT;
    }

    while (count) {
        size = MIN(count, pipe_data->max_msg_size);
        pipe_size = roundup(size + sizeof(uint32_t), sizeof(uint32_t));

        *(uint32_t *)&pipe_data->tx_buf[0] = size;
        memcpy(pipe_data->tx_buf + sizeof(uint32_t), buf, size);

        status = zx_hyp_pipe_send(pipe_data->rsrc, pipe_data->tx_pipe,
                pipe_data->tx_buf, pipe_size);
        if (status == ZX_ERR_SHOULD_WAIT) {
            pipe_data->tx_maybe_avail = false;
            update_dev_state(pipe_data);
            if (*actual > 0) {
                status = ZX_OK;
            }
            break;
        } else if (status != ZX_OK) {
            break;
        }

        buf += size;
        count -= size;
        *actual += size;
    }

    mtx_unlock(&pipe_data->pipe_lock);

    return status;
}

static void hyp_pipe_unbind(void* ctx) {
    struct hyp_pipe_data *pipe_data = ctx;

    device_remove(pipe_data->dev);
}

static void hyp_pipe_release(void* ctx) {
    struct hyp_pipe_data *pipe_data = ctx;

    // Destroy irqs and wait for thread to exit
    zx_interrupt_destroy(pipe_data->rx_irq_handle);
    zx_interrupt_destroy(pipe_data->tx_irq_handle);
    zx_handle_close(pipe_data->rx_irq_handle);
    zx_handle_close(pipe_data->tx_irq_handle);
    thrd_join(pipe_data->irq_thrd, NULL);
    // Do remaining clean up
    zx_handle_close(pipe_data->port);
    free(pipe_data->tx_buf);
    free(pipe_data->rx_buf);
    free(pipe_data);
}

static zx_protocol_device_t hyp_pipe_protocol = {
    .version = DEVICE_OPS_VERSION,
    .open = hyp_pipe_open,
    .close = hyp_pipe_close,
    .read = hyp_pipe_read,
    .write = hyp_pipe_write,
    .unbind = hyp_pipe_unbind,
    .release = hyp_pipe_release,
};

static zx_status_t hyp_pipe_bind(void* ctx, zx_device_t* parent) {
    zx_status_t status;
    link_pipe_info_t info;
    size_t actual;
    int ret;
    struct hyp_pipe_data *pipe_data = calloc(1, sizeof(struct hyp_pipe_data));
    if (pipe_data == NULL) {
        return ZX_ERR_NO_MEMORY;
    }

    status = device_get_metadata(parent, LINK_PIPE_METADATA, &info,
            sizeof(info), &actual);
    if ((status != ZX_OK) || (actual != sizeof(info))) {
        status = ZX_ERR_NOT_FOUND;
        goto get_metadata_fail;
    }

    pipe_data->rsrc = get_root_resource();
    pipe_data->tx_pipe = info.tx_kcap;
    pipe_data->rx_pipe = info.rx_kcap;
    pipe_data->tx_irq = info.tx_irq;
    pipe_data->rx_irq = info.rx_irq;
    pipe_data->max_msg_size = 64;
    pipe_data->ref_count = 0;

    mtx_init(&pipe_data->pipe_lock, mtx_plain);

    // alloc rx buf
    pipe_data->rx_buf = malloc(pipe_data->max_msg_size + sizeof(uint32_t));
    if (pipe_data->rx_buf == NULL) {
        status = ZX_ERR_NO_MEMORY;
        goto rx_buf_alloc_fail;
    }

    pipe_data->tx_buf = malloc(pipe_data->max_msg_size + sizeof(uint32_t));
    if (pipe_data->rx_buf == NULL) {
        status = ZX_ERR_NO_MEMORY;
        goto tx_buf_alloc_fail;
    }

    // create irq objects
    status = zx_interrupt_create(pipe_data->rsrc, pipe_data->tx_irq,
            ZX_INTERRUPT_MODE_EDGE_HIGH, &pipe_data->tx_irq_handle);
    if (status != ZX_OK) {
        goto tx_irq_create_fail;
    }

    status = zx_interrupt_create(pipe_data->rsrc, pipe_data->rx_irq,
            ZX_INTERRUPT_MODE_EDGE_HIGH, &pipe_data->rx_irq_handle);
    if (status != ZX_OK) {
        goto rx_irq_create_fail;
    }

    // create port
    status = zx_port_create(ZX_PORT_BIND_TO_INTERRUPT, &pipe_data->port);
    if (status != ZX_OK) {
        goto port_create_fail;
    }

    // bind irqs to port
    status = zx_interrupt_bind(pipe_data->tx_irq_handle, pipe_data->port,
            HYP_PIPE_TX_IRQ_KEY, 0);
    if (status != ZX_OK) {
        goto tx_irq_bind_fail;
    }

    status = zx_interrupt_bind(pipe_data->rx_irq_handle, pipe_data->port,
            HYP_PIPE_RX_IRQ_KEY, 0);
    if (status != ZX_OK) {
        goto rx_irq_bind_fail;
    }

    device_add_args_t args = {
        .version = DEVICE_ADD_ARGS_VERSION,
        .name = info.name,
        .ctx = pipe_data,
        .proto_id = ZX_PROTOCOL_HYP_PIPE,
        .ops = &hyp_pipe_protocol,
    };

    status = device_add(parent, &args, &pipe_data->dev);
    if (status != ZX_OK) {
        goto device_add_fail;
    }

    // create irq handler thread
    ret = thrd_create(&pipe_data->irq_thrd, irq_handler, pipe_data);
    if (ret != thrd_success) {
        device_remove(pipe_data->dev);
        return ZX_ERR_NO_MEMORY;
    }

    return ZX_OK;

device_add_fail:
    zx_interrupt_destroy(pipe_data->rx_irq_handle);
rx_irq_bind_fail:
    zx_interrupt_destroy(pipe_data->tx_irq_handle);
tx_irq_bind_fail:
    zx_handle_close(pipe_data->port);
port_create_fail:
    zx_handle_close(pipe_data->rx_irq_handle);
rx_irq_create_fail:
    zx_handle_close(pipe_data->tx_irq_handle);
tx_irq_create_fail:
    free(pipe_data->tx_buf);
tx_buf_alloc_fail:
    free(pipe_data->rx_buf);
rx_buf_alloc_fail:
get_metadata_fail:
    free(pipe_data);

    return status;
}

static zx_driver_ops_t hyp_pipe_driver_ops = {
    .version = DRIVER_OPS_VERSION,
    .bind = hyp_pipe_bind,
};

ZIRCON_DRIVER_BEGIN(hyp_pipe, hyp_pipe_driver_ops, "zircon", "0.1", 3)
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_VID, PDEV_VID_OKL4),
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_PID, PDEV_PID_OKL4),
    BI_MATCH_IF(EQ, BIND_PLATFORM_DEV_DID, PDEV_DID_HYP_PIPE),
ZIRCON_DRIVER_END(hyp_pipe)
