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

#include <reg.h>
#include <stdio.h>
#include <trace.h>
#include <lib/cbuf.h>
#include <kernel/thread.h>
#include <dev/interrupt.h>
#include <dev/uart.h>
#include <platform/debug.h>
#include <pdev/driver.h>
#include <pdev/uart.h>
#include <zircon/boot/driver-config.h>

#include <microvisor/microvisor.h>

#define RX_BUF_SIZE 128
#define DELAY_TIME  200000
#define MAX_PIPE_PAYLOAD 32

static uint32_t tx_irq = 0;
static uint32_t rx_irq = 0;

static uint32_t tx_kcap = 0;
static uint32_t rx_kcap = 0;

static cbuf_t rx_buf;

static bool uart_tx_irq_enabled = false;

/* Events used for waiting when pipes full */
static event_t uart_dputs_event = EVENT_INITIAL_VALUE(uart_dputs_event,
                                                      true,
                                                      EVENT_FLAG_AUTOUNSIGNAL);

static event_t uart_getc_event = EVENT_INITIAL_VALUE(uart_getc_event,
                                                     true,
                                                     EVENT_FLAG_AUTOUNSIGNAL);


static spin_lock_t uart_spinlock = SPIN_LOCK_INITIAL_VALUE;

static void
delay(void)
{
    volatile uint32_t i = DELAY_TIME;
    do { i--; } while (i > 0);
}

static void
tx_irq_isr(void *arg)
{
    struct _okl4_sys_interrupt_get_payload_return ret =
            _okl4_sys_interrupt_get_payload(tx_irq);
    okl4_pipe_state_t payload = ret.payload;

    if (okl4_pipe_state_gettxavailable(&payload)) {
        spin_lock(&uart_spinlock);
        event_signal(&uart_dputs_event, true);
        spin_unlock(&uart_spinlock);
    }
}

static void
rx_irq_isr(void *arg)
{
    struct _okl4_sys_interrupt_get_payload_return ret =
            _okl4_sys_interrupt_get_payload(rx_irq);
    okl4_pipe_state_t payload = ret.payload;

    if (okl4_pipe_state_getrxavailable(&payload)) {
        spin_lock(&uart_spinlock);
        event_signal(&uart_getc_event, true);
        spin_unlock(&uart_spinlock);
    }
}

static void
pipe_control(uint32_t kcap, uint8_t control)
{
    okl4_pipe_control_t x = 0;
    okl4_pipe_control_setdoop(&x, true);
    okl4_pipe_control_setoperation(&x, control);
    (void)_okl4_sys_pipe_control(kcap, x);
}

static bool
pipe_recv(void)
{
    struct _okl4_sys_pipe_recv_return ret_recv;
    uint32_t payload[(MAX_PIPE_PAYLOAD / sizeof(uint32_t)) + 1];
    uint32_t bytes;
    const char *str;

    ret_recv = _okl4_sys_pipe_recv(rx_kcap, sizeof(payload),
            (void *)payload);

    if (ret_recv.error == OKL4_ERROR_PIPE_NOT_READY) {
        pipe_control(rx_kcap, OKL4_PIPE_CONTROL_OP_SET_RX_READY);
        ret_recv = _okl4_sys_pipe_recv(rx_kcap, sizeof(payload),
                (void *)payload);
    }

    if (ret_recv.error != OKL4_OK) {
        return false;
    }
    if (ret_recv.size <= sizeof(uint32_t)) {
        return false;
    }
    bytes = payload[0];
    if (bytes > (ret_recv.size - sizeof(uint32_t))) {
        return false;
    }

    if (bytes > RX_BUF_SIZE) {
        panic("hyp vtty: pipe recv payload bigger than rx buf\n");
    }

    str = (const char *)&payload[1];

    while (bytes > 0) {
        cbuf_write_char(&rx_buf, *str++);
        bytes--;
    }

    return true;
}

static int
hyp_vtty_read_char(bool wait)
{
    char c;
    spin_lock_saved_state_t state;

    /* Get next available char in buffer */
    if (cbuf_read_char(&rx_buf, &c, false) == 1) {
        return c;
    }

    /* Buffer empty, refill from pipe */
    spin_lock_irqsave(&uart_spinlock, state);
    while (!pipe_recv() && wait) {
        spin_unlock_irqrestore(&uart_spinlock, state);
        event_wait(&uart_getc_event);
        spin_lock_irqsave(&uart_spinlock, state);
    }
    spin_unlock_irqrestore(&uart_spinlock, state);

    return (cbuf_read_char(&rx_buf, &c, false) == 1) ? c : -1;
}

static int
hyp_vtty_pgetc(void)
{
    return hyp_vtty_read_char(false);
}

static int
hyp_vtty_getc(bool wait)
{
    return hyp_vtty_read_char(wait);
}

static int
hyp_vtty_pputc(char c)
{
    okl4_error_t ret;
    uint32_t payload[2];
    char *p = (char *)&payload[1];

    if (c != '\n') {
        payload[0] = 1;
    } else {
        payload[0] = 2;
        *p++ = '\r';
    }
    *p = c;

pputc_again:
    ret = _okl4_sys_pipe_send(tx_kcap,
            (2 * sizeof(uint32_t)), (void *)payload);
    if (ret == OKL4_ERROR_PIPE_NOT_READY) {
        pipe_control(tx_kcap, OKL4_PIPE_CONTROL_OP_SET_TX_READY);
        goto pputc_again;
    } else if (ret == OKL4_ERROR_PIPE_FULL) {
        /* Cannot use event, so retry after a delay */
        delay();
        goto pputc_again;
    }

    return (ret == OKL4_OK) ? 1 : -1;
}

static void
hyp_vtty_dputs(const char* str, size_t len, bool block, bool map_NL)
{
    spin_lock_saved_state_t state;
    uint32_t payload[(MAX_PIPE_PAYLOAD / sizeof(uint32_t)) + 1];

    if (!uart_tx_irq_enabled) {
        block = false;
    }

    while (len > 0) {
        uint32_t bytes = 0, size;
        char *c = (char *)&payload[1];
        okl4_error_t ret;

        while ((len > 0) && (bytes < MAX_PIPE_PAYLOAD)) {
            if (map_NL && *str == '\n') {
                if (bytes == MAX_PIPE_PAYLOAD - 1)
                    break;
                *c++ = '\r';
                bytes++;
            }
            *c++ = *str++;
            bytes++;
            len--;
        }

        payload[0] = bytes;
        size = sizeof(uint32_t) + ((bytes + 3) & ~3);

        spin_lock_irqsave(&uart_spinlock, state);
dputs_again:
        ret = _okl4_sys_pipe_send(tx_kcap, size, (void *)payload);
        if (ret == OKL4_ERROR_PIPE_FULL) {
            spin_unlock_irqrestore(&uart_spinlock, state);
            if (block) {
                event_wait(&uart_dputs_event);
            } else {
                arch_spinloop_pause();
            }
            spin_lock_irqsave(&uart_spinlock, state);
            goto dputs_again;
        }
        if (ret == OKL4_ERROR_PIPE_NOT_READY) {
            pipe_control(tx_kcap, OKL4_PIPE_CONTROL_OP_SET_TX_READY);
            goto dputs_again;
        }
        spin_unlock_irqrestore(&uart_spinlock, state);
        if (ret != OKL4_OK) {
            panic("hyp vtty: dputs failed!\n");
        }
    }
}

static void
hyp_vtty_start_panic(void)
{
    uart_tx_irq_enabled = false;
}

static const struct pdev_uart_ops uart_ops = {
    .getc = hyp_vtty_getc,
    .pputc = hyp_vtty_pputc,
    .pgetc = hyp_vtty_pgetc,
    .start_panic = hyp_vtty_start_panic,
    .dputs = hyp_vtty_dputs,
};

static void
hyp_vtty_init(const void* driver_data, uint32_t length)
{
    zx_status_t status;

    cbuf_initialize(&rx_buf, RX_BUF_SIZE);

    /* set irq handlers */
    status = register_int_handler(tx_irq, &tx_irq_isr, NULL);
    DEBUG_ASSERT(status == ZX_OK);
    status = register_int_handler(rx_irq, &rx_irq_isr, NULL);
    DEBUG_ASSERT(status == ZX_OK);

    /* ready pipes */
    pipe_control(tx_kcap, OKL4_PIPE_CONTROL_OP_SET_TX_READY);
    pipe_control(rx_kcap, OKL4_PIPE_CONTROL_OP_SET_RX_READY);

    /* enable irqs */
    unmask_interrupt(tx_irq);
    unmask_interrupt(rx_irq);

#if ENABLE_KERNEL_LL_DEBUG
    uart_tx_irq_enabled = false;
#else
    /* start up tx driven output */
    printf("VTTY: started IRQ driven TX\n");
    uart_tx_irq_enabled = true;
#endif
}

static void
hyp_vtty_init_early(const void* driver_data, uint32_t length)
{
    const dcfg_hyp_vtty_driver_t *driver = driver_data;

    ASSERT(length >= sizeof(dcfg_hyp_vtty_driver_t));

    tx_irq = driver->tx_irq;
    rx_irq = driver->rx_irq;
    tx_kcap = driver->tx_kcap;
    rx_kcap = driver->rx_kcap;

    if ((tx_irq == 0) || (rx_irq == 0)) {
        panic("hyp vtty: failed to get irqs\n");
    }
    if ((tx_kcap == 0) || (rx_kcap == 0)) {
        panic("hyp vtty: failed to get kcaps\n");
    }

    pdev_register_uart(&uart_ops);
}

LK_PDEV_INIT(hyp_vtty_init_early, KDRV_HYP_VTTY, hyp_vtty_init_early, LK_INIT_LEVEL_PLATFORM_EARLY);
LK_PDEV_INIT(hyp_vtty_init, KDRV_HYP_VTTY, hyp_vtty_init, LK_INIT_LEVEL_PLATFORM);
