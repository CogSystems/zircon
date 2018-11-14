// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Copyright 2018 Cog Systems Pty Ltd. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <stdint.h>
#include "debug.h"

// uncomment to enable boot shim debug uart on hikey 6220 board
// #define HIKEY_6220

#if DEBUG_UART && HIKEY_6220
static volatile uint32_t* uart_fifo_dr = (uint32_t *)0xf7113000;
static volatile uint32_t* uart_fifo_fr = (uint32_t *)0xf7113018;

void uart_pputc(char c)
{
    if (c == '\n')
        uart_pputc('\r');
    /* spin while fifo is full */
    while (*uart_fifo_fr & (1<<5))
        ;
    *uart_fifo_dr = c;
}
#else
void uart_pputc(char c)
{
}
#endif
