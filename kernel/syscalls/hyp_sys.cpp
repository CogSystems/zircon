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

#include <stdint.h>

#include <lib/user_copy/user_ptr.h>
#include <object/resource.h>
#include <zircon/syscalls/hyp_sys.h>
#include <vm/vm.h>

#if ARCH_ARM64

#include <microvisor/microvisor.h>

zx_status_t sys_hyp_virq_get_payload(zx_handle_t rsrc, uint32_t virq,
                                     user_out_ptr<uint64_t> user_payload) {
    zx_status_t status;
    struct _okl4_sys_interrupt_get_payload_return ret;

    status = validate_resource(rsrc, ZX_RSRC_KIND_ROOT);
    if (status != ZX_OK) {
        return status;
    }

    ret = _okl4_sys_interrupt_get_payload(virq);
    if (ret.error != OKL4_OK) {
        return ZX_ERR_INVALID_ARGS;
    }

    return user_payload.copy_to_user(ret.payload);
}

zx_status_t sys_hyp_virq_raise(zx_handle_t rsrc, uint32_t virqline,
                               uint64_t payload)
{
    zx_status_t status;
    okl4_error_t err;

    status = validate_resource(rsrc, ZX_RSRC_KIND_ROOT);
    if (status != ZX_OK) {
        return status;
    }

    err = _okl4_sys_vinterrupt_raise(virqline, payload);
    if (err != OKL4_OK) {
        return ZX_ERR_INVALID_ARGS;
    }

    return ZX_OK;
}

zx_status_t sys_hyp_pipe_control(zx_handle_t rsrc, uint32_t pipe, uint8_t op) {
    zx_status_t status;
    okl4_error_t err;
    okl4_pipe_control_t control = 0;

    status = validate_resource(rsrc, ZX_RSRC_KIND_ROOT);
    if (status != ZX_OK) {
        return status;
    }

    okl4_pipe_control_setdoop(&control, true);

    switch (op) {
        case ZX_HYP_PIPE_RESET:
            okl4_pipe_control_setoperation(&control,
                    OKL4_PIPE_CONTROL_OP_RESET);
            break;
        case ZX_HYP_PIPE_RX_READY:
            okl4_pipe_control_setoperation(&control,
                    OKL4_PIPE_CONTROL_OP_SET_RX_READY);
            break;
        case ZX_HYP_PIPE_TX_READY:
            okl4_pipe_control_setoperation(&control,
                    OKL4_PIPE_CONTROL_OP_SET_TX_READY);
            break;
        default:
            return ZX_ERR_INVALID_ARGS;
    }

    err = _okl4_sys_pipe_control(pipe, control);
    if (err != OKL4_OK) {
        return ZX_ERR_INVALID_ARGS;
    }

    return ZX_OK;
}

zx_status_t sys_hyp_pipe_send(zx_handle_t rsrc, uint32_t pipe,
                              user_in_ptr<const void> user_buf, size_t len) {
    zx_status_t status;
    const uint8_t *buf = static_cast<const uint8_t *>(user_buf.get());
    okl4_error_t err;

    status = validate_resource(rsrc, ZX_RSRC_KIND_ROOT);
    if (status != ZX_OK) {
        return status;
    }

    if ((len % sizeof(uint32_t)) != 0) {
        return ZX_ERR_INVALID_ARGS;
    }

    if (!is_user_address_range((vaddr_t)buf, len)) {
        return ZX_ERR_INVALID_ARGS;
    }

    err = _okl4_sys_pipe_send(pipe, len, buf);
    if (err != OKL4_OK) {
        return ZX_ERR_INVALID_ARGS;
    }

    return ZX_OK;
}

zx_status_t sys_hyp_pipe_recv(zx_handle_t rsrc, uint32_t pipe,
                              user_out_ptr<void> user_buf, size_t len,
                              user_out_ptr<size_t> ret_size) {
    zx_status_t status;
    uint8_t *buf = static_cast<uint8_t *>(user_buf.get());
    struct _okl4_sys_pipe_recv_return ret;

    status = validate_resource(rsrc, ZX_RSRC_KIND_ROOT);
    if (status != ZX_OK) {
        return status;
    }

    if ((len % sizeof(uint32_t)) != 0) {
        return ZX_ERR_INVALID_ARGS;
    }

    if (!is_user_address_range((vaddr_t)buf, len)) {
        return ZX_ERR_INVALID_ARGS;
    }

    ret = _okl4_sys_pipe_recv(pipe, len, buf);
    if (ret.error != OKL4_OK) {
        return ZX_ERR_INVALID_ARGS;
    }

    return ret_size.copy_to_user(ret.size);
}

#else

zx_status_t sys_hyp_virq_get_payload(zx_handle_t rsrc, uint32_t virq,
                                     user_out_ptr<uint64_t> user_payload) {
    return ZX_ERR_NOT_SUPPORTED;
}

zx_status_t sys_hyp_virq_raise(zx_handle_t rsrc, uint32_t virqline,
                               uint64_t payload) {
    return ZX_ERR_NOT_SUPPORTED;
}

zx_status_t sys_hyp_pipe_control(zx_handle_t rsrc, uint32_t pipe, uint8_t op) {
    return ZX_ERR_NOT_SUPPORTED;
}

zx_status_t sys_hyp_pipe_send(zx_handle_t rsrc, uint32_t pipe,
                              user_in_ptr<const void> user_buf, uint32_t len) {
    return ZX_ERR_NOT_SUPPORTED;
}

zx_status_t sys_hyp_pipe_recv(zx_handle_t rsrc, uint32_t pipe,
                              user_out_ptr<void> user_buf, uint32_t len) {
    return ZX_ERR_NOT_SUPPORTED;
}
#endif // ARCH_ARM64
