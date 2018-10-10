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

#if ARCH_ARM64

#define STR(x) #x

#define DO_HYP_SYSCALL(syscall_num, args) \
    __asm__ __volatile__( \
        "ldp x0, x1, [%0] \n\t" \
        "ldp x2, x3, [%0, 16] \n\t" \
        "ldp x4, x5, [%0, 32] \n\t" \
        "ldp x6, x7, [%0, 48] \n\t" \
        "hvc " STR(syscall_num) "\n\t" \
        "stp x0, x1, [%0] \n\t" \
        "stp x2, x3, [%0, 16] \n\t" \
        "stp x4, x5, [%0, 32] \n\t" \
        "stp x6, x7, [%0, 48] \n\t" \
        : \
        : "r" (&args) \
        : "x0", "x1", "x2", "x3", "x4", \
          "x5", "x6", "x7", "memory" \
    )

static bool is_user_buffer(uint64_t addr, uint64_t size) {
    // Overflow check
    if ((addr + size) < addr) {
        return false;
    }

    // Ensure buffer lies in user address space
    if ((addr < USER_ASPACE_BASE) ||
            ((addr + size) > (USER_ASPACE_BASE + USER_ASPACE_SIZE))) {
        return false;
    }

    return true;
}

#endif // ARCH_ARM64

zx_status_t sys_hyp_syscall(zx_handle_t handle,
                            uint16_t syscall_num,
                            user_inout_ptr<zx_hyp_sys_args_t> user_args) {
#if ARCH_ARM64
    zx_status_t status;
    zx_hyp_sys_args_t args;

    // TODO add VIRT resource
    status = validate_resource(handle, ZX_RSRC_KIND_ROOT);
    if (status != ZX_OK) {
        return status;
    }

    status = user_args.copy_from_user(&args);
    if (status != ZX_OK) {
        return status;
    }

    switch (syscall_num) {
    case HYP_SYSCALL_INTERRUPT_GET_PAYLOAD:
        DO_HYP_SYSCALL(HYP_SYSCALL_INTERRUPT_GET_PAYLOAD, args);
        break;
    case HYP_SYSCALL_PIPE_CONTROL:
        DO_HYP_SYSCALL(HYP_SYSCALL_PIPE_CONTROL, args);
        break;
    case HYP_SYSCALL_PIPE_RECV:
        if (!is_user_buffer(args.x2, args.x1)) {
            return ZX_ERR_INVALID_ARGS;
        }
        DO_HYP_SYSCALL(HYP_SYSCALL_PIPE_RECV, args);
        break;
    case HYP_SYSCALL_PIPE_SEND:
        if (!is_user_buffer(args.x2, args.x1)) {
            return ZX_ERR_INVALID_ARGS;
        }
        DO_HYP_SYSCALL(HYP_SYSCALL_PIPE_SEND, args);
        break;
    case HYP_SYSCALL_VINTERRUPT_CLEAR_AND_RAISE:
        DO_HYP_SYSCALL(HYP_SYSCALL_VINTERRUPT_CLEAR_AND_RAISE, args);
        break;
    case HYP_SYSCALL_VINTERRUPT_MODIFY:
        DO_HYP_SYSCALL(HYP_SYSCALL_VINTERRUPT_MODIFY, args);
        break;
    case HYP_SYSCALL_VINTERRUPT_RAISE:
        DO_HYP_SYSCALL(HYP_SYSCALL_VINTERRUPT_RAISE, args);
        break;
    default:
        return ZX_ERR_NOT_SUPPORTED;
    }

    return user_args.copy_to_user(args);
#else
    return ZX_ERR_NOT_SUPPORTED;
#endif // ARCH_ARM64
}
