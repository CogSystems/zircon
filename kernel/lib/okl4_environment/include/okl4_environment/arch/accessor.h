/* Please see LICENSE.TXT for licensing details. */
/* Copyright (c) 2012-2015 General Dynamics */
/* Copyright (c) 2014 Open Kernel Labs, Inc. */
/* Developed by Cog Systems PTY LTD */

#ifndef __OKL4_ENVIRONMENT_ARCH_ACCESSOR_H__
#define __OKL4_ENVIRONMENT_ARCH_ACCESSOR_H__

#if !defined(ASSEMBLY)

#if defined(OKL4_VCPU_SUPPORT)

#if !defined(OKL4_VCPU_SMP_SUPPORT)

OKL4_INLINE okl4_vcpu_id_t
okl4_get_vcpu_id(void)
{
    return 0;
}

#else

OKL4_INLINE okl4_vcpu_id_t
okl4_get_vcpu_id(void)
{
    okl4_register_t mpidr;

    __asm__ __volatile__ ("mrs %0, mpidr_el1" : "=r" (mpidr) );

    return (okl4_vcpu_id_t)(mpidr & 0xff);
}

#endif /* !OKL4_VCPU_SMP_SUPPORT */

#endif /* OKL4_VCPU_SUPPORT */

#else /* !ASSEMBLY */

#if defined(OKL4_VCPU_SUPPORT)

#if !defined(OKL4_VCPU_SMP_SUPPORT)

.macro _OKL4_ASM_GET_VCPU_ID _reg_
    mov \_reg_, 0
.endm

#else

.macro _OKL4_ASM_GET_VCPU_ID _reg_
    mrs \_reg_, mpidr_el1
    and \_reg_, \_reg_, #0xff
.endm

#endif /* !OKL4_VCPU_SMP_SUPPORT */

#define OKL4_ASM_GET_VCPU_ID(_reg_) _OKL4_ASM_GET_VCPU_ID _reg_

#endif /* OKL4_VCPU_SUPPORT */

#endif /* !ASSEMBLY */

#endif /* __OKL4_ENVIRONMENT_ARCH_ACCESSOR_H__ */
