/* Please see LICENSE.TXT for licensing details. */
/* Copyright (c) 2012-2014 General Dynamics */
/* Copyright (c) 2014 Open Kernel Labs, Inc. */

#ifndef __OKL4_ENVIRONMENT_ACCESSOR_H__
#define __OKL4_ENVIRONMENT_ACCESSOR_H__

#if !defined(ASSEMBLY)

#if defined(__HOST_AARCH64) && __SIZEOF_POINTER__ != 8
#define OKL4_ENV_PTRARRAY_GET(var, index) var[index].val
#else
#define OKL4_ENV_PTRARRAY_GET(var, index) var[index]
#endif

OKL4_INLINE okl4_laddr_t
okl4_virtmem_item_getbase(okl4_virtmem_item_t *item)
{
    return item->range.base;
}

OKL4_INLINE okl4_laddr_t
okl4_virtmem_item_getsize(okl4_virtmem_item_t *item)
{
    return item->range.size;
}

#if defined(OKL4_VCPU_SUPPORT)

OKL4_INLINE okl4_vcpu_id_t
okl4_get_vcpu_id(void);

OKL4_INLINE okl4_kcap_t
okl4_kmmu_getkcap(okl4_kmmu_t *kmmu)
{
    return kmmu->kcap;
}

OKL4_INLINE okl4_kcap_t
okl4_get_my_vcpu_kcap(okl4_vcpu_table_t *table)
{
    return table->vcpu[okl4_get_vcpu_id()].vcpu;
}

OKL4_INLINE okl4_kcap_t
okl4_get_vcpu_ipi(okl4_vcpu_table_t *table, okl4_vcpu_id_t core)
{
    return table->vcpu[core].ipi;
}

OKL4_INLINE okl4_kcap_t
okl4_get_my_vcpu_ipi(okl4_vcpu_table_t *table)
{
    return table->vcpu[okl4_get_vcpu_id()].irq;
}

OKL4_INLINE okl4_register_t
okl4_get_my_vcpu_stack_pointer(okl4_vcpu_table_t *table)
{
    return table->vcpu[okl4_get_vcpu_id()].stack_pointer;
}

OKL4_INLINE okl4_count_t
okl4_get_num_vcpus(okl4_vcpu_table_t *table)
{
    return table->num_vcpus;
}

#endif /* OKL4_VCPU_SUPPORT */

#endif /* !ASSEMBLY */

#include <okl4_environment/arch/accessor.h>

#endif /* __OKL4_ENVIRONMENT_ACCESSOR_H__ */
