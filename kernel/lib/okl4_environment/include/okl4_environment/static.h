/* Please see LICENSE.TXT for licensing details. */
/* Copyright (c) 2012-2014 General Dynamics */
/* Copyright (c) 2014 Open Kernel Labs, Inc. */

#ifndef __OKL4_ENVIRONMENT_STATIC_H__
#define __OKL4_ENVIRONMENT_STATIC_H__

/*lint -save -e773 */
#if defined(LINTER)
#define LCONST const
#else
#define LCONST
#define SECTION(x) __attribute__((__section__(x)))
#endif

#define OKL4_WORD_T_BIT (sizeof(okl4_word_t) * 8)

#define OKL4_ENV_GET_GLOBAL(_type, _var_name, _env_key) \
    /*lint -esym(9003, _var_name) supposed to be global */ \
    SECTION(".weaved_env$required$" #_type "$" _env_key) _type *LCONST _var_name

/* FIXME: Redmine issue #1192 - philip. */
#define OKL4_ENV_GET_STATIC(_type, _var_name, _env_key) \
    /*lint -esym(9003, _var_name) supposed to be global */ \
    SECTION(".weaved_env$required$" #_type "$" _env_key) \
    static _type *LCONST volatile _var_name

#define OKL4_ENV_GET_GLOBAL_OPT(_type, _var_name, _env_key) \
    /*lint -esym(9003, _var_name) supposed to be global */ \
    SECTION(".weaved_env$optional$" #_type "$" _env_key) _type *LCONST _var_name

/* FIXME: Redmine issue #1192 - philip. */
#define OKL4_ENV_GET_STATIC_OPT(_type, _var_name, _env_key) \
    /*lint -esym(9003, _var_name) supposed to be global */ \
    SECTION(".weaved_env$optional$" #_type "$" _env_key) \
    static _type *LCONST volatile _var_name

#define OKL4_ENV_GET_EXTERN(_type, _var_name) \
    /*lint -esym(9003, _var_name) supposed to be global */ \
    extern _type *LCONST _var_name

/* FIXME: Redmine issue #1192 - philip. */
#define OKL4_CREATE_WEAVED_STATIC(_type, _var_name, _attr, _env_key) \
    /*lint -esym(9003, _var_name) supposed to be global */ \
    SECTION(".weaved_env_create$required$" #_type "$" _env_key "$" _attr) \
            static _type *LCONST volatile _var_name

#define OKL4_CREATE_WEAVED_GLOBAL(_type, _var_name, _attr, _env_key) \
    /*lint -esym(9003, _var_name) supposed to be global */ \
    SECTION(".weaved_env_create$required$" #_type "$" _env_key "$" _attr) \
            _type *LCONST _var_name

#define OKL4_CREATE_WEAVED_EXTERN(_type, _var_name) \
    extern _type *LCONST _var_name

#if defined(OKL4_THREAD_SUPPORT)

/* FIXME: Redmine issue #1192 - philip. */
#define OKL4_ENV_TLS_SLOT(_env_key) \
    SECTION(".weaved_env_create$required$okl4_register_t$" _env_key         \
            "_TLS_SLOT$TLS_SLOT") static okl4_register_t *volatile          \
            __okl4_lib_tls_id;                                              \
                                                                            \
    static inline okl4_register_t                                           \
    okl4_tls_get(void)                                                      \
    {                                                                       \
        extern okl4_tls_block_t *__okl4_tls_block;                          \
        return __okl4_tls_block->threads[okl4_get_thread_id()]->            \
                lib_slots[*__okl4_lib_tls_id];                              \
    }                                                                       \
                                                                            \
    static inline void                                                      \
    okl4_tls_set(okl4_register_t val)                                       \
    {                                                                       \
        extern okl4_tls_block_t *__okl4_tls_block;                          \
        __okl4_tls_block->threads[okl4_get_thread_id()]->                   \
                lib_slots[*__okl4_lib_tls_id] = val;                        \
    }

#endif /* OKL4_THREAD_SUPPORT */

/*lint -restore */

#endif /* __OKL4_ENVIRONMENT_STATIC_H__ */
