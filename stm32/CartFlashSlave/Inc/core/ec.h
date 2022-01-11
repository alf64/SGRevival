/*
 * core_ec.h
 *
 * File contains error codes and useful defines and typedefs.
 *
 *  Created on: 18 Oct 2021
 *      Author: alf64
 */

#ifndef CORE_EC_H_
#define CORE_EC_H_

#include <stdint.h>

//!< returns the lower of the two variables
#ifndef MIN
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif
//!< returns the higher of the two variables
#ifndef MAX
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#endif

//!< returns 1 if addr is aligned to alignment, otherwise returns 0
#define IS_ALIGNED(addr, alignment) (((uintptr_t)(addr)) % (alignment) == 0)
//!< Use to avoid gcc/g++ warnings when you don't use a variable
#define UNUSED(X) (void)(X)

//!< rounds up given val to the nearest multiplication of roundup_val
#ifndef ROUND_UP
#define ROUND_UP(val, roundup_val)                                              \
    (((val) % (roundup_val) == 0)                                               \
         ? (val)                                                              \
         : ((val) - ((val) % (roundup_val)) + (roundup_val)))
#endif
//!< rounds down given val to the nearest multiplication of rounddown_val
#ifndef ROUND_DOWN
#define ROUND_DOWN(val, rounddown_val) ((val) - ((val) % rounddown_val)))
#endif

//!< Makes data aligned to given alignment (x).
#if !defined(__aligned) && !defined(ALIGNED)
#define __aligned(x) __attribute__((__aligned__(x)))
#define ALIGNED(x) __attribute__((__aligned__(x)))
#endif

//!< Useful for making data structures packed.
#ifndef PACKED
#define PACKED __attribute__((__packed__))
#endif

//!< Forces inline attribute.
#define FORCE_INLINE __attribute__((always_inline))
//!< Forces no-inline attribute.
#define FORCE_NO_INLINE __attribute__((noinline))
//!< Disables optimization.
#define NO_OPTIMIZE __attribute__((optimize("-O0")))
//!< Forces the code to be in a specific section (given by sct_name)
#define SECTION(sct_name) __attribute__((section(sct_name)))
#define RODATA_SECTION SECTION(".rodata")

// concatenates two defs
#ifndef CONCATENATE
#define CONCS(a, b) a##b
#define CONCATENATE(a, b) CONCS(a, b)
#endif

#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *) 0)
#endif
#endif


//!< Error codes for core stm32 functionality.
typedef enum
{
    CORE_EC_SUCCESS = 0,
    CORE_EC_FAILED = 1
} core_ec_t;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

typedef uint8_t bool;

#define HALT_ON_ERROR(ec)                   \
    if ((ec) != CORE_EC_SUCCESS)            \
    {                                       \
        while(1){};                         \
    }

#define RETURN_EC_ON_ERROR(ec)              \
    if ((ec) != CORE_EC_SUCCESS)            \
    {                                       \
        return (ec);                        \
    }

#define RETURN_VOID_ON_ERROR(ec)            \
    if ((ec) != CORE_EC_SUCCESS)            \
    {                                       \
        return;                             \
    }

#define BREAK_ON_ERROR(ec)                  \
    if ((ec) != CORE_EC_SUCCESS)            \
    {                                       \
        break;                              \
    }

#define RETURN_VOID_ON_FAIL(condition)      \
    if (!(condition))                       \
    {                                       \
        return;                             \
    }

#define RETURN_VAL_ON_FAIL(condition, val)  \
    if (!(condition))                       \
    {                                       \
        return val;                         \
    }

#define RETURN_NULL_ON_FAIL(condition, val)     \
    if (!(condition))                           \
    {                                           \
        return NULL;                             \
    }


#endif /* CORE_EC_H_ */
