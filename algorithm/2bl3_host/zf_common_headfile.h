#ifndef BEACON_2BL3_HOST_COMMON_HEADFILE_H
#define BEACON_2BL3_HOST_COMMON_HEADFILE_H

#include <stdint.h>

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int32_t int32;

typedef struct
{
    volatile uint32 CTRL;
    volatile uint32 CYCCNT;
} beacon_host_dwt_t;

typedef struct
{
    volatile uint32 DEMCR;
} beacon_host_core_debug_t;

extern beacon_host_dwt_t *DWT;
extern beacon_host_core_debug_t *CoreDebug;

#define DWT_CTRL_CYCCNTENA_Msk       (1UL << 0U)
#define CoreDebug_DEMCR_TRCENA_Msk   (1UL << 24U)

static inline uint32 interrupt_global_disable(void)
{
    return 0U;
}

static inline void interrupt_global_enable(uint32 state)
{
    (void)state;
}

#endif
