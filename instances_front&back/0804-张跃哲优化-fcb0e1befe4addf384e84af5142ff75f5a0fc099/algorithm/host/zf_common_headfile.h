#ifndef BEACON_2BL3_HOST_COMMON_HEADFILE_H
#define BEACON_2BL3_HOST_COMMON_HEADFILE_H

#include <stdint.h>
#include <time.h>

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int32_t int32;

#include "cm4_dsp_emulation.h"

#define TC_TIME2_CH0 (0U)

static inline uint32 timer_get(uint32 channel)
{
    double microseconds;

    (void)channel;
    microseconds = (double)clock() * 1000000.0 / (double)CLOCKS_PER_SEC;
    return (uint32)microseconds;
}

#endif
