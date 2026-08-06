#ifndef BEACON_2BL3_HOST_CM4_DSP_EMULATION_H
#define BEACON_2BL3_HOST_CM4_DSP_EMULATION_H

#include <stdint.h>
#include <string.h>

static unsigned int g_cm4_dsp_emulated_ge;

static inline uint32_t cm4_dsp_unaligned_uint32_read(const void *address)
{
    uint32_t value;

    memcpy(&value, address, sizeof(value));
    return value;
}

static inline void cm4_dsp_unaligned_uint32_write(void *address,
                                                  uint32_t value)
{
    memcpy(address, &value, sizeof(value));
}

static inline uint32_t cm4_dsp_rotate_right(uint32_t value,
                                            uint32_t shift)
{
    shift &= 31U;
    return (shift == 0U) ? value :
           ((value >> shift) | (value << (32U - shift)));
}

static inline uint32_t cm4_dsp_pack_halfword_bottom(uint32_t bottom,
                                                    uint32_t top,
                                                    uint32_t shift)
{
    return (bottom & 0x0000FFFFU) |
           ((top << shift) & 0xFFFF0000U);
}

static inline uint32_t cm4_dsp_pack_halfword_top(uint32_t top,
                                                 uint32_t bottom,
                                                 uint32_t shift)
{
    return (top & 0xFFFF0000U) |
           ((bottom >> shift) & 0x0000FFFFU);
}

static inline uint32_t cm4_dsp_unpack_bytes_to_halfwords(uint32_t value)
{
    return value & 0x00FF00FFU;
}

static inline uint32_t cm4_dsp_add16(uint32_t left, uint32_t right)
{
    uint32_t low = ((left & 0xFFFFU) + (right & 0xFFFFU)) & 0xFFFFU;
    uint32_t high = (((left >> 16U) + (right >> 16U)) & 0xFFFFU) << 16U;

    return low | high;
}

static inline uint32_t cm4_dsp_sub16(uint32_t left, uint32_t right)
{
    uint32_t low = ((left & 0xFFFFU) - (right & 0xFFFFU)) & 0xFFFFU;
    uint32_t high = (((left >> 16U) - (right >> 16U)) & 0xFFFFU) << 16U;

    return low | high;
}

static inline int32_t cm4_dsp_signed_multiply_add_dual(uint32_t left,
                                                       uint32_t right,
                                                       int32_t accumulator)
{
    int32_t left_low = (int16_t)(left & 0xFFFFU);
    int32_t left_high = (int16_t)(left >> 16U);
    int32_t right_low = (int16_t)(right & 0xFFFFU);
    int32_t right_high = (int16_t)(right >> 16U);
    uint32_t bits = (uint32_t)accumulator;
    int32_t result;

    bits += (uint32_t)(left_low * right_low);
    bits += (uint32_t)(left_high * right_high);
    memcpy(&result, &bits, sizeof(result));
    return result;
}

static inline uint32_t cm4_dsp_unsigned_subtract8(uint32_t left,
                                                 uint32_t right)
{
    uint32_t result = 0U;
    uint32_t lane;

    g_cm4_dsp_emulated_ge = 0U;
    for(lane = 0U; lane < 4U; lane++)
    {
        uint32_t shift = lane * 8U;
        uint32_t left_lane = (left >> shift) & 0xFFU;
        uint32_t right_lane = (right >> shift) & 0xFFU;

        result |= ((left_lane - right_lane) & 0xFFU) << shift;
        if(left_lane >= right_lane)
        {
            g_cm4_dsp_emulated_ge |= 1U << lane;
        }
    }
    return result;
}

static inline uint32_t cm4_dsp_select(uint32_t when_ge, uint32_t when_lt)
{
    uint32_t result = 0U;
    uint32_t lane;

    for(lane = 0U; lane < 4U; lane++)
    {
        uint32_t mask = 0xFFU << (lane * 8U);
        result |= ((g_cm4_dsp_emulated_ge & (1U << lane)) != 0U) ?
                  (when_ge & mask) : (when_lt & mask);
    }
    return result;
}

static inline uint32_t cm4_dsp_unsigned_sum_absolute_differences8(
    uint32_t left,
    uint32_t right,
    uint32_t accumulator)
{
    uint32_t lane;

    for(lane = 0U; lane < 4U; lane++)
    {
        uint32_t shift = lane * 8U;
        int32_t difference = (int32_t)((left >> shift) & 0xFFU) -
                             (int32_t)((right >> shift) & 0xFFU);

        accumulator += (uint32_t)((difference < 0) ?
                                  -difference : difference);
    }
    return accumulator;
}

#define __UNALIGNED_UINT32_READ(address) \
    cm4_dsp_unaligned_uint32_read((address))
#define __UNALIGNED_UINT32_WRITE(address, value) \
    cm4_dsp_unaligned_uint32_write((address), (value))
#define __ROR(value, shift) cm4_dsp_rotate_right((value), (shift))
#define __PKHBT(bottom, top, shift) \
    cm4_dsp_pack_halfword_bottom((bottom), (top), (shift))
#define __PKHTB(top, bottom, shift) \
    cm4_dsp_pack_halfword_top((top), (bottom), (shift))
#define __UXTB16(value) cm4_dsp_unpack_bytes_to_halfwords((value))
#define __UADD16(left, right) cm4_dsp_add16((left), (right))
#define __USUB16(left, right) cm4_dsp_sub16((left), (right))
#define __SSUB16(left, right) cm4_dsp_sub16((left), (right))
#define __SMLAD(left, right, accumulator) \
    cm4_dsp_signed_multiply_add_dual((left), (right), (accumulator))
#define __USUB8(left, right) cm4_dsp_unsigned_subtract8((left), (right))
#define __SEL(when_ge, when_lt) cm4_dsp_select((when_ge), (when_lt))
#define __USADA8(left, right, accumulator) \
    cm4_dsp_unsigned_sum_absolute_differences8( \
        (left), (right), (accumulator))

#endif
