#ifndef CM4_DSP_EMULATION_H_
#define CM4_DSP_EMULATION_H_

/*
 * Desktop-only semantic verifier for the Cortex-M4 DSP data path.
 * It emulates the CMSIS intrinsics used by image.c; production firmware
 * continues to use the real single-cycle instructions supplied by CMSIS.
 */
static unsigned int g_cm4_dsp_emulated_ge;

static unsigned int cm4_dsp_emulate_usada8(
    unsigned int left,
    unsigned int right,
    unsigned int accumulator)
{
    int lane;

    for(lane = 0; lane < 4; lane++)
    {
        unsigned int shift = (unsigned int)lane * 8U;
        int difference = (int)((left >> shift) & 0xFFU) -
                         (int)((right >> shift) & 0xFFU);

        accumulator += (unsigned int)((difference < 0) ?
                                      -difference : difference);
    }
    return accumulator;
}

static unsigned int cm4_dsp_emulate_usub8(
    unsigned int left,
    unsigned int right)
{
    unsigned int result = 0U;
    int lane;

    g_cm4_dsp_emulated_ge = 0U;
    for(lane = 0; lane < 4; lane++)
    {
        unsigned int shift = (unsigned int)lane * 8U;
        unsigned int lhs = (left >> shift) & 0xFFU;
        unsigned int rhs = (right >> shift) & 0xFFU;

        result |= ((lhs - rhs) & 0xFFU) << shift;
        if(lhs >= rhs)
        {
            g_cm4_dsp_emulated_ge |= 1U << lane;
        }
    }
    return result;
}

static unsigned int cm4_dsp_emulate_sel(
    unsigned int when_ge,
    unsigned int when_lt)
{
    unsigned int result = 0U;
    int lane;

    for(lane = 0; lane < 4; lane++)
    {
        unsigned int shift = (unsigned int)lane * 8U;
        unsigned int source =
            ((g_cm4_dsp_emulated_ge & (1U << lane)) != 0U) ?
                when_ge : when_lt;

        result |= source & (0xFFU << shift);
    }
    return result;
}

static unsigned int cm4_dsp_emulate_add16(
    unsigned int left,
    unsigned int right)
{
    unsigned int low = (unsigned short)(
        (signed short)(left & 0xFFFFU) +
        (signed short)(right & 0xFFFFU));
    unsigned int high = (unsigned short)(
        (signed short)(left >> 16) +
        (signed short)(right >> 16));

    return low | (high << 16);
}

static unsigned int cm4_dsp_emulate_sub16(
    unsigned int left,
    unsigned int right)
{
    unsigned int low = (unsigned short)(
        (signed short)(left & 0xFFFFU) -
        (signed short)(right & 0xFFFFU));
    unsigned int high = (unsigned short)(
        (signed short)(left >> 16) -
        (signed short)(right >> 16));

    return low | (high << 16);
}

#define __USADA8(left, right, accumulator) \
    cm4_dsp_emulate_usada8((left), (right), (accumulator))
#define __USUB8(left, right) cm4_dsp_emulate_usub8((left), (right))
#define __SEL(when_ge, when_lt) cm4_dsp_emulate_sel((when_ge), (when_lt))
#define __SADD16(left, right) cm4_dsp_emulate_add16((left), (right))
#define __SSUB16(left, right) cm4_dsp_emulate_sub16((left), (right))

#endif
