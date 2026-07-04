#include "beacon_image.h"

#include <string.h>

#define BEACON_ANALYZER_ADAPTER
#define beacon_circle_t image_beacon_circle_t
#define beacon_rect_t image_beacon_rect_t
#define beacon_result_t image_internal_result_t
#define beacon_image_init embedded_beacon_image_init
#define beacon_image_process embedded_beacon_image_process
#include "image.c"
#undef beacon_image_process
#undef beacon_image_init
#undef beacon_result_t
#undef beacon_rect_t
#undef beacon_circle_t

void beacon_image_init(void)
{
    embedded_beacon_image_init();
}

void beacon_image_process(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result)
{
    image_internal_result_t internal_result;
    unsigned char i;

    if (result == 0)
    {
        return;
    }

    memset(result, 0, sizeof(*result));
    if (image == 0)
    {
        return;
    }

    memset(&internal_result, 0, sizeof(internal_result));
    embedded_beacon_image_process(image, &internal_result);

    result->count = internal_result.count;
    if (result->count > BEACON_MAX_CIRCLE_COUNT)
    {
        result->count = BEACON_MAX_CIRCLE_COUNT;
    }
    for (i = 0; i < result->count; i++)
    {
        result->circles[i].x = -internal_result.circles[i].x;
        result->circles[i].y = internal_result.circles[i].y;
        result->circles[i].radius = internal_result.circles[i].radius;
        result->circles[i].valid = internal_result.circles[i].valid;
    }

    result->beacon_count = internal_result.beacon_count;
    if (result->beacon_count > BEACON_MAX_BEACON_COUNT)
    {
        result->beacon_count = BEACON_MAX_BEACON_COUNT;
    }
    for (i = 0; i < result->beacon_count; i++)
    {
        result->beacons[i].x = -internal_result.beacons[i].x;
        result->beacons[i].y = internal_result.beacons[i].y;
        result->beacons[i].radius = internal_result.beacons[i].radius;
        result->beacons[i].valid = internal_result.beacons[i].valid;
    }

    result->car_lamp_count = internal_result.car_lamp_count;
    if (result->car_lamp_count > 2)
    {
        result->car_lamp_count = 2;
    }
    for (i = 0; i < result->car_lamp_count; i++)
    {
        result->car_lamps[i].cx = -internal_result.car_lamps[i].cx;
        result->car_lamps[i].cy = internal_result.car_lamps[i].cy;
        result->car_lamps[i].width = internal_result.car_lamps[i].width;
        result->car_lamps[i].length = internal_result.car_lamps[i].length;
        result->car_lamps[i].angle = internal_result.car_lamps[i].angle;
        result->car_lamps[i].valid = internal_result.car_lamps[i].valid;
    }
}

void beacon_image_debug_binary(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char binary[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    int i;

    if (binary == 0)
    {
        return;
    }

    if (image == 0)
    {
        memset(binary, 0, BEACON_IMAGE_W * BEACON_IMAGE_H);
        return;
    }

    for (i = 0; i < BEACON_IMAGE_W * BEACON_IMAGE_H; i++)
    {
        (&binary[0][0])[i] = ((&image[0][0])[i] >= 200U) ? 255U : 0U;
    }
}
