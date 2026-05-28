/*
 * Stub algorithm for distribution builds.
 *
 * This file provides an empty implementation of the beacon detection
 * interface. When linked, the application can browse videos but will
 * not produce any detection results. Users who want beacon detection
 * must write their own implementation of beacon_image_process().
 *
 * See docs/beacon_algorithm_api.md for the full interface specification.
 */

#include "beacon_image.h"
#include <string.h>

void beacon_image_init(void)
{
    /* No state to initialize in stub mode. */
}

void beacon_image_process(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result)
{
    if (result)
    {
        memset(result, 0, sizeof(*result));
    }
    (void)image;
}

unsigned char beacon_image_debug_threshold(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    (void)image;
    return 200;
}

void beacon_image_debug_binary(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char binary[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    if (binary)
    {
        memset(binary, 0, BEACON_IMAGE_H * BEACON_IMAGE_W);
    }
    (void)image;
}
