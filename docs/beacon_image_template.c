/*
 * Beacon Algorithm Template
 *
 * This file implements the minimum required interface for
 * BeaconImageAnalyzer. Copy this file and modify beacon_image_process()
 * to implement your own beacon detection algorithm.
 *
 * Compile test:
 *   gcc -shared -O2 -std=c11 -I. -o test.dll beacon_image_template.c -lm
 *
 * See beacon_algorithm_api.md for the full interface specification.
 */

#include "beacon_image.h"
#include <string.h>

/* Initialize your algorithm state here. Called once on load. */
void beacon_image_init(void)
{
    /* Add your initialization code here */
}

/* Main processing function. Called once per frame. */
void beacon_image_process(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result)
{
    /* Clear the result */
    memset(result, 0, sizeof(*result));

    /*
     * TODO: Implement your beacon detection algorithm here.
     *
     * Input:  image[y][x] is the grayscale value (0-255) at pixel (x, y)
     *         Row 0 = top, Column 0 = left
     *
     * Output: Fill result->beacons[] with detected beacon positions.
     *         Use IMAGE CENTER coordinates (see spec for details).
     *
     * Example: detect a single bright spot at pixel (94, 60)
     *
     *   result->beacons[0].x = 94.0f - 94.0f;  // = 0.0 (center X)
     *   result->beacons[0].y = 60.0f - 60.0f;  // = 0.0 (center Y)
     *   result->beacons[0].radius = 5.0f;
     *   result->beacons[0].valid = 1;
     *   result->beacon_count = 1;
     */
}

/* Optional: return the threshold for the binary debug view */
unsigned char beacon_image_debug_threshold(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    (void)image;
    return 200;
}

/* Optional: produce a binary image for the debug view */
void beacon_image_debug_binary(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char binary[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    int x, y;
    unsigned char threshold = 200;

    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            binary[y][x] = (image[y][x] >= threshold) ? 255 : 0;
        }
    }
}
