#include "beacon_image.h"

#include <math.h>
#include <stdio.h>
#include <string.h>


static int expect_beacon_at(int center_x, int center_y)
{
    static unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W];
    beacon_result_t result;
    int x;
    int y;

    memset(image, 0, sizeof(image));
    for (y = center_y - 2; y <= center_y + 2; y++)
    {
        for (x = center_x - 2; x <= center_x + 2; x++)
        {
            image[y][x] = 255;
        }
    }

    beacon_image_reset_temporal();
    beacon_image_process(image, &result);
    return (result.beacon_count == 1 &&
            fabsf(result.beacons[0].x - (94.0f - (float)center_x)) < 0.01f &&
            fabsf(result.beacons[0].y - ((float)center_y - 60.0f)) < 0.01f) ? 1 : 0;
}


static int expect_current_car_position(void)
{
    static unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W];
    beacon_result_t result;
    int x;
    int y;

    beacon_image_reset_temporal();
    for (x = 60; x <= 79; x++)
    {
        for (y = 54; y <= 59; y++)
        {
            image[y][x] = 255;
        }
    }
    beacon_image_process(image, &result);

    memset(image, 0, sizeof(image));
    for (x = 100; x <= 119; x++)
    {
        for (y = 70; y <= 75; y++)
        {
            image[y][x] = 255;
        }
    }
    beacon_image_process(image, &result);

    return (result.car_lamp_count == 1 &&
            fabsf(result.car_lamps[0].cx - (94.0f - 109.5f)) < 0.01f &&
            fabsf(result.car_lamps[0].cy - 12.5f) < 0.01f) ? 1 : 0;
}


int main(void)
{
    beacon_image_init();
    if (!expect_beacon_at(50, 30) ||
        !expect_beacon_at(BEACON_IMAGE_W - 1 - 50, 30) ||
        !expect_beacon_at(50, BEACON_IMAGE_H - 1 - 30) ||
        !expect_beacon_at(BEACON_IMAGE_W - 1 - 50,
                          BEACON_IMAGE_H - 1 - 30))
    {
        fputs("symmetric beacon test failed\n", stderr);
        return 1;
    }
    if (!expect_current_car_position())
    {
        fputs("current car position test failed\n", stderr);
        return 2;
    }

    puts("down-camera symmetric smoke test passed");
    return 0;
}
