#include "../algorithm/beacon_image.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    beacon_result_t result;
    unsigned int guard;
} guarded_result_t;

int main(void)
{
    static unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W];
    guarded_result_t output;
    int x;
    int y;

    memset(&output, 0xA5, sizeof(output));
    memset(image, 0, sizeof(image));
    output.guard = 0x5AA55AA5U;

    beacon_image_init();
    beacon_image_reset_temporal();
    beacon_image_process(image, &output.result);

    if(output.guard != 0x5AA55AA5U)
    {
        fputs("result ABI overwrite\n", stderr);
        return 1;
    }
    if((output.result.beacon_count != 0U) ||
       (output.result.car_lamp_count != 0U) ||
       (output.result.count != 0U))
    {
        fputs("zero frame produced detections\n", stderr);
        return 2;
    }

    for(y = 39; y <= 41; y++)
    {
        for(x = 99; x <= 101; x++)
        {
            image[y][x] = 255U;
        }
    }
    beacon_image_process(image, &output.result);
    if((output.result.beacon_count != 1U) ||
       (output.result.count != 1U) ||
       (output.result.beacons[0].valid == 0U))
    {
        fputs("synthetic beacon was not detected\n", stderr);
        return 3;
    }
    if((fabsf(output.result.beacons[0].x + 6.0f) > 0.01f) ||
       (fabsf(output.result.beacons[0].y + 20.0f) > 0.01f) ||
       (fabsf(output.result.circles[0].x - output.result.beacons[0].x) > 0.01f) ||
       (fabsf(output.result.circles[0].y - output.result.beacons[0].y) > 0.01f))
    {
        fputs("coordinate conversion or legacy sync failed\n", stderr);
        return 4;
    }

    puts("0716 instance smoke test passed");
    return 0;
}
