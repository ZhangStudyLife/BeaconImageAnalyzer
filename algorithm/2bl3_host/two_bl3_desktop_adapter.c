#include <math.h>
#include <stdint.h>
#include <string.h>

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"
#include "Image/image.h"

#define DESKTOP_IMAGE_W 188
#define DESKTOP_IMAGE_H 120
#define DESKTOP_MAX_CIRCLES 8
#define DESKTOP_MAX_BEACONS 8
#define DESKTOP_MAX_CAR_LAMPS 4
#define DESKTOP_PI 3.1415926f

typedef struct
{
    float x;
    float y;
    float radius;
    unsigned char valid;
} desktop_circle_t;

typedef struct
{
    float cx;
    float cy;
    float width;
    float length;
    float angle;
    unsigned char valid;
} desktop_rect_t;

typedef struct
{
    desktop_circle_t circles[DESKTOP_MAX_CIRCLES];
    unsigned char count;
    desktop_circle_t beacons[DESKTOP_MAX_BEACONS];
    unsigned char beacon_count;
    desktop_rect_t car_lamps[DESKTOP_MAX_CAR_LAMPS];
    unsigned char car_lamp_count;
    desktop_circle_t temporal_beacons[DESKTOP_MAX_BEACONS];
    unsigned char temporal_beacon_count;
    desktop_rect_t temporal_car_lamps[DESKTOP_MAX_CAR_LAMPS];
    unsigned char temporal_car_lamp_count;
} desktop_result_t;

uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
volatile uint8 mt9v03x_finish_flag;
uint16 g_mt9v03x_exp_time;

uint8 mt9v03x_init(void)
{
    return 0U;
}

void flash_init(void)
{
}

uint8 flash_check(uint32 sector, uint32 page)
{
    (void)sector;
    (void)page;
    return 0U;
}

void flash_read_page(uint32 sector, uint32 page, uint32 *buffer, uint32 length)
{
    (void)sector;
    (void)page;
    memset(buffer, 0, length * sizeof(*buffer));
}

void flash_erase_page(uint32 sector, uint32 page)
{
    (void)sector;
    (void)page;
}

void flash_write_page(uint32 sector, uint32 page, const uint32 *buffer, uint32 length)
{
    (void)sector;
    (void)page;
    (void)buffer;
    (void)length;
}

void beacon_image_init(void)
{
    image_init();
}

void beacon_image_reset_temporal(void)
{
    image_algorithm_params_changed();
}

void beacon_image_process(
    const unsigned char image[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W],
    desktop_result_t *result)
{
    uint8 index;

    if((image == NULL) || (result == NULL))
    {
        return;
    }
    memset(result, 0, sizeof(*result));
    memcpy(mt9v03x_image, image, sizeof(mt9v03x_image));
    mt9v03x_finish_flag = 1U;
    image_update();

    for(index = 0U; index < IMAGE_MAX_BEACON_COUNT; index++)
    {
        const beacon_data *source = &g_image_data.beacon_data[index];
        desktop_circle_t circle;
        if(image_data_beacon_valid(source) == 0U)
        {
            continue;
        }
        circle.x = -source->x;
        circle.y = source->y;
        circle.radius = sqrtf(source->area / DESKTOP_PI);
        circle.valid = 1U;
        result->beacons[result->beacon_count++] = circle;
        result->circles[result->count++] = circle;
    }

    for(index = 0U; index < IMAGE_MAX_CAR_LAMP_COUNT; index++)
    {
        const car_lamp_data *source = &g_image_data.car_lamp_data[index];
        desktop_rect_t *target;
        if(image_data_car_lamp_valid(source) == 0U)
        {
            continue;
        }
        target = &result->car_lamps[result->car_lamp_count++];
        target->cx = -source->cx;
        target->cy = source->cy;
        target->width = source->width;
        target->length = source->length;
        target->angle = source->angle;
        target->valid = 1U;
    }
}

uint32_t beacon_image_debug_build_id(void)
{
    return IMAGE_ALGORITHM_BUILD_ID;
}

uint16_t beacon_image_debug_parameter_count(void)
{
    return image_param_count();
}

int beacon_image_debug_parameter_info(uint16_t index,
                                      uint16_t *parameter_id,
                                      uint8_t *type,
                                      float *minimum,
                                      float *maximum)
{
    return image_param_info(index, parameter_id, type, minimum, maximum);
}

int beacon_image_debug_parameter_get(uint8_t type,
                                     uint16_t parameter_id,
                                     uint32_t *value_bits)
{
    return image_param_get(type, parameter_id, value_bits);
}

int beacon_image_debug_parameter_set(uint8_t type,
                                     uint16_t parameter_id,
                                     uint32_t value_bits,
                                     uint32_t *actual_bits)
{
    return image_param_set(type, parameter_id, value_bits, actual_bits);
}
