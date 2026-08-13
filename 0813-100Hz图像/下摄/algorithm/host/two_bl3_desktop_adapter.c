#include <math.h>
#include <stdint.h>
#include <string.h>

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"
#include "Image/image.h"
#include "Image/image_down_horizon.h"
#include "Display/image_debug_screen.h"

#define DESKTOP_IMAGE_W 188
#define DESKTOP_IMAGE_H 120
#define DESKTOP_MAX_CIRCLES 8
#define DESKTOP_MAX_BEACONS 8
#define DESKTOP_MAX_CAR_LAMPS 4
#define DESKTOP_HORIZON_COLUMN_OUTSIDE 0U
#define DESKTOP_HORIZON_COLUMN_PARTIAL 1U
#define DESKTOP_HORIZON_COLUMN_INSIDE 2U

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
    desktop_circle_t candidate_beacons[DESKTOP_MAX_BEACONS];
    unsigned char candidate_beacon_count;
} desktop_result_t;

typedef struct
{
    float roll_deg;
    float pitch_deg;
    float height_mm;
    unsigned char attitude_valid;
    unsigned char height_valid;
} desktop_telemetry_t;

uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
volatile uint8 mt9v03x_finish_flag;
volatile uint32 mt9v03x_frame_sequence;
uint16 g_mt9v03x_exp_time = 400U;
struct image_data image_data[IMAGE_CAMERA_COUNT];
static uint8 s_screen_mode;
static desktop_telemetry_t s_telemetry;
static uint32_t s_processed_frame_count;

static float desktop_x(float board_x)
{
    return -board_x;
}

uint8 mt9v03x_init(void)
{
    return 0U;
}

uint8 ImageDebugScreen_SetMode(uint8 mode)
{
    if (mode > IMAGE_DEBUG_SCREEN_MODE_OVERLAY)
    {
        return 1U;
    }
    s_screen_mode = mode;
    return 0U;
}

uint8 ImageDebugScreen_GetMode(void)
{
    return s_screen_mode;
}

static void run_frame(const unsigned char image[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W])
{
    image_down_horizon_update(s_telemetry.roll_deg,
                              s_telemetry.pitch_deg,
                              s_telemetry.height_mm,
                              s_telemetry.attitude_valid,
                              s_telemetry.height_valid);
    memcpy(mt9v03x_image, image, sizeof(mt9v03x_image));
    mt9v03x_frame_sequence++;
    mt9v03x_finish_flag = 1U;
    (void)image_down_update();
    s_processed_frame_count++;
}

void beacon_image_init(void)
{
    memset(mt9v03x_image, 0, sizeof(mt9v03x_image));
    memset(image_data, 0, sizeof(image_data));
    memset(&s_telemetry, 0, sizeof(s_telemetry));
    mt9v03x_finish_flag = 0U;
    mt9v03x_frame_sequence = 0U;
    s_screen_mode = IMAGE_DEBUG_SCREEN_MODE_DATA;
    s_processed_frame_count = 0U;
    image_down_horizon_init();
    image_down_init();
}

void beacon_image_reset_temporal(void)
{
    memset(&s_telemetry, 0, sizeof(s_telemetry));
    s_processed_frame_count = 0U;
    image_down_horizon_invalidate();
    image_down_init();
}

void beacon_image_set_telemetry(unsigned char board_id,
                                float roll_deg,
                                float pitch_deg,
                                float height_mm,
                                unsigned char attitude_valid,
                                unsigned char height_valid)
{
    (void)board_id;
    s_telemetry.roll_deg = roll_deg;
    s_telemetry.pitch_deg = pitch_deg;
    s_telemetry.height_mm = height_mm;
    s_telemetry.attitude_valid = attitude_valid;
    s_telemetry.height_valid = height_valid;
}

void beacon_image_process(const unsigned char image[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W],
                          desktop_result_t *result)
{
    uint8 index;
    if (result == 0 || image == 0)
    {
        return;
    }
    memset(result, 0, sizeof(*result));
    run_frame(image);
    for (index = 0U; index < IMAGE_MAX_BEACON_COUNT && index < DESKTOP_MAX_BEACONS; ++index)
    {
        const beacon_data *source = &image_data[Center].beacon_data[index];
        if (image_data_beacon_valid(source) == 0U)
        {
            continue;
        }
        result->beacons[result->beacon_count].x = desktop_x(source->x);
        result->beacons[result->beacon_count].y = source->y;
        result->beacons[result->beacon_count].radius = sqrtf(source->area / 3.1415926f);
        result->beacons[result->beacon_count].valid = 1U;
        result->beacon_count++;
    }
    for (index = 0U; index < IMAGE_MAX_CAR_LAMP_COUNT && index < DESKTOP_MAX_CAR_LAMPS; ++index)
    {
        const car_lamp_data *source = &image_data[Center].car_lamp_data[index];
        if (image_data_car_lamp_valid(source) == 0U)
        {
            continue;
        }
        result->car_lamps[result->car_lamp_count].cx = desktop_x(source->cx);
        result->car_lamps[result->car_lamp_count].cy = source->cy;
        result->car_lamps[result->car_lamp_count].width = source->width;
        result->car_lamps[result->car_lamp_count].length = source->length;
        result->car_lamps[result->car_lamp_count].angle = source->angle;
        result->car_lamps[result->car_lamp_count].valid = 1U;
        result->car_lamp_count++;
    }
}

void beacon_image_debug_binary(const unsigned char image[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W],
                               unsigned char binary[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W])
{
    (void)image;
    if (binary == 0)
    {
        return;
    }
    if (s_screen_mode == IMAGE_DEBUG_SCREEN_MODE_LAMP_BINARY)
    {
        memcpy(binary, image_down_get_car_lamp_binary_buffer(),
               DESKTOP_IMAGE_W * DESKTOP_IMAGE_H);
    }
    else
    {
        memcpy(binary, image_down_get_binary_buffer(),
               DESKTOP_IMAGE_W * DESKTOP_IMAGE_H);
    }
}

void beacon_image_debug_car_lamp_binary(
    const unsigned char image[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W],
    unsigned char binary[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W])
{
    (void)image;
    if (binary == 0)
    {
        return;
    }
    memcpy(binary, image_down_get_car_lamp_binary_buffer(),
           DESKTOP_IMAGE_W * DESKTOP_IMAGE_H);
}

unsigned char beacon_image_debug_car_lamp_pixel_areas(unsigned short areas[DESKTOP_MAX_CAR_LAMPS])
{
    uint8 index;
    uint8 count = 0U;
    if (areas == 0)
    {
        return 0U;
    }
    for (index = 0U; index < IMAGE_MAX_CAR_LAMP_COUNT && count < DESKTOP_MAX_CAR_LAMPS; ++index)
    {
        const car_lamp_data *lamp = &image_data[Center].car_lamp_data[index];
        if (image_data_car_lamp_valid(lamp) != 0U)
        {
            areas[count++] = (unsigned short)(lamp->width * lamp->length + 0.5f);
        }
    }
    return count;
}

unsigned char beacon_image_debug_horizon(float y[DESKTOP_IMAGE_W],
                                         unsigned char column_valid[DESKTOP_IMAGE_W])
{
    if (y == 0 || column_valid == 0 || g_image_down_horizon_valid == 0U)
    {
        return 0U;
    }
    memcpy(y, g_image_down_horizon_top_y, DESKTOP_IMAGE_W * sizeof(y[0]));
    memcpy(column_valid,
           g_image_down_horizon_column_valid,
           DESKTOP_IMAGE_W * sizeof(column_valid[0]));
    return 1U;
}

unsigned char beacon_image_debug_horizon_secondary(
    float y[DESKTOP_IMAGE_W],
    unsigned char column_valid[DESKTOP_IMAGE_W])
{
    if (y == 0 || column_valid == 0 || g_image_down_horizon_valid == 0U)
    {
        return 0U;
    }
    memcpy(y, g_image_down_horizon_bottom_y, DESKTOP_IMAGE_W * sizeof(y[0]));
    memcpy(column_valid,
           g_image_down_horizon_column_valid,
           DESKTOP_IMAGE_W * sizeof(column_valid[0]));
    return 1U;
}

unsigned char beacon_image_debug_horizon_region(
    unsigned char column_state[DESKTOP_IMAGE_W])
{
    uint16 x;

    if (column_state == 0 || g_image_down_horizon_valid == 0U)
    {
        return 0U;
    }
    if (g_image_down_horizon_extrapolated != 0U)
    {
        memset(column_state,
               DESKTOP_HORIZON_COLUMN_INSIDE,
               DESKTOP_IMAGE_W * sizeof(column_state[0]));
        return 1U;
    }
    for (x = 0U; x < DESKTOP_IMAGE_W; x++)
    {
        if (g_image_down_horizon_column_valid[x] != 0U)
        {
            column_state[x] = DESKTOP_HORIZON_COLUMN_PARTIAL;
        }
        else if (g_image_down_horizon_top_y[x] < 0.0f &&
                 g_image_down_horizon_bottom_y[x] >
                     (float)(DESKTOP_IMAGE_H - 1))
        {
            column_state[x] = DESKTOP_HORIZON_COLUMN_INSIDE;
        }
        else
        {
            column_state[x] = DESKTOP_HORIZON_COLUMN_OUTSIDE;
        }
    }
    return 1U;
}

uint32_t beacon_image_debug_processed_frame_count(void)
{
    return s_processed_frame_count;
}

uint32_t beacon_image_debug_build_id(void)
{
    return IMAGE_ALGORITHM_BUILD_ID;
}

uint16_t beacon_image_debug_parameter_count(void)
{
    return image_param_count();
}

int beacon_image_debug_parameter_info(uint16_t index, uint16_t *id, uint8_t *type,
                                      float *minimum, float *maximum)
{
    return image_param_info(index, id, type, minimum, maximum);
}

int beacon_image_debug_parameter_get(uint8_t type, uint16_t id, uint32_t *value_bits)
{
    return image_param_get(type, id, value_bits);
}

int beacon_image_debug_parameter_set(uint8_t type, uint16_t id, uint32_t value_bits,
                                     uint32_t *actual_bits)
{
    return image_param_set(type, id, value_bits, actual_bits);
}
