#ifndef BEACON_IMAGE_H_
#define BEACON_IMAGE_H_

#ifdef __cplusplus
extern "C" {
#endif

#define BEACON_IMAGE_W 188
#define BEACON_IMAGE_H 120
#define BEACON_MAX_CIRCLE_COUNT 8
#define BEACON_MAX_BEACON_COUNT 8
#define BEACON_MAX_CAR_LAMP_COUNT 4

typedef struct
{
    float x;
    float y;
    float radius;
    unsigned char valid;
} beacon_circle_t;

typedef struct
{
    float cx;
    float cy;
    float width;
    float length;
    float angle;
    unsigned char valid;
} beacon_rect_t;

typedef struct
{
    beacon_circle_t circles[BEACON_MAX_CIRCLE_COUNT];
    unsigned char count;
    beacon_circle_t beacons[BEACON_MAX_BEACON_COUNT];
    unsigned char beacon_count;
    beacon_rect_t car_lamps[BEACON_MAX_CAR_LAMP_COUNT];
    unsigned char car_lamp_count;
    beacon_circle_t temporal_beacons[BEACON_MAX_BEACON_COUNT];
    unsigned char temporal_beacon_count;
    beacon_rect_t temporal_car_lamps[BEACON_MAX_CAR_LAMP_COUNT];
    unsigned char temporal_car_lamp_count;
} beacon_result_t;

void beacon_image_init(void);
void beacon_image_reset_temporal(void);

void beacon_image_process(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result
);

unsigned char beacon_image_debug_threshold(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W]
);

void beacon_image_debug_binary(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char binary[BEACON_IMAGE_H][BEACON_IMAGE_W]
);

unsigned char beacon_image_debug_car_lamp_pixel_areas(
    unsigned short areas[BEACON_MAX_CAR_LAMP_COUNT]
);

#ifdef __cplusplus
}
#endif

#endif
