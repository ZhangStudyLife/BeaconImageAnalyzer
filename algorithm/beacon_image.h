#ifndef BEACON_IMAGE_H_
#define BEACON_IMAGE_H_

#ifdef __cplusplus
extern "C" {
#endif

#define BEACON_IMAGE_W 188
#define BEACON_IMAGE_H 120
#define BEACON_MAX_CIRCLE_COUNT 8

typedef struct
{
    float x;
    float y;
    float radius;
    unsigned char valid;
} beacon_circle_t;

typedef struct
{
    beacon_circle_t circles[BEACON_MAX_CIRCLE_COUNT];
    unsigned char count;
} beacon_result_t;

void beacon_image_init(void);

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

#ifdef __cplusplus
}
#endif

#endif
