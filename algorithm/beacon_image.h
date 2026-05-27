#ifndef BEACON_IMAGE_H_
#define BEACON_IMAGE_H_

#ifdef __cplusplus
extern "C" {
#endif

#define BEACON_IMAGE_W 188
#define BEACON_IMAGE_H 120
#define BEACON_MAX_CIRCLE_COUNT 5
#define BEACON_IMAGE_TARGET_PIXEL_X 94.0f
#define BEACON_IMAGE_TARGET_PIXEL_Y 100.0f

#define BEACON_IMAGE_MAX_CANDIDATES 32
#define BEACON_IMAGE_TRACK_COUNT 3
#define BEACON_IMAGE_MIN_COMPONENT_AREA 4

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

typedef struct
{
    unsigned short area;
    unsigned int sum_x;
    unsigned int sum_y;
    unsigned char min_x;
    unsigned char min_y;
    unsigned char max_x;
    unsigned char max_y;
    unsigned char peak;
    unsigned char seed_x;
    unsigned char seed_y;
    unsigned char ring_mean;
    int score;
} beacon_image_candidate_t;

typedef struct
{
    float x;
    float y;
    float radius;
    int score;
    unsigned char missing;
    unsigned char active;
} beacon_image_track_t;

typedef struct
{
    unsigned char frame[BEACON_IMAGE_H][BEACON_IMAGE_W];
    unsigned char visit_stamp[BEACON_IMAGE_H][BEACON_IMAGE_W];
    unsigned char current_visit_stamp;
    unsigned char queue_x[BEACON_IMAGE_W * BEACON_IMAGE_H];
    unsigned char queue_y[BEACON_IMAGE_W * BEACON_IMAGE_H];
    beacon_image_candidate_t candidates[BEACON_IMAGE_MAX_CANDIDATES];
    beacon_image_track_t tracks[BEACON_IMAGE_TRACK_COUNT];
} beacon_image_context_t;

void beacon_image_init(void);

void beacon_image_context_init(beacon_image_context_t *context);

void beacon_image_process_with_context(
    beacon_image_context_t *context,
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result
);

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
