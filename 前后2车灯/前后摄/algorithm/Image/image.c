#include "image.h"

#include <math.h>
#include <string.h>

#include "zf_device_mt9v03x.h"
#include "image_horizon.h"

#define BEACON_IMAGE_W 188
#define BEACON_IMAGE_H 120
#define BEACON_MAX_BEACON_COUNT 3
#define BEACON_MAX_CAR_LAMP_COUNT 2

typedef struct
{
    beacon_circle_t beacons[BEACON_MAX_BEACON_COUNT];
    unsigned char beacon_count;
    beacon_rect_t car_lamps[BEACON_MAX_CAR_LAMP_COUNT];
    unsigned char car_lamp_count;
    unsigned char car_lamp_measured_mask;
} beacon_result_t;

#if (MT9V03X_W != BEACON_IMAGE_W) || (MT9V03X_H != BEACON_IMAGE_H)
#error "Beacon image algorithm is tuned for MT9V03X 188x120 frames."
#endif

static float beacon_area(const beacon_circle_t *beacon);
#define CAR_LAMP_BINARY_THRESHOLD car_lamp_binary_threshold
#define CAR_LAMP_UPPER_THRESHOLD  car_lamp_upper_threshold
#define CAR_LAMP_UPPER_Y          car_lamp_upper_y
#define CAR_LAMP_BRIDGE_MAX_GAP   car_lamp_bridge_max_gap
#define BEACON_TRACK_THRESHOLD    beacon_track_threshold
#define BEACON_EDGE_THRESHOLD     beacon_edge_threshold
#define BEACON_TOP_THRESHOLD_Y    45
#define BEACON_EDGE_MIN_AREA      beacon_edge_min_area
#define BEACON_EDGE_MAX_AREA      beacon_edge_max_area
#define BEACON_TOP_EDGE_MAX_AREA  beacon_top_edge_max_area
#define BEACON_EDGE_TOP_Y         36
#define BEACON_EDGE_BOTTOM_Y      104
#define BEACON_EDGE_LEFT_X        19
#define BEACON_EDGE_RIGHT_X       172
#define BEACON_TOP_CORNER_WIDTH   40.0f
#define BEACON_TOP_CORNER_HEIGHT  27.0f
#define COMPONENT_TOP_REJECT_Y    0
#define COMPONENT_BOTTOM_REJECT_Y (BEACON_IMAGE_H - 1)
#define BEACON_LOCAL_RING_INNER   beacon_local_ring_inner
#define BEACON_LOCAL_RING_OUTER   beacon_local_ring_outer
#define BEACON_ISOLATED_MIN_AREA  2
#define BEACON_ISOLATED_MAX_AREA  5
#define BEACON_ISOLATED_GRAY_MIN  beacon_isolated_gray_min
#define BEACON_ISOLATED_BG_MAX    beacon_isolated_bg_max
#define BEACON_LOWER_SMALL_Y      90
#define BEACON_WEAK_FOOTPRINT_RADIUS 4
#define BEACON_WEAK_FOOTPRINT_GRAY 40
#define BEACON_WEAK_FOOTPRINT_MAX 15
#define BEACON_LINEAR_MIN_AREA     12
#define BEACON_LINEAR_MIN_MAJOR    13.0f
#define BEACON_LINEAR_MAX_MINOR    4.5f
#define BEACON_LINEAR_MIN_ELONGATION linear_min_elongation
#define BEACON_TINY_LINE_MAX_AREA  3
#define BEACON_TINY_LINE_MAX_MINOR 1.05f
#define BEACON_TINY_LINE_MIN_ELONGATION vertical_glare_min_elongation
#define BEACON_TINY_LINE_MAX_GRAY  vertical_glare_max_gray
#define BEACON_TINY_TRACK_MAX_AREA 3.0f
#define BEACON_WEAK_CENTER_THRESHOLD weak_center_threshold
#define BEACON_WEAK_CENTER_MIN_X   70
#define BEACON_WEAK_CENTER_MAX_X   172
#define BEACON_WEAK_CENTER_BASE_MAX_X 125
#define BEACON_WEAK_CENTER_NEAR_RIGHT_MAX_X 145
#define BEACON_WEAK_CENTER_FAR_RIGHT_MIN_X 165
#define BEACON_WEAK_CENTER_RIGHT_MIN_GRAY 120
#define BEACON_WEAK_CENTER_FAR_RIGHT_MIN_Y 55
#define BEACON_WEAK_CENTER_FAR_RIGHT_MIN_AREA 7
#define BEACON_WEAK_CENTER_FAR_RIGHT_MAX_AREA 8
#define BEACON_WEAK_CENTER_FAR_RIGHT_MIN_GRAY 150
#define BEACON_WEAK_CENTER_FAR_RIGHT_MAX_ELONGATION 1.2f
#define BEACON_WEAK_CENTER_MIN_Y   45
#define BEACON_WEAK_CENTER_MAX_Y   100
#define BEACON_WEAK_CENTER_MIN_AREA weak_center_min_area
#define BEACON_WEAK_CENTER_UPPER_MIN_AREA 7
#define BEACON_WEAK_CENTER_UPPER_MAX_AREA 8
#define BEACON_WEAK_CENTER_UPPER_MIN_GRAY 110
#define BEACON_WEAK_CENTER_UPPER_MAX_GRAY 120
#define BEACON_WEAK_CENTER_UPPER_MAX_MEAN 94
#define BEACON_WEAK_CENTER_FULL_MIN_Y 58
#define BEACON_WEAK_CENTER_RIGHT_MAX_AREA 6
#define BEACON_WEAK_CENTER_MAX_AREA weak_center_max_area
#define BEACON_WEAK_CENTER_MIN_GRAY weak_center_min_gray
#define BEACON_WEAK_CENTER_MAX_BG  weak_center_max_bg
#define BEACON_WEAK_CENTER_MAX_ELONGATION 2.0f
#define BEACON_WEAK_CENTER_DUPLICATE_DISTANCE 8.0f
#define BEACON_SHAPE_MIN_AREA      shape_min_area
#define BEACON_SHAPE_RATIO_MIN_AREA 12
#define BEACON_SHAPE_MAX_RATIO_NUM shape_max_ratio
#define BEACON_SHAPE_MAX_RATIO_DEN 1.0f
#define BEACON_SHAPE_FILL_MIN_AREA 12
#define BEACON_SHAPE_MIN_FILL_PERCENT shape_min_fill_percent
#define BEACON_SHAPE_SMALL_MIN_FILL_PERCENT shape_small_min_fill_percent
#define BEACON_BAD_SHAPE_MAX_COUNT 8
#define BEACON_BAD_SHAPE_MATCH_PAD 4
#define BEACON_OUTPUT_DIM_MID_MIN_Y 35.0f
#define BEACON_OUTPUT_DIM_MID_SPLIT_Y 45.0f
#define BEACON_OUTPUT_DIM_MID_MAX_Y 58.0f
#define BEACON_OUTPUT_DIM_MID_MAX_AREA 12.0f
#define BEACON_OUTPUT_DIM_MID_MAX_GRAY 99
#define BEACON_OUTPUT_DIM_UPPER_MAX_GRAY 110
#define BEACON_OUTPUT_SIDE_MIN_Y 30.0f
#define BEACON_OUTPUT_SIDE_MARGIN 16.0f
#define BEACON_OUTPUT_SIDE_MAX_AREA 12.0f
#define BEACON_OUTPUT_SIDE_MAX_GRAY 120
#define BEACON_OUTPUT_LOCAL_RADIUS 4
#define BEACON_TOP_VERTICAL_MIN_AREA 4
#define BEACON_TOP_VERTICAL_MAX_AREA 4
#define BEACON_TOP_VERTICAL_MIN_GRAY 125
#define BEACON_TOP_VERTICAL_MAX_GRAY 135
#define BEACON_TOP_VERTICAL_MIN_ELONGATION top_vertical_min_elongation
#define BEACON_TOP_VERTICAL_MAX_WIDTH 2
#define BEACON_TOP_VERTICAL_LAMP_MIN_AREA 60
#define BEACON_TOP_VERTICAL_LAMP_MAX_DX 6.0f
#define BEACON_TOP_VERTICAL_LAMP_MIN_DY 50.0f
#define BEACON_TOP_VERTICAL_LAMP_MAX_DY 58.0f
#define BEACON_SATURATED_TOP_MIN_Y 20.0f
#define BEACON_SATURATED_TOP_MAX_Y 40.0f
#define BEACON_SATURATED_TOP_MIN_AREA 18
#define BEACON_SATURATED_TOP_MAX_AREA 20
#define BEACON_SATURATED_TOP_MIN_GRAY saturated_top_min_gray
#define BEACON_SATURATED_TOP_MIN_FILL_PERCENT 75
#define BEACON_SATURATED_TOP_LAMP_MAX_DX 30.0f
#define BEACON_SATURATED_TOP_LAMP_MIN_DX 20.0f
#define BEACON_SATURATED_TOP_LAMP_MIN_DY 35.0f
#define BEACON_SATURATED_TOP_LAMP_MAX_DY 46.0f
#define LAMP_MASK_PAD             2
#define LAMP_MASK_DOWN_PAD        6
#define LAMP_NEAR_BEACON_PAD      lamp_near_beacon_pad
#define LAMP_NEAR_BEACON_MIN_AREA lamp_near_beacon_min_area
#define LAMP_NEAR_BEACON_ISOLATED_MIN_AREA 3
#define LAMP_NEAR_BEACON_BACKGROUND_MAX lamp_near_beacon_background_max
#define LAMP_NEAR_BEACON_GRAY_MIN lamp_near_beacon_gray_min
#define BEACON_MIN_COMPONENT_AREA beacon_min_component_area
#define CAR_LAMP_MIN_AREA         car_lamp_min_area
#define CAR_LAMP_MAX_AREA         car_lamp_max_area
#define CAR_LAMP_MIN_ELONGATION   car_lamp_min_elongation
#define CAR_LAMP_MIN_LENGTH       car_lamp_min_length
#define CAR_LAMP_MIN_WIDTH        car_lamp_min_width
#define CAR_LAMP_NARROW_MIN_WIDTH car_lamp_narrow_min_width
#define CAR_LAMP_NARROW_MIN_ELONGATION car_lamp_narrow_min_elongation
#define CAR_LAMP_UPPER_MIN_AREA   car_lamp_upper_min_area
#define CAR_LAMP_UPPER_MIN_LENGTH car_lamp_upper_min_length
#define CAR_LAMP_UPPER_MIN_WIDTH  car_lamp_upper_min_width
#define CAR_LAMP_UPPER_COMPACT_MIN_Y car_lamp_compact_min_y
#define CAR_LAMP_UPPER_COMPACT_MIN_AREA car_lamp_compact_min_area
#define CAR_LAMP_UPPER_COMPACT_MIN_LENGTH car_lamp_compact_min_length
#define CAR_LAMP_UPPER_COMPACT_MIN_WIDTH car_lamp_compact_min_width
#define CAR_LAMP_UPPER_COMPACT_MIN_ELONGATION car_lamp_compact_min_elongation
#define CAR_LAMP_HORIZON_MARGIN_PX 2.0f
#define CAR_LAMP_MIN_CENTER_Y     (BEACON_IMAGE_H / 3)
#define CAR_LAMP_Y_PRIORITY_MARGIN 3.0f
#define CAR_LAMP_EDGE_MAX_MISSES  3
#define CAR_LAMP_CENTER_MAX_MISSES 24
#define CAR_LAMP_TEMPORAL_EDGE_MARGIN 8
#define CAR_LAMP_TEMPORAL_MASK_PAD 4
#define CAR_LAMP_TEMPORAL_CORE_PAD 2
#define CAR_LAMP_TEMPORAL_TAKEOVER_PAD 10
#define CAR_LAMP_TRACK_SIZE_MIN_RATIO  0.55f
#define CAR_LAMP_TRACK_SIZE_MAX_RATIO  1.80f
#define CAR_LAMP_GRAY_SCENE_MAX        40.0f
#define CAR_LAMP_GRAY_THRESHOLD        120U
#define CAR_LAMP_GRAY_MIN_AREA         16
#define CAR_LAMP_GRAY_MAX_AREA         80
#define CAR_LAMP_GRAY_MIN_LENGTH       8.0f
#define CAR_LAMP_GRAY_MAX_LENGTH       20.0f
#define CAR_LAMP_GRAY_MIN_WIDTH        3.0f
#define CAR_LAMP_GRAY_MIN_ELONGATION   1.8f
#define CAR_LAMP_GRAY_MIN_PEAK         240U
#define CAR_LAMP_GRAY_MIN_MEAN         170.0f
#define CAR_LAMP_GRAY_MIN_CONTRAST     100.0f
#define CAR_LAMP_GRAY_MIN_FILL_PERCENT 35
#define CAR_LAMP_GRAY_STRIP_MIN_GRAY   150U
#define CAR_LAMP_GRAY_STRIP_MIN_FILL_PERCENT 38
#define CAR_LAMP_GRAY_TRACK_MIN_FILL_PERCENT 35
#define CAR_LAMP_STRIP_MIN_FILL_PERCENT 35
#define CAR_LAMP_MODE_SINGLE             1U
#define CAR_LAMP_MODE_DUAL               2U
#define CAR_LAMP_PAIR_MAX_ANGLE_DEG      20.0f
#define CAR_LAMP_PAIR_MAX_LENGTH_RATIO   2.0f
#define CAR_LAMP_PAIR_MAX_DISTANCE_SCALE 2.0f
#define CAR_BODY_MASK_CORNER_COUNT       8U
#define BEACON_HORIZON_TOLERANCE_PX    2.0f
#define BEACON_HORIZON_SMALL_BAND_PX   6.0f
#define B0_MATCH_DISTANCE         b0_match_distance
#define B0_SWITCH_AREA_RATIO      1.70f
#define B0_SMALL_SWITCH_AREA      12.0f
#define B0_SMALL_SWITCH_RATIO     2.50f
#define B0_INIT_CONFIRM_FRAMES    b0_init_confirm_frames
#define BEACON_MAX_MISSES         beacon_max_misses
#define KALMAN_GATE_DISTANCE      kalman_gate_distance
#define KALMAN_NEW_TARGET_DISTANCE kalman_new_target_distance
#define FILTER_POS_ALPHA          filter_pos_alpha
#define FILTER_VEL_ALPHA          filter_vel_alpha
/* Bound expensive shape and gray evaluation on fragmented-noise frames. */
#define BEACON_MAX_EVALUATED_COMPONENTS 256
#define IMAGE_QUEUE_SIZE          (BEACON_IMAGE_W * BEACON_IMAGE_H)
#define COMPONENT_EXACT_INTEGER_MOMENT_MAX \
    (0x00FFFFFFU / ((BEACON_IMAGE_W - 1U) * (BEACON_IMAGE_W - 1U)))
#define PI_F                      3.1415926f

#define GRAY_BEACON_MAX_PEAKS          16
#define GRAY_BEACON_MAX_CANDIDATES     24
#define GRAY_BEACON_PATCH_RADIUS       7
#define GRAY_BEACON_MIN_PEAK_GRAY      60U
#define GRAY_BEACON_DARK_SCENE_MEAN    10.0f
#define GRAY_BEACON_LOW_LIGHT_MEAN     20.0f
#define GRAY_BEACON_BRIGHT_SCENE_MEAN  30.0f
#define GRAY_BEACON_COMPACT_MIN_AREA   4.0f
#define GRAY_BEACON_COMPACT_MAX_AREA   20.5f
#define GRAY_BEACON_COMPACT_MIN_PEAK   220U
#define GRAY_BEACON_LARGE_MIN_Y        50.0f
#define GRAY_BEACON_UPPER_LARGE_MAX_AREA 40.0f
#define GRAY_BEACON_LARGE_SHAPE_MIN_AREA 30
#define GRAY_BEACON_LARGE_SHAPE_RADIUS 12
#define GRAY_BEACON_LARGE_SHAPE_MAX_ELONGATION 1.8f
#define GRAY_BEACON_HORIZON_SMALL_MAX_DEPTH 10.0f
#define GRAY_BEACON_HORIZON_SMALL_BG_MIN 22.0f
#define GRAY_BEACON_HORIZON_SMALL_BG_MAX 40.0f
#define BEACON_OUTPUT_PREDICT_FRAMES     2U
#define BEACON_OUTPUT_TRACK_GATE         36.0f
#define BEACON_OUTPUT_HORIZON_TOLERANCE  2.0f
#define BEACON_OUTPUT_HORIZON_SMALL_BAND 6.0f
#define BEACON_OUTPUT_TINY_AREA_MAX      8.5f
#define BEACON_OUTPUT_TINY_DEPTH_MIN     20.0f
#define BEACON_OUTPUT_REFLECTION_BG_MIN  25
#define BEACON_OUTPUT_REFLECTION_BG_MAX  50
#define BEACON_OUTPUT_OUTER_10_MIN       90
#define BEACON_OUTPUT_OUTER_20_MIN       45
#define BEACON_OUTPUT_OUTER_30_MAX       15

typedef struct
{
    int area;
    int min_x;
    int min_y;
    int max_x;
    int max_y;
    float cx;
    float cy;
    float major;
    float minor;
    float elongation;
    float angle;
    unsigned char valid;
} component_t;

typedef struct
{
    float numerator;
    float denominator;
} component_orientation_t;

typedef struct
{
    float cx;
    float cy;
    float cos_angle;
    float sin_angle;
    float half_length;
    float half_width;
    unsigned char valid;
} gray_lamp_geometry_t;

typedef struct
{
    signed short min_x[BEACON_IMAGE_H];
    signed short max_x[BEACON_IMAGE_H];
    unsigned char valid;
} car_body_mask_t;

typedef struct
{
    float x;
    float y;
} car_body_point_t;

static unsigned char gray_car_lamp_valid(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    component_t *comp,
    const component_orientation_t *orientation);
static void gray_lamp_geometry_init(
    const component_t *lamp,
    gray_lamp_geometry_t *geometry);
static unsigned char gray_point_in_lamp(
    int x,
    int y,
    const gray_lamp_geometry_t *geometry);
static unsigned char gray_point_in_lamp_shadow(
    int x,
    int y,
    const component_t *lamp);

typedef struct
{
    unsigned char active;
    unsigned char confirmed;
    unsigned char hits;
    unsigned char misses;
    float x;
    float y;
    float vx;
    float vy;
    float radius;
    float width;
    float length;
    float angle;
    float area;
} temporal_track_t;

typedef struct
{
    signed short response;
    unsigned char x;
    unsigned char y;
} gray_beacon_peak_t;

typedef struct
{
    float background;
    float inner_contrast;
    float radial_drop;
    float concentration;
    float elongation;
    float offset;
    float centroid_x;
    float centroid_y;
    unsigned char peak;
    unsigned char half_area;
} gray_beacon_features_t;

typedef struct
{
    float x;
    float y;
    float area;
    gray_beacon_features_t features;
    unsigned char features_valid;
} gray_beacon_candidate_t;

static unsigned char g_binary[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_visit[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_visit_generation = 1U;
static unsigned short g_queue[IMAGE_QUEUE_SIZE];
static temporal_track_t g_b0_track;
static temporal_track_t g_car_track;
static temporal_track_t g_car_track_secondary;
static component_t g_secondary_lamp_mask;
static component_t g_secondary_temporal_lamp_mask;
static car_body_mask_t g_car_body_mask;
static unsigned char g_car_lamp_mode = CAR_LAMP_MODE_SINGLE;
static unsigned char g_track_reinforced;
#if defined(__ICCARM__)
#pragma data_alignment=4
#endif
static unsigned short g_gray_box3_storage[BEACON_IMAGE_W + 3];
#if defined(__ICCARM__)
#pragma data_alignment=4
#endif
static unsigned short g_gray_box9_storage[BEACON_IMAGE_W + 8];
static signed short g_gray_response_rows[3][BEACON_IMAGE_W];

image_profile_data_t g_image_profile;

int32 g_beacon_binary_threshold = IMAGE_BEACON_BINARY_THRESHOLD_DEFAULT;

static void beacon_image_init(void)
{
    memset(g_binary, 0, sizeof(g_binary));
    memset(g_visit, 0, sizeof(g_visit));
    g_visit_generation = 1U;
}

static void reset_visit(void)
{
    g_visit_generation++;
    if(g_visit_generation == 0U)
    {
        memset(g_visit, 0, sizeof(g_visit));
        g_visit_generation = 1U;
    }
}

static void beacon_image_reset_temporal(void)
{
    memset(&g_b0_track, 0, sizeof(g_b0_track));
    memset(&g_car_track, 0, sizeof(g_car_track));
    memset(&g_car_track_secondary, 0, sizeof(g_car_track_secondary));
    memset(&g_secondary_lamp_mask, 0, sizeof(g_secondary_lamp_mask));
    memset(&g_secondary_temporal_lamp_mask, 0,
           sizeof(g_secondary_temporal_lamp_mask));
    g_car_body_mask.valid = 0U;
}

static void clear_result(beacon_result_t *result)
{
    if(result != 0)
    {
        memset(result, 0, sizeof(*result));
    }
}

#if defined(__ICCARM__)
#pragma inline=never
#endif
static void threshold_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char threshold)
{
#if defined(__ICCARM__)
    const unsigned char *src = &image[0][0];
    unsigned char *dst = &g_binary[0][0];
    uint32 threshold_word = (uint32)threshold * 0x01010101UL;
    int count = BEACON_IMAGE_W * BEACON_IMAGE_H / 4;

    while(count-- > 0)
    {
        uint32 gray_word = __UNALIGNED_UINT32_READ(src);
        uint32 binary_word;

        (void)__USUB8(gray_word, threshold_word);
        binary_word = __SEL(0xFFFFFFFFUL, 0U);
        __UNALIGNED_UINT32_WRITE(dst, binary_word);
        src += 4;
        dst += 4;
    }
#else
    int i;
    const unsigned char *src = &image[0][0];
    unsigned char *dst = &g_binary[0][0];

    for(i = 0; i < BEACON_IMAGE_W * BEACON_IMAGE_H; i++)
    {
        dst[i] = (src[i] >= threshold) ? 255 : 0;
    }
#endif
    reset_visit();
}

static void bridge_upper_car_lamp_gaps(void)
{
    int y;
    int limit_y = (int)CAR_LAMP_UPPER_Y;

    if(limit_y > BEACON_IMAGE_H)
    {
        limit_y = BEACON_IMAGE_H;
    }

    for(y = 0; y < limit_y; y++)
    {
        int x = 0;

        while(x < BEACON_IMAGE_W)
        {
            int start;
            int end;

            if(g_binary[y][x] != 0)
            {
                x++;
                continue;
            }

            start = x;
            while((x < BEACON_IMAGE_W) && (g_binary[y][x] == 0))
            {
                x++;
            }
            end = x;

            if((start > 0) && (end < BEACON_IMAGE_W) &&
               ((end - start) <= CAR_LAMP_BRIDGE_MAX_GAP))
            {
                int fill_x;

                for(fill_x = start; fill_x < end; fill_x++)
                {
                    g_binary[y][fill_x] = 255;
                }
            }
        }
    }
}

static uint32 threshold_car_lamp_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    uint32 gray_sum = 0U;
    int x;
    int y;

    for(y = 0; y < BEACON_IMAGE_H; y++)
    {
        unsigned char threshold = CAR_LAMP_BINARY_THRESHOLD;

        if(y < CAR_LAMP_UPPER_Y)
        {
            threshold = CAR_LAMP_UPPER_THRESHOLD;
        }

#if defined(__ICCARM__)
        {
            uint32 threshold_word =
                (uint32)threshold * 0x01010101UL;

            for(x = 0; x < BEACON_IMAGE_W; x += 4)
            {
                uint32 gray_word;
                uint32 binary_word;

                gray_word = __UNALIGNED_UINT32_READ(&image[y][x]);
                gray_sum = __USADA8(gray_word, 0U, gray_sum);
                (void)__USUB8(gray_word, threshold_word);
                binary_word = __SEL(0xFFFFFFFFUL, 0U);
                __UNALIGNED_UINT32_WRITE(&g_binary[y][x], binary_word);
            }
        }
#else
        for(x = 0; x < BEACON_IMAGE_W; x++)
        {
            unsigned char gray = image[y][x];

            gray_sum += gray;
            g_binary[y][x] = (gray >= threshold) ? 255 : 0;
        }
#endif
    }
    bridge_upper_car_lamp_gaps();
    reset_visit();
    return gray_sum;
}

static void threshold_beacon_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    int x;
    int y;

    for(y = 0; y < BEACON_IMAGE_H; y++)
    {
        for(x = 0; x < BEACON_IMAGE_W; x++)
        {
            unsigned char threshold = (unsigned char)g_beacon_binary_threshold;

            if((y < BEACON_TOP_THRESHOLD_Y) ||
               (x < BEACON_EDGE_LEFT_X) ||
               (x >= BEACON_EDGE_RIGHT_X))
            {
                threshold = BEACON_EDGE_THRESHOLD;
            }
            g_binary[y][x] = (image[y][x] >= threshold) ? 255 : 0;
        }
    }
    reset_visit();
}

static void reinforce_tracked_beacon(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    int x;
    int y;
    int count = 0;
    int cx;
    int cy;
    int min_x;
    int max_x;
    int min_y;
    int max_y;

    g_track_reinforced = 0U;
    if((g_b0_track.confirmed == 0U) ||
       (g_b0_track.misses >= BEACON_MAX_MISSES))
    {
        return;
    }

    cx = (int)((float)BEACON_IMAGE_W * 0.5f +
               g_b0_track.x + g_b0_track.vx + 0.5f);
    cy = (int)((float)BEACON_IMAGE_H * 0.5f +
               g_b0_track.y + g_b0_track.vy + 0.5f);
    min_x = cx - (int)B0_MATCH_DISTANCE;
    max_x = cx + (int)B0_MATCH_DISTANCE;
    min_y = cy - (int)B0_MATCH_DISTANCE;
    max_y = cy + (int)B0_MATCH_DISTANCE;
    if(min_x < 0) min_x = 0;
    if(min_y < 0) min_y = 0;
    if(max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if(max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;

    for(y = min_y; y <= max_y; y++)
    {
        for(x = min_x; x <= max_x; x++)
        {
            if((g_binary[y][x] != 0U) &&
               (++count >= BEACON_EDGE_MIN_AREA))
            {
                return;
            }
        }
    }

    for(y = min_y; y <= max_y; y++)
    {
        for(x = min_x; x <= max_x; x++)
        {
            if(image[y][x] >= BEACON_TRACK_THRESHOLD)
            {
                g_binary[y][x] = 255U;
            }
        }
    }
    g_track_reinforced = 1U;
}

static inline unsigned short *grow_component_enqueue(
    const unsigned char *binary,
    unsigned char *visit,
    uint32 index,
    unsigned short coordinate,
    unsigned char generation,
    unsigned short *tail)
{
    if((binary[index] == 0U) || (visit[index] == generation))
    {
        return tail;
    }

    visit[index] = generation;
    *tail++ = coordinate;
    return tail;
}

#if defined(__ICCARM__)
#pragma inline=forced
#endif
static component_t grow_component_core(
    unsigned char start_x,
    unsigned char start_y,
    int shape_min_area,
    int shape_max_area,
    component_orientation_t *orientation,
    unsigned char consume_binary)
{
    unsigned short *head = g_queue;
    unsigned short *tail = g_queue;
    unsigned char generation = g_visit_generation;
    const unsigned char *binary =
        (const unsigned char *)(const void *)g_binary;
    unsigned char *visit = (unsigned char *)(void *)g_visit;
    int sum_x = 0;
    int sum_y = 0;
#if defined(__ICCARM__)
    uint32 min_xy = __PKHBT((uint32)start_x, (uint32)start_y, 16);
    uint32 max_xy = min_xy;
#endif
    component_t comp;

    if(consume_binary != 0U)
    {
        generation = 0U;
        visit = (unsigned char *)(void *)g_binary;
    }

    memset(&comp, 0, sizeof(comp));
    comp.min_x = start_x;
    comp.max_x = start_x;
    comp.min_y = start_y;
    comp.max_y = start_y;
    *tail++ = (unsigned short)start_x | ((unsigned short)start_y << 8U);
    visit[(uint32)start_y * BEACON_IMAGE_W + start_x] = generation;

    while(head < tail)
    {
        unsigned short coordinate = *head++;
        unsigned char x = (unsigned char)coordinate;
        unsigned char y = (unsigned char)(coordinate >> 8U);
        uint32 index = (uint32)y * BEACON_IMAGE_W + x;

        comp.area++;
        sum_x += x;
        sum_y += y;

#if defined(__ICCARM__)
        {
            uint32 xy = __PKHBT((uint32)x, (uint32)y, 16);

            (void)__USUB16(xy, min_xy);
            min_xy = __SEL(min_xy, xy);
            (void)__USUB16(xy, max_xy);
            max_xy = __SEL(xy, max_xy);
        }
#else
        if((int)x < comp.min_x) comp.min_x = x;
        if((int)x > comp.max_x) comp.max_x = x;
        if((int)y < comp.min_y) comp.min_y = y;
        if((int)y > comp.max_y) comp.max_y = y;
#endif

        if((x > 0U) && (x < (BEACON_IMAGE_W - 1)) &&
           (y > 0U) && (y < (BEACON_IMAGE_H - 1)))
        {
            tail = grow_component_enqueue(
                binary, visit,
                index + 1U,
                (unsigned short)(coordinate + 1U), generation, tail);
            tail = grow_component_enqueue(
                binary, visit,
                index - 1U,
                (unsigned short)(coordinate - 1U), generation, tail);
            tail = grow_component_enqueue(
                binary, visit,
                index + BEACON_IMAGE_W,
                (unsigned short)(coordinate + 0x0100U), generation, tail);
            tail = grow_component_enqueue(
                binary, visit,
                index - BEACON_IMAGE_W,
                (unsigned short)(coordinate - 0x0100U), generation, tail);
            tail = grow_component_enqueue(
                binary, visit,
                index + BEACON_IMAGE_W + 1U,
                (unsigned short)(coordinate + 0x0101U), generation, tail);
            tail = grow_component_enqueue(
                binary, visit,
                index - BEACON_IMAGE_W + 1U,
                (unsigned short)(coordinate - 0x00FFU), generation, tail);
            tail = grow_component_enqueue(
                binary, visit,
                index + BEACON_IMAGE_W - 1U,
                (unsigned short)(coordinate + 0x00FFU), generation, tail);
            tail = grow_component_enqueue(
                binary, visit,
                index - BEACON_IMAGE_W - 1U,
                (unsigned short)(coordinate - 0x0101U), generation, tail);
        }
        else
        {
            if(x + 1U < BEACON_IMAGE_W)
            {
                tail = grow_component_enqueue(
                    binary, visit,
                    index + 1U,
                    (unsigned short)(coordinate + 1U), generation, tail);
            }
            if(x > 0U)
            {
                tail = grow_component_enqueue(
                    binary, visit,
                    index - 1U,
                    (unsigned short)(coordinate - 1U), generation, tail);
            }
            if(y + 1U < BEACON_IMAGE_H)
            {
                tail = grow_component_enqueue(
                    binary, visit,
                    index + BEACON_IMAGE_W,
                    (unsigned short)(coordinate + 0x0100U),
                    generation, tail);
            }
            if(y > 0U)
            {
                tail = grow_component_enqueue(
                    binary, visit,
                    index - BEACON_IMAGE_W,
                    (unsigned short)(coordinate - 0x0100U),
                    generation, tail);
            }
            if((x + 1U < BEACON_IMAGE_W) && (y + 1U < BEACON_IMAGE_H))
            {
                tail = grow_component_enqueue(
                    binary, visit,
                    index + BEACON_IMAGE_W + 1U,
                    (unsigned short)(coordinate + 0x0101U),
                    generation, tail);
            }
            if((x + 1U < BEACON_IMAGE_W) && (y > 0U))
            {
                tail = grow_component_enqueue(
                    binary, visit,
                    index - BEACON_IMAGE_W + 1U,
                    (unsigned short)(coordinate - 0x00FFU),
                    generation, tail);
            }
            if((x > 0U) && (y + 1U < BEACON_IMAGE_H))
            {
                tail = grow_component_enqueue(
                    binary, visit,
                    index + BEACON_IMAGE_W - 1U,
                    (unsigned short)(coordinate + 0x00FFU),
                    generation, tail);
            }
            if((x > 0U) && (y > 0U))
            {
                tail = grow_component_enqueue(
                    binary, visit,
                    index - BEACON_IMAGE_W - 1U,
                    (unsigned short)(coordinate - 0x0101U),
                    generation, tail);
            }
        }
    }

#if defined(__ICCARM__)
    comp.min_x = (int)(min_xy & 0xFFFFU);
    comp.min_y = (int)(min_xy >> 16U);
    comp.max_x = (int)(max_xy & 0xFFFFU);
    comp.max_y = (int)(max_xy >> 16U);
#endif
    if(comp.area > 0)
    {
        float inv_area = 1.0f / (float)comp.area;
        float sum_xx = 0.0f;
        float sum_yy = 0.0f;
        float sum_xy = 0.0f;
        float var_x;
        float var_y;
        float cov_xy;
        float trace;
        float det;
        float disc;
        float eig_delta;
        float eig_major;
        float eig_minor;
        unsigned short moment_count = (unsigned short)(tail - g_queue);
        unsigned short *moment_end;

        comp.cx = (float)sum_x * inv_area;
        comp.cy = (float)sum_y * inv_area;
        comp.valid = 1;
        /* Rejected size ranges only need area, bounds and centroid. */
        if((shape_min_area <= 0) ||
           (comp.area < shape_min_area) ||
           ((shape_max_area >= shape_min_area) &&
            (comp.area > shape_max_area)))
        {
            return comp;
        }
        if((shape_max_area >= 0) &&
           (shape_max_area < (int)moment_count))
        {
            moment_count = (unsigned short)shape_max_area;
        }
        if(moment_count <= COMPONENT_EXACT_INTEGER_MOMENT_MAX)
        {
            uint32 int_sum_xx = 0U;
            uint32 int_sum_yy = 0U;
            uint32 int_sum_xy = 0U;

            head = g_queue;
            moment_end = g_queue + moment_count;
#if defined(__ICCARM__)
            {
                unsigned short *pair_end =
                    g_queue + (moment_count & ~1U);

                while(head < pair_end)
                {
                    uint32 coordinates = __UNALIGNED_UINT32_READ(head);
                    uint32 x_pair = __UXTB16(coordinates);
                    uint32 y_pair = __UXTB16(__ROR(coordinates, 8U));

                    head += 2;
                    int_sum_xx = (uint32)__SMLAD(
                        x_pair, x_pair, int_sum_xx);
                    int_sum_yy = (uint32)__SMLAD(
                        y_pair, y_pair, int_sum_yy);
                    int_sum_xy = (uint32)__SMLAD(
                        x_pair, y_pair, int_sum_xy);
                }
            }
#endif
            while(head < moment_end)
            {
                unsigned short coordinate = *head++;
                uint32 x = (unsigned char)coordinate;
                uint32 y = (unsigned char)(coordinate >> 8U);

                int_sum_xx += x * x;
                int_sum_yy += y * y;
                int_sum_xy += x * y;
            }
            sum_xx = (float)int_sum_xx;
            sum_yy = (float)int_sum_yy;
            sum_xy = (float)int_sum_xy;
        }
        else
        {
            head = g_queue;
            moment_end = g_queue + moment_count;
            while(head < moment_end)
            {
                unsigned short coordinate = *head++;
                unsigned char x = (unsigned char)coordinate;
                unsigned char y = (unsigned char)(coordinate >> 8U);

                sum_xx += (float)x * (float)x;
                sum_yy += (float)y * (float)y;
                sum_xy += (float)x * (float)y;
            }
        }
        var_x = sum_xx * inv_area - comp.cx * comp.cx;
        var_y = sum_yy * inv_area - comp.cy * comp.cy;
        cov_xy = sum_xy * inv_area - comp.cx * comp.cy;
        if(orientation != 0)
        {
            orientation->numerator = 2.0f * cov_xy;
            orientation->denominator = var_x - var_y;
        }
        trace = var_x + var_y;
        det = var_x * var_y - cov_xy * cov_xy;
        disc = trace * trace * 0.25f - det;
        if(disc < 0.0f) disc = 0.0f;
        eig_delta = sqrtf(disc);
        eig_major = trace * 0.5f + eig_delta;
        eig_minor = trace * 0.5f - eig_delta;
        if(eig_minor < 0.0f) eig_minor = 0.0f;

        comp.major = 4.0f * sqrtf(eig_major + 0.0001f);
        comp.minor = 4.0f * sqrtf(eig_minor + 0.0001f);
        if(comp.minor < 1.0f) comp.minor = 1.0f;
        comp.elongation = comp.major / comp.minor;
    }

    return comp;
}

#if defined(__ICCARM__)
#pragma inline=never
#endif
static component_t grow_component_visit(
    unsigned char start_x,
    unsigned char start_y,
    int shape_min_area,
    int shape_max_area,
    component_orientation_t *orientation)
{
    return grow_component_core(
        start_x, start_y, shape_min_area, shape_max_area, orientation, 0U);
}

#if defined(__ICCARM__)
#pragma inline=never
#endif
static component_t grow_component_consume(
    unsigned char start_x,
    unsigned char start_y,
    int shape_min_area,
    int shape_max_area,
    component_orientation_t *orientation)
{
    return grow_component_core(
        start_x, start_y, shape_min_area, shape_max_area, orientation, 1U);
}

#if defined(__ICCARM__)
#pragma inline=never
#endif
static float component_orientation_angle(
    const component_orientation_t *orientation)
{
    return 0.5f * atan2f(
        orientation->numerator, orientation->denominator) * 180.0f / PI_F;
}

static unsigned char car_lamp_below_horizon(const component_t *comp)
{
    int min_x;
    int max_x;
    int x;
    float minimum_horizon = (float)BEACON_IMAGE_H;
    unsigned char found = 0U;

    if((comp == 0) || (g_image_horizon_valid == 0U))
    {
        return 1U;
    }
    min_x = (comp->min_x < 0) ? 0 : comp->min_x;
    max_x = (comp->max_x >= BEACON_IMAGE_W) ?
            (BEACON_IMAGE_W - 1) : comp->max_x;
    for(x = min_x; x <= max_x; x++)
    {
        float horizon_y;
        if((image_horizon_get_y((uint16)x, &horizon_y) != 0U) &&
           ((found == 0U) || (horizon_y < minimum_horizon)))
        {
            minimum_horizon = horizon_y;
            found = 1U;
        }
    }
    return ((found == 0U) ||
            ((float)comp->max_y + CAR_LAMP_HORIZON_MARGIN_PX >=
             minimum_horizon)) ? 1U : 0U;
}

static unsigned char is_lamp_geometry_candidate(const component_t *comp)
{
    if((comp == 0) || (comp->valid == 0) ||
       (comp->min_y <= COMPONENT_TOP_REJECT_Y) ||
       (comp->area < CAR_LAMP_MIN_AREA) ||
       (comp->area > CAR_LAMP_MAX_AREA) ||
       (comp->elongation < CAR_LAMP_MIN_ELONGATION) ||
       (comp->major < CAR_LAMP_MIN_LENGTH) ||
       (car_lamp_below_horizon(comp) == 0U))
    {
        return 0U;
    }
    if(comp->cy >= (float)CAR_LAMP_MIN_CENTER_Y)
    {
        if(comp->minor >= CAR_LAMP_MIN_WIDTH)
        {
            return 1U;
        }

        return ((comp->minor >= CAR_LAMP_NARROW_MIN_WIDTH) &&
                (comp->elongation >=
                CAR_LAMP_NARROW_MIN_ELONGATION)) ? 1U : 0U;
    }

    if((comp->cy >= CAR_LAMP_UPPER_COMPACT_MIN_Y) &&
       (comp->area >= CAR_LAMP_UPPER_COMPACT_MIN_AREA) &&
       (comp->major >= CAR_LAMP_UPPER_COMPACT_MIN_LENGTH) &&
       (comp->minor >= CAR_LAMP_UPPER_COMPACT_MIN_WIDTH) &&
       (comp->elongation >= CAR_LAMP_UPPER_COMPACT_MIN_ELONGATION))
    {
        return 1U;
    }

    return ((comp->area >= CAR_LAMP_UPPER_MIN_AREA) &&
            (comp->major >= CAR_LAMP_UPPER_MIN_LENGTH) &&
            (comp->minor >= CAR_LAMP_UPPER_MIN_WIDTH)) ? 1U : 0U;
}

static unsigned char car_lamp_uniform_strip_valid(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp)
{
    unsigned int sum = 0U;
    unsigned int sum_sq = 0U;
    int count = 0;
    int full_count = 0;
    int saturated = 0;
    int threshold;
    int x;
    int y;
    float angle;
    float cos_a;
    float sin_a;
    float half_length;
    float half_width;
    float mean;
    float variance;
    int min_x;
    int max_x;
    int min_y;
    int max_y;

    if((image == 0) || (comp == 0) || (comp->valid == 0U))
    {
        return 0U;
    }
    if(((comp->min_x <= 0) || (comp->max_x >= BEACON_IMAGE_W - 1)) &&
       (fabsf(comp->angle) >= 60.0f))
    {
        return 0U;
    }
    angle = comp->angle * (PI_F / 180.0f);
    cos_a = cosf(angle);
    sin_a = sinf(angle);
    half_length = comp->major * 0.5f + 0.5f;
    half_width = comp->minor * 0.5f + 0.5f;
    min_x = (int)floorf(comp->cx - half_length - half_width);
    max_x = (int)ceilf(comp->cx + half_length + half_width);
    min_y = (int)floorf(comp->cy - half_length - half_width);
    max_y = (int)ceilf(comp->cy + half_length + half_width);
    if(min_x < 0) min_x = 0;
    if(min_y < 0) min_y = 0;
    if(max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if(max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;
    threshold = (comp->cy < CAR_LAMP_UPPER_Y) ?
                CAR_LAMP_UPPER_THRESHOLD : CAR_LAMP_BINARY_THRESHOLD;
    for(y = min_y; y <= max_y; y++)
    {
        float dy = (float)y - comp->cy;
        float dy_sin = dy * sin_a;
        float dy_cos = dy * cos_a;

        for(x = min_x; x <= max_x; x++)
        {
            float dx = (float)x - comp->cx;
            float major = dx * cos_a + dy_sin;
            float minor = -dx * sin_a + dy_cos;
            unsigned int gray;

            if((fabsf(major) > half_length) ||
               (fabsf(minor) > half_width))
            {
                continue;
            }
            gray = image[y][x];
            full_count++;
            if(gray < (unsigned int)threshold)
            {
                continue;
            }
            count++;
            if(gray >= 240U)
            {
                sum += gray;
                sum_sq += gray * gray;
                saturated++;
            }
        }
    }
    if((count == 0) || (full_count == 0) ||
       (count * 100 < full_count * CAR_LAMP_STRIP_MIN_FILL_PERCENT) ||
       (saturated * 2 < count))
    {
        return 0U;
    }
    mean = (float)sum / (float)saturated;
    variance = (float)sum_sq / (float)saturated - mean * mean;
    return ((mean >= 245.0f) && (variance <= 100.0f)) ? 1U : 0U;
}

static unsigned char find_car_lamp(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    component_t *best_lamp)
{
    unsigned char x;
    unsigned char y;
    float best_score = 0.0f;
    unsigned char found = 0U;

    memset(best_lamp, 0, sizeof(*best_lamp));

    for(y = 0; y < BEACON_IMAGE_H; y++)
    {
        for(x = 0; x < BEACON_IMAGE_W; x++)
        {
            component_t comp;
            component_orientation_t orientation;
            float score;

#if defined(__ICCARM__)
            if(((x & 3U) == 0U) &&
               (__UNALIGNED_UINT32_READ(&g_binary[y][x]) == 0U))
            {
                x += 3U;
                continue;
            }
#endif
            if(g_binary[y][x] == 0U)
            {
                continue;
            }

            comp = grow_component_consume(
                x, y, CAR_LAMP_MIN_AREA, CAR_LAMP_MAX_AREA, &orientation);
            if(is_lamp_geometry_candidate(&comp) == 0U)
            {
                continue;
            }
            comp.angle = component_orientation_angle(&orientation);
            if(car_lamp_uniform_strip_valid(image, &comp) == 0U)
            {
                continue;
            }

            score = (float)comp.area * comp.elongation;
            if((found == 0U) ||
               (comp.elongation > best_lamp->elongation + 0.5f) ||
               ((fabsf(comp.elongation - best_lamp->elongation) <= 0.5f) &&
                ((comp.cy > best_lamp->cy + CAR_LAMP_Y_PRIORITY_MARGIN) ||
                 ((fabsf(comp.cy - best_lamp->cy) <=
                   CAR_LAMP_Y_PRIORITY_MARGIN) &&
                  (score > best_score)))))
            {
                *best_lamp = comp;
                best_score = score;
                found = 1U;
            }
        }
    }

    return found;
}

static float car_lamp_angle_difference(float left, float right)
{
    float difference = fabsf(left - right);

    while(difference >= 180.0f)
    {
        difference -= 180.0f;
    }
    if(difference > 90.0f)
    {
        difference = 180.0f - difference;
    }
    return difference;
}

static unsigned char car_lamp_pair_compatible(
    const component_t *primary,
    const component_t *candidate)
{
    float longer;
    float shorter;
    float dx;
    float dy;

    if((primary == 0) || (candidate == 0) ||
       (primary->valid == 0U) || (candidate->valid == 0U) ||
       (primary->major <= 0.0f) || (candidate->major <= 0.0f))
    {
        return 0U;
    }
    if(car_lamp_angle_difference(primary->angle, candidate->angle) >
       CAR_LAMP_PAIR_MAX_ANGLE_DEG)
    {
        return 0U;
    }
    longer = (primary->major >= candidate->major) ?
             primary->major : candidate->major;
    shorter = (primary->major < candidate->major) ?
              primary->major : candidate->major;
    if((shorter <= 0.0f) ||
       (longer / shorter > CAR_LAMP_PAIR_MAX_LENGTH_RATIO))
    {
        return 0U;
    }
    if(!((primary->max_x < candidate->min_x) ||
         (candidate->max_x < primary->min_x) ||
         (primary->max_y < candidate->min_y) ||
         (candidate->max_y < primary->min_y)))
    {
        return 0U;
    }
    dx = primary->cx - candidate->cx;
    dy = primary->cy - candidate->cy;
    return ((dx * dx + dy * dy) <=
            longer * longer * CAR_LAMP_PAIR_MAX_DISTANCE_SCALE *
            CAR_LAMP_PAIR_MAX_DISTANCE_SCALE) ? 1U : 0U;
}

static float car_body_cross(
    const car_body_point_t *origin,
    const car_body_point_t *left,
    const car_body_point_t *right)
{
    return (left->x - origin->x) * (right->y - origin->y) -
           (left->y - origin->y) * (right->x - origin->x);
}

static float car_body_point_distance2(
    const car_body_point_t *left,
    const car_body_point_t *right)
{
    float dx = left->x - right->x;
    float dy = left->y - right->y;
    return dx * dx + dy * dy;
}

static void car_body_rect_corners(
    const component_t *lamp,
    car_body_point_t corners[4])
{
    float angle = lamp->angle * (PI_F / 180.0f);
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);
    /* 连接遮罩只覆盖两灯板真实边界之间，单灯外扩由原遮罩独立负责。 */
    float half_length = lamp->major * 0.5f;
    float half_width = lamp->minor * 0.5f;
    float major_x = cos_angle * half_length;
    float major_y = sin_angle * half_length;
    float minor_x = -sin_angle * half_width;
    float minor_y = cos_angle * half_width;

    corners[0].x = lamp->cx - major_x - minor_x;
    corners[0].y = lamp->cy - major_y - minor_y;
    corners[1].x = lamp->cx + major_x - minor_x;
    corners[1].y = lamp->cy + major_y - minor_y;
    corners[2].x = lamp->cx + major_x + minor_x;
    corners[2].y = lamp->cy + major_y + minor_y;
    corners[3].x = lamp->cx - major_x + minor_x;
    corners[3].y = lamp->cy - major_y + minor_y;
}

static unsigned char car_body_pair_compatible(
    const component_t *primary,
    const component_t *secondary)
{
    float longer;
    float shorter;
    float dx;
    float dy;

    if((primary == 0) || (secondary == 0) ||
       (primary->valid == 0U) || (secondary->valid == 0U) ||
       (primary->major <= 0.0f) || (secondary->major <= 0.0f))
    {
        return 0U;
    }
    if(car_lamp_angle_difference(primary->angle, secondary->angle) >
       CAR_LAMP_PAIR_MAX_ANGLE_DEG)
    {
        return 0U;
    }
    longer = (primary->major >= secondary->major) ?
             primary->major : secondary->major;
    shorter = (primary->major < secondary->major) ?
              primary->major : secondary->major;
    if((shorter <= 0.0f) ||
       (longer / shorter > CAR_LAMP_PAIR_MAX_LENGTH_RATIO))
    {
        return 0U;
    }
    dx = primary->cx - secondary->cx;
    dy = primary->cy - secondary->cy;
    return ((dx * dx + dy * dy) <=
            longer * longer * CAR_LAMP_PAIR_MAX_DISTANCE_SCALE *
            CAR_LAMP_PAIR_MAX_DISTANCE_SCALE) ? 1U : 0U;
}

static void car_body_mask_build(
    const component_t *primary,
    const component_t *secondary)
{
    car_body_point_t points[CAR_BODY_MASK_CORNER_COUNT];
    car_body_point_t hull[CAR_BODY_MASK_CORNER_COUNT];
    unsigned char hull_count = 0U;
    unsigned char leftmost = 0U;
    unsigned char current;
    unsigned char index;
    int y;

    g_car_body_mask.valid = 0U;
    if((g_car_lamp_mode != CAR_LAMP_MODE_DUAL) ||
       (car_body_pair_compatible(primary, secondary) == 0U))
    {
        return;
    }

    car_body_rect_corners(primary, &points[0]);
    car_body_rect_corners(secondary, &points[4]);
    for(index = 1U; index < CAR_BODY_MASK_CORNER_COUNT; index++)
    {
        if((points[index].x < points[leftmost].x) ||
           ((points[index].x == points[leftmost].x) &&
            (points[index].y < points[leftmost].y)))
        {
            leftmost = index;
        }
    }

    current = leftmost;
    do
    {
        unsigned char next = (unsigned char)(
            (current + 1U) % CAR_BODY_MASK_CORNER_COUNT);

        hull[hull_count++] = points[current];
        for(index = 0U; index < CAR_BODY_MASK_CORNER_COUNT; index++)
        {
            float cross;

            if(index == current)
            {
                continue;
            }
            cross = car_body_cross(
                &points[current], &points[next], &points[index]);
            if((cross < 0.0f) ||
               ((fabsf(cross) < 0.0001f) &&
                (car_body_point_distance2(
                     &points[current], &points[index]) >
                 car_body_point_distance2(
                     &points[current], &points[next]))))
            {
                next = index;
            }
        }
        current = next;
    }
    while((current != leftmost) &&
          (hull_count < CAR_BODY_MASK_CORNER_COUNT));

    if(hull_count < 3U)
    {
        return;
    }
    for(y = 0; y < BEACON_IMAGE_H; y++)
    {
        float scan_y = (float)y + 0.5f;
        float min_x = (float)BEACON_IMAGE_W;
        float max_x = -1.0f;
        unsigned char intersection_count = 0U;

        g_car_body_mask.min_x[y] = 1;
        g_car_body_mask.max_x[y] = 0;
        for(index = 0U; index < hull_count; index++)
        {
            const car_body_point_t *start = &hull[index];
            const car_body_point_t *end = &hull[
                (unsigned char)((index + 1U) % hull_count)];

            if(((start->y <= scan_y) && (end->y > scan_y)) ||
               ((end->y <= scan_y) && (start->y > scan_y)))
            {
                float x = start->x +
                    (scan_y - start->y) * (end->x - start->x) /
                    (end->y - start->y);

                if(x < min_x) min_x = x;
                if(x > max_x) max_x = x;
                intersection_count++;
            }
        }
        if(intersection_count >= 2U)
        {
            int row_min_x = (int)floorf(min_x) - 1;
            int row_max_x = (int)ceilf(max_x) + 1;

            if(row_min_x < 0) row_min_x = 0;
            if(row_max_x >= BEACON_IMAGE_W)
            {
                row_max_x = BEACON_IMAGE_W - 1;
            }
            if(row_min_x <= row_max_x)
            {
                g_car_body_mask.min_x[y] = (signed short)row_min_x;
                g_car_body_mask.max_x[y] = (signed short)row_max_x;
                g_car_body_mask.valid = 1U;
            }
        }
    }
}

static unsigned char car_body_mask_contains(int x, int y)
{
    if((g_car_body_mask.valid == 0U) ||
       (x < 0) || (x >= BEACON_IMAGE_W) ||
       (y < 0) || (y >= BEACON_IMAGE_H))
    {
        return 0U;
    }
    return ((x >= g_car_body_mask.min_x[y]) &&
            (x <= g_car_body_mask.max_x[y])) ? 1U : 0U;
}

static void erase_car_body_from_binary(void)
{
    int y;

    if(g_car_body_mask.valid == 0U)
    {
        return;
    }
    for(y = 0; y < BEACON_IMAGE_H; y++)
    {
        int min_x = g_car_body_mask.min_x[y];
        int max_x = g_car_body_mask.max_x[y];

        if(min_x <= max_x)
        {
            memset(&g_binary[y][min_x], 0,
                   (size_t)(max_x - min_x + 1));
        }
    }
}

static unsigned char find_compatible_car_lamp(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *primary,
    component_t *best_lamp)
{
    unsigned char x;
    unsigned char y;
    float best_score = 0.0f;
    unsigned char found = 0U;

    if((image == 0) || (primary == 0) || (best_lamp == 0) ||
       (primary->valid == 0U))
    {
        return 0U;
    }
    memset(best_lamp, 0, sizeof(*best_lamp));
    (void)threshold_car_lamp_image(image);
    for(y = 0U; y < BEACON_IMAGE_H; y++)
    {
        for(x = 0U; x < BEACON_IMAGE_W; x++)
        {
            component_t comp;
            component_orientation_t orientation;
            float score;

            if(g_binary[y][x] == 0U)
            {
                continue;
            }
            comp = grow_component_consume(
                x, y, CAR_LAMP_MIN_AREA, CAR_LAMP_MAX_AREA, &orientation);
            if(is_lamp_geometry_candidate(&comp) == 0U)
            {
                continue;
            }
            comp.angle = component_orientation_angle(&orientation);
            if((car_lamp_uniform_strip_valid(image, &comp) == 0U) ||
               (car_lamp_pair_compatible(primary, &comp) == 0U))
            {
                continue;
            }
            score = (float)comp.area * comp.elongation;
            if((found == 0U) ||
               (comp.elongation > best_lamp->elongation + 0.5f) ||
               ((fabsf(comp.elongation - best_lamp->elongation) <= 0.5f) &&
                ((comp.cy > best_lamp->cy + CAR_LAMP_Y_PRIORITY_MARGIN) ||
                 ((fabsf(comp.cy - best_lamp->cy) <=
                   CAR_LAMP_Y_PRIORITY_MARGIN) &&
                  (score > best_score)))))
            {
                *best_lamp = comp;
                best_score = score;
                found = 1U;
            }
        }
    }
    return found;
}

static void write_car_lamp(const component_t *lamp, beacon_result_t *result)
{
    if((lamp == 0) || (lamp->valid == 0))
    {
        result->car_lamp_count = 0;
        return;
    }

    result->car_lamps[0].cx = lamp->cx - (float)BEACON_IMAGE_W * 0.5f;
    result->car_lamps[0].cy = lamp->cy - (float)BEACON_IMAGE_H * 0.5f;
    result->car_lamps[0].width = lamp->minor;
    result->car_lamps[0].length = lamp->major;
    result->car_lamps[0].angle = lamp->angle;
    result->car_lamps[0].valid = 1;
    result->car_lamp_count = 1;
    result->car_lamp_measured_mask = 0x01U;
}

static void append_car_lamp(const component_t *lamp, beacon_result_t *result)
{
    beacon_rect_t *target;

    if((lamp == 0) || (lamp->valid == 0U) ||
       (result == 0) ||
       (result->car_lamp_count >= BEACON_MAX_CAR_LAMP_COUNT))
    {
        return;
    }
    target = &result->car_lamps[result->car_lamp_count];
    target->cx = lamp->cx - (float)BEACON_IMAGE_W * 0.5f;
    target->cy = lamp->cy - (float)BEACON_IMAGE_H * 0.5f;
    target->width = lamp->minor;
    target->length = lamp->major;
    target->angle = lamp->angle;
    target->valid = 1U;
    result->car_lamp_measured_mask |=
        (unsigned char)(1U << result->car_lamp_count);
    result->car_lamp_count++;
}

static void write_car_lamps(
    const component_t lamps[BEACON_MAX_CAR_LAMP_COUNT],
    unsigned char lamp_count,
    beacon_result_t *result)
{
    unsigned char index;

    memset(result->car_lamps, 0, sizeof(result->car_lamps));
    result->car_lamp_count = 0U;
    result->car_lamp_measured_mask = 0U;
    for(index = 0U;
        (index < lamp_count) && (index < BEACON_MAX_CAR_LAMP_COUNT);
        index++)
    {
        append_car_lamp(&lamps[index], result);
    }
}

static void erase_lamp_from_binary(const component_t *lamp)
{
    int x;
    int y;
    int min_x;
    int max_x;
    int min_y;
    int max_y;

    if((lamp == 0) || (lamp->valid == 0))
    {
        return;
    }

    min_x = lamp->min_x - LAMP_MASK_PAD;
    max_x = lamp->max_x + LAMP_MASK_PAD;
    min_y = lamp->min_y - LAMP_MASK_PAD;
    max_y = lamp->max_y + LAMP_MASK_DOWN_PAD;
    if(min_x < 0) min_x = 0;
    if(min_y < 0) min_y = 0;
    if(max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if(max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;

    for(y = min_y; y <= max_y; y++)
    {
        for(x = min_x; x <= max_x; x++)
        {
            g_binary[y][x] = 0;
        }
    }
}

static unsigned char component_from_temporal_car(
    const temporal_track_t *track,
    component_t *lamp,
    unsigned char predicted)
{
    float image_cx;
    float image_cy;
    float half_len;
    float half_wid;
    float radius;

    if((track == 0) || (lamp == 0) || (track->confirmed == 0U) ||
       (track->length <= 0.0f) || (track->width <= 0.0f))
    {
        return 0;
    }

    memset(lamp, 0, sizeof(*lamp));
    image_cx = track->x + (float)BEACON_IMAGE_W * 0.5f;
    image_cy = track->y + (float)BEACON_IMAGE_H * 0.5f;
    if(predicted != 0U)
    {
        image_cx += track->vx;
        image_cy += track->vy;
    }

    half_len = track->length * 0.5f + (float)CAR_LAMP_TEMPORAL_MASK_PAD;
    half_wid = track->width * 0.5f + (float)CAR_LAMP_TEMPORAL_MASK_PAD;
    radius = sqrtf(half_len * half_len + half_wid * half_wid);
    if((isfinite(image_cx) == 0) ||
       (isfinite(image_cy) == 0) ||
       (isfinite(radius) == 0) ||
       (isfinite(track->angle) == 0) ||
       (image_cx + radius < 0.0f) ||
       (image_cy + radius < 0.0f) ||
       (image_cx - radius >= (float)BEACON_IMAGE_W) ||
       (image_cy - radius >= (float)BEACON_IMAGE_H))
    {
        return 0;
    }

    lamp->cx = image_cx;
    lamp->cy = image_cy;
    lamp->major = track->length;
    lamp->minor = track->width;
    lamp->angle = track->angle;
    lamp->area = (int)(track->length * track->width + 0.5f);
    lamp->min_x = (int)(image_cx - radius);
    lamp->max_x = (int)(image_cx + radius);
    lamp->min_y = (int)(image_cy - radius);
    lamp->max_y = (int)(image_cy + radius);
    if(lamp->min_x < 0) lamp->min_x = 0;
    if(lamp->min_y < 0) lamp->min_y = 0;
    if(lamp->max_x >= BEACON_IMAGE_W) lamp->max_x = BEACON_IMAGE_W - 1;
    if(lamp->max_y >= BEACON_IMAGE_H) lamp->max_y = BEACON_IMAGE_H - 1;
    if((lamp->min_x > lamp->max_x) || (lamp->min_y > lamp->max_y))
    {
        memset(lamp, 0, sizeof(*lamp));
        return 0;
    }
    lamp->valid = 1;
    return 1;
}

static void erase_temporal_lamp_from_binary(const component_t *lamp)
{
    int x;
    int y;
    float angle;
    float cos_a;
    float sin_a;
    float half_len;
    float half_wid;

    if((lamp == 0) || (lamp->valid == 0) ||
       (lamp->min_x < 0) || (lamp->min_y < 0) ||
       (lamp->max_x >= BEACON_IMAGE_W) ||
       (lamp->max_y >= BEACON_IMAGE_H) ||
       (lamp->min_x > lamp->max_x) || (lamp->min_y > lamp->max_y))
    {
        return;
    }

    angle = lamp->angle * (PI_F / 180.0f);
    cos_a = cosf(angle);
    sin_a = sinf(angle);
    half_len = lamp->major * 0.5f + (float)CAR_LAMP_TEMPORAL_MASK_PAD;
    half_wid = lamp->minor * 0.5f + (float)CAR_LAMP_TEMPORAL_MASK_PAD;

    for(y = lamp->min_y; y <= lamp->max_y; y++)
    {
        float dy = (float)y - lamp->cy;
        float dy_sin = dy * sin_a;
        float dy_cos = dy * cos_a;

        for(x = lamp->min_x; x <= lamp->max_x; x++)
        {
            float dx = (float)x - lamp->cx;
            float major = dx * cos_a + dy_sin;
            float minor = -dx * sin_a + dy_cos;

            if((fabsf(major) <= half_len) && (fabsf(minor) <= half_wid))
            {
                g_binary[y][x] = 0;
            }
        }
    }
}

static inline int gray_reflect_index(int index, int limit)
{
    if(index < 0)
    {
        return -index;
    }
    if(index >= limit)
    {
        return limit * 2 - 2 - index;
    }
    return index;
}

static inline unsigned char gray_pixel(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int x,
    int y)
{
    return image[gray_reflect_index(y, BEACON_IMAGE_H)]
                [gray_reflect_index(x, BEACON_IMAGE_W)];
}

static inline void gray_vertical_box_sum_reflect_edges(void)
{
    unsigned short *box3_sum = &g_gray_box3_storage[2];
    unsigned short *box9_sum = &g_gray_box9_storage[4];
    int x;

    box3_sum[-1] = box3_sum[1];
    box3_sum[BEACON_IMAGE_W] = box3_sum[BEACON_IMAGE_W - 2];
    for(x = 1; x <= 4; x++)
    {
        box9_sum[-x] = box9_sum[x];
        box9_sum[BEACON_IMAGE_W - 1 + x] =
            box9_sum[BEACON_IMAGE_W - 1 - x];
    }
}

static inline void gray_vertical_box_sum_init_at(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int center_y)
{
    unsigned short *box3_sum = &g_gray_box3_storage[2];
    unsigned short *box9_sum = &g_gray_box9_storage[4];
    int x;

#if defined(__ICCARM__)
    if((center_y >= 4) && (center_y <= BEACON_IMAGE_H - 5))
    {
        const unsigned char *row_m4 = &image[center_y - 4][0];

        for(x = 0; x < BEACON_IMAGE_W; x += 4)
        {
            uint32 word = __UNALIGNED_UINT32_READ(&row_m4[x]);
            uint32 box9_even = __UXTB16(word);
            uint32 box9_odd = __UXTB16(__ROR(word, 8U));
            uint32 box3_even;
            uint32 box3_odd;
            uint32 even;
            uint32 odd;

            word = __UNALIGNED_UINT32_READ(
                &row_m4[BEACON_IMAGE_W + x]);
            box9_even = __UADD16(box9_even, __UXTB16(word));
            box9_odd = __UADD16(box9_odd, __UXTB16(__ROR(word, 8U)));
            word = __UNALIGNED_UINT32_READ(
                &row_m4[2 * BEACON_IMAGE_W + x]);
            box9_even = __UADD16(box9_even, __UXTB16(word));
            box9_odd = __UADD16(box9_odd, __UXTB16(__ROR(word, 8U)));
            word = __UNALIGNED_UINT32_READ(
                &row_m4[3 * BEACON_IMAGE_W + x]);
            box3_even = __UXTB16(word);
            box3_odd = __UXTB16(__ROR(word, 8U));
            box9_even = __UADD16(box9_even, box3_even);
            box9_odd = __UADD16(box9_odd, box3_odd);
            word = __UNALIGNED_UINT32_READ(
                &row_m4[4 * BEACON_IMAGE_W + x]);
            even = __UXTB16(word);
            odd = __UXTB16(__ROR(word, 8U));
            box3_even = __UADD16(box3_even, even);
            box3_odd = __UADD16(box3_odd, odd);
            box9_even = __UADD16(box9_even, even);
            box9_odd = __UADD16(box9_odd, odd);
            word = __UNALIGNED_UINT32_READ(
                &row_m4[5 * BEACON_IMAGE_W + x]);
            even = __UXTB16(word);
            odd = __UXTB16(__ROR(word, 8U));
            box3_even = __UADD16(box3_even, even);
            box3_odd = __UADD16(box3_odd, odd);
            box9_even = __UADD16(box9_even, even);
            box9_odd = __UADD16(box9_odd, odd);

            __UNALIGNED_UINT32_WRITE(
                &box3_sum[x], __PKHBT(box3_even, box3_odd, 16));
            __UNALIGNED_UINT32_WRITE(
                &box3_sum[x + 2], __PKHTB(box3_odd, box3_even, 16));

            word = __UNALIGNED_UINT32_READ(
                &row_m4[6 * BEACON_IMAGE_W + x]);
            box9_even = __UADD16(box9_even, __UXTB16(word));
            box9_odd = __UADD16(box9_odd, __UXTB16(__ROR(word, 8U)));
            word = __UNALIGNED_UINT32_READ(
                &row_m4[7 * BEACON_IMAGE_W + x]);
            box9_even = __UADD16(box9_even, __UXTB16(word));
            box9_odd = __UADD16(box9_odd, __UXTB16(__ROR(word, 8U)));
            word = __UNALIGNED_UINT32_READ(
                &row_m4[8 * BEACON_IMAGE_W + x]);
            box9_even = __UADD16(box9_even, __UXTB16(word));
            box9_odd = __UADD16(box9_odd, __UXTB16(__ROR(word, 8U)));

            __UNALIGNED_UINT32_WRITE(
                &box9_sum[x], __PKHBT(box9_even, box9_odd, 16));
            __UNALIGNED_UINT32_WRITE(
                &box9_sum[x + 2], __PKHTB(box9_odd, box9_even, 16));
        }
    }
    else
#endif
    {
        const unsigned char *row_m4 =
            image[gray_reflect_index(center_y - 4, BEACON_IMAGE_H)];
        const unsigned char *row_m3 =
            image[gray_reflect_index(center_y - 3, BEACON_IMAGE_H)];
        const unsigned char *row_m2 =
            image[gray_reflect_index(center_y - 2, BEACON_IMAGE_H)];
        const unsigned char *row_m1 =
            image[gray_reflect_index(center_y - 1, BEACON_IMAGE_H)];
        const unsigned char *row_0 = image[center_y];
        const unsigned char *row_p1 =
            image[gray_reflect_index(center_y + 1, BEACON_IMAGE_H)];
        const unsigned char *row_p2 =
            image[gray_reflect_index(center_y + 2, BEACON_IMAGE_H)];
        const unsigned char *row_p3 =
            image[gray_reflect_index(center_y + 3, BEACON_IMAGE_H)];
        const unsigned char *row_p4 =
            image[gray_reflect_index(center_y + 4, BEACON_IMAGE_H)];

        for(x = 0; x < BEACON_IMAGE_W; x++)
        {
            box3_sum[x] =
                (unsigned short)((unsigned int)row_m1[x] +
                                 (unsigned int)row_0[x] +
                                 (unsigned int)row_p1[x]);
            box9_sum[x] =
                (unsigned short)((unsigned int)row_m4[x] +
                                 (unsigned int)row_m3[x] +
                                 (unsigned int)row_m2[x] +
                                 (unsigned int)row_m1[x] +
                                 (unsigned int)row_0[x] +
                                 (unsigned int)row_p1[x] +
                                 (unsigned int)row_p2[x] +
                                 (unsigned int)row_p3[x] +
                                 (unsigned int)row_p4[x]);
        }
    }
    gray_vertical_box_sum_reflect_edges();
}

static inline void gray_vertical_box_sum_update4(
    unsigned short *sum,
    const unsigned char *sub_row,
    const unsigned char *add_row,
    int x)
{
#if defined(__ICCARM__)
    uint32 current01 = __UNALIGNED_UINT32_READ(&sum[x]);
    uint32 current23 = __UNALIGNED_UINT32_READ(&sum[x + 2]);
    uint32 sub_word = __UNALIGNED_UINT32_READ(&sub_row[x]);
    uint32 add_word = __UNALIGNED_UINT32_READ(&add_row[x]);
    uint32 even = __PKHBT(current01, current23, 16);
    uint32 odd = __PKHTB(current23, current01, 16);

    even = __USUB16(even, __UXTB16(sub_word));
    even = __UADD16(even, __UXTB16(add_word));
    odd = __USUB16(odd, __UXTB16(__ROR(sub_word, 8U)));
    odd = __UADD16(odd, __UXTB16(__ROR(add_word, 8U)));

    current01 = __PKHBT(even, odd, 16);
    current23 = __PKHTB(odd, even, 16);
    __UNALIGNED_UINT32_WRITE(&sum[x], current01);
    __UNALIGNED_UINT32_WRITE(&sum[x + 2], current23);
#else
    int offset;

    for(offset = 0; offset < 4; offset++)
    {
        sum[x + offset] =
            (unsigned short)(sum[x + offset] - sub_row[x + offset] +
                             add_row[x + offset]);
    }
#endif
}

static void gray_insert_peak(
    gray_beacon_peak_t peaks[GRAY_BEACON_MAX_PEAKS],
    unsigned char *count,
    signed short response,
    int x,
    int y)
{
    int position;
    int last;

    if((count == 0) || (response <= 0))
    {
        return;
    }
    last = *count;
    if(last >= GRAY_BEACON_MAX_PEAKS)
    {
        last = GRAY_BEACON_MAX_PEAKS - 1;
        if(response <= peaks[last].response)
        {
            return;
        }
    }
    else
    {
        (*count)++;
    }

    position = last;
    while((position > 0) &&
          (response > peaks[position - 1].response))
    {
        peaks[position] = peaks[position - 1];
        position--;
    }
    peaks[position].response = response;
    peaks[position].x = (unsigned char)x;
    peaks[position].y = (unsigned char)y;
}

static unsigned char gray_point_below_horizon(int x, int y)
{
    if((g_image_horizon_valid == 0U) ||
       (x < 0) || (x >= BEACON_IMAGE_W) ||
       (g_image_horizon_column_valid[x] == 0U))
    {
        return 1U;
    }
    return ((float)y >=
            g_image_horizon_y[x] - BEACON_HORIZON_TOLERANCE_PX) ? 1U : 0U;
}

#if defined(__ICCARM__)
#pragma inline=never
#endif
static unsigned char gray_find_box_peaks(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *lamp,
    const component_t *temporal_lamp,
    gray_beacon_peak_t peaks[GRAY_BEACON_MAX_PEAKS])
{
    gray_lamp_geometry_t lamp_geometry;
    gray_lamp_geometry_t temporal_lamp_geometry;
    gray_lamp_geometry_t secondary_lamp_geometry;
    gray_lamp_geometry_t secondary_temporal_lamp_geometry;
    unsigned char horizon_min_y[BEACON_IMAGE_W];
    unsigned char count = 0U;
    unsigned short *box3_vertical = &g_gray_box3_storage[2];
    unsigned short *box9_vertical = &g_gray_box9_storage[4];
    int minimum_candidate_y = BEACON_IMAGE_H;
    int response_start_y;
    int x;
    int y;

    gray_lamp_geometry_init(lamp, &lamp_geometry);
    gray_lamp_geometry_init(temporal_lamp, &temporal_lamp_geometry);
    gray_lamp_geometry_init(&g_secondary_lamp_mask,
                            &secondary_lamp_geometry);
    gray_lamp_geometry_init(&g_secondary_temporal_lamp_mask,
                            &secondary_temporal_lamp_geometry);

    for(x = 0; x < BEACON_IMAGE_W; x++)
    {
        int minimum_y = 0;

        if((g_image_horizon_valid != 0U) &&
           (g_image_horizon_column_valid[x] != 0U))
        {
            float threshold =
                g_image_horizon_y[x] - BEACON_HORIZON_TOLERANCE_PX;

            minimum_y = (int)threshold;
            if((float)minimum_y < threshold)
            {
                minimum_y++;
            }
            if(minimum_y < 0)
            {
                minimum_y = 0;
            }
            else if(minimum_y > BEACON_IMAGE_H)
            {
                minimum_y = BEACON_IMAGE_H;
            }
        }
        horizon_min_y[x] = (unsigned char)minimum_y;
    }
    for(x = 2; x <= BEACON_IMAGE_W - 3; x++)
    {
        if((int)horizon_min_y[x] < minimum_candidate_y)
        {
            minimum_candidate_y = horizon_min_y[x];
        }
    }
    if(minimum_candidate_y < 2)
    {
        minimum_candidate_y = 2;
    }
    response_start_y = (minimum_candidate_y <= BEACON_IMAGE_H - 3) ?
        (minimum_candidate_y - 1) : BEACON_IMAGE_H;

    if(response_start_y >= BEACON_IMAGE_H)
    {
        return 0U;
    }
    gray_vertical_box_sum_init_at(image, response_start_y);

    for(y = response_start_y; y <= BEACON_IMAGE_H - 2; y++)
    {
        int response_slot = y % 3;
        signed short *bottom_response =
            g_gray_response_rows[response_slot];
        const signed short *top_response = 0;
        const signed short *center_response = 0;
        const unsigned char *gray_row = 0;
        int center_y = y - 1;

        if(center_y >= minimum_candidate_y)
        {
            top_response = g_gray_response_rows[(y - 2) % 3];
            center_response = g_gray_response_rows[(y - 1) % 3];
            gray_row = image[center_y];
        }

        {
            unsigned int box3_sum =
                (unsigned int)box3_vertical[0] +
                2U * (unsigned int)box3_vertical[1];
            unsigned int box9_sum =
                (unsigned int)box9_vertical[0] +
                2U * ((unsigned int)box9_vertical[1] +
                      (unsigned int)box9_vertical[2] +
                      (unsigned int)box9_vertical[3] +
                      (unsigned int)box9_vertical[4]);
            int box_response = 9 * (int)box3_sum - (int)box9_sum;

            bottom_response[0] = (signed short)box_response;
#if defined(__ICCARM__)
#pragma unroll=3
#endif
            for(x = 1; x < BEACON_IMAGE_W; x++)
            {
#if defined(__ICCARM__)
                uint32 entering = __PKHBT(
                    (uint32)box3_vertical[x + 1],
                    (uint32)box9_vertical[x + 4], 16);
                uint32 leaving = __PKHBT(
                    (uint32)box3_vertical[x - 2],
                    (uint32)box9_vertical[x - 5], 16);
                uint32 delta = __SSUB16(entering, leaving);

                box_response = (int)__SMLAD(
                    delta, 0xFFFF0009UL, (uint32)box_response);
#else
                box_response +=
                    9 * ((int)box3_vertical[x + 1] -
                         (int)box3_vertical[x - 2]) -
                    ((int)box9_vertical[x + 4] -
                     (int)box9_vertical[x - 5]);
#endif
                bottom_response[x] = (signed short)box_response;
            }
        }

        if(top_response != 0)
        {
#if defined(__ICCARM__)
            const uint32 minimum_peak_gray_word =
                (uint32)GRAY_BEACON_MIN_PEAK_GRAY * 0x01010101UL;
#endif
            for(x = 3; x <= BEACON_IMAGE_W - 2; x++)
            {
                int candidate_x = x - 1;
                signed short response;

#if defined(__ICCARM__)
                if((x & 3) == 3)
                {
                    uint32 gray_word =
                        __UNALIGNED_UINT32_READ(&gray_row[candidate_x]);

                    (void)__USUB8(gray_word, minimum_peak_gray_word);
                    if(__SEL(0xFFFFFFFFUL, 0U) == 0U)
                    {
                        x += 3;
                        continue;
                    }
                }
#endif
                response = center_response[candidate_x];
                if((response > 0) &&
                   (gray_row[candidate_x] >= GRAY_BEACON_MIN_PEAK_GRAY) &&
                   (center_y >= (int)horizon_min_y[candidate_x]) &&
                   (response >= center_response[candidate_x + 1]) &&
                   (response >= center_response[candidate_x - 1]) &&
                   (response >= top_response[candidate_x]) &&
                   (response >= bottom_response[candidate_x]) &&
                   (response >= top_response[candidate_x + 1]) &&
                   (response >= bottom_response[candidate_x + 1]) &&
                   (response >= top_response[candidate_x - 1]) &&
                   (response >= bottom_response[candidate_x - 1]))
                {
                    if(((count < GRAY_BEACON_MAX_PEAKS) ||
                        (response >
                         peaks[GRAY_BEACON_MAX_PEAKS - 1].response)) &&
                       (gray_point_in_lamp_shadow(
                            candidate_x, center_y, lamp) == 0U) &&
                       (gray_point_in_lamp_shadow(
                            candidate_x, center_y, temporal_lamp) == 0U) &&
                       (gray_point_in_lamp_shadow(
                            candidate_x, center_y,
                            &g_secondary_lamp_mask) == 0U) &&
                       (gray_point_in_lamp_shadow(
                            candidate_x, center_y,
                            &g_secondary_temporal_lamp_mask) == 0U) &&
                       (gray_point_in_lamp(
                            candidate_x, center_y, &lamp_geometry) == 0U) &&
                       (gray_point_in_lamp(
                            candidate_x, center_y,
                            &temporal_lamp_geometry) == 0U) &&
                       (gray_point_in_lamp(
                            candidate_x, center_y,
                            &secondary_lamp_geometry) == 0U) &&
                       (gray_point_in_lamp(
                            candidate_x, center_y,
                            &secondary_temporal_lamp_geometry) == 0U) &&
                       (car_body_mask_contains(
                            candidate_x, center_y) == 0U))
                    {
                        gray_insert_peak(
                            peaks, &count, response, candidate_x, center_y);
                    }
                }
            }
        }

        if(y == BEACON_IMAGE_H - 2)
        {
            break;
        }
        {
            const unsigned char *box3_sub_row =
                image[gray_reflect_index(y - 1, BEACON_IMAGE_H)];
            const unsigned char *box3_add_row =
                image[gray_reflect_index(y + 2, BEACON_IMAGE_H)];
            const unsigned char *box9_sub_row =
                image[gray_reflect_index(y - 4, BEACON_IMAGE_H)];
            const unsigned char *box9_add_row =
                image[gray_reflect_index(y + 5, BEACON_IMAGE_H)];

            for(x = 0; x < BEACON_IMAGE_W; x += 4)
            {
                gray_vertical_box_sum_update4(
                    box3_vertical, box3_sub_row, box3_add_row, x);
                gray_vertical_box_sum_update4(
                    box9_vertical, box9_sub_row, box9_add_row, x);
            }
        }
        gray_vertical_box_sum_reflect_edges();
    }

    return count;
}
static void gray_lamp_geometry_init(
    const component_t *lamp,
    gray_lamp_geometry_t *geometry)
{
    float angle;

    memset(geometry, 0, sizeof(*geometry));
    if((lamp == 0) || (lamp->valid == 0U))
    {
        return;
    }

    angle = lamp->angle * (PI_F / 180.0f);
    geometry->cx = lamp->cx;
    geometry->cy = lamp->cy;
    geometry->cos_angle = cosf(angle);
    geometry->sin_angle = sinf(angle);
    geometry->half_length = lamp->major * 0.5f + 2.0f;
    geometry->half_width = lamp->minor * 0.5f + 2.0f;
    geometry->valid = 1U;
}

static unsigned char gray_point_in_lamp(
    int x,
    int y,
    const gray_lamp_geometry_t *geometry)
{
    float dx;
    float dy;
    float major;
    float minor;

    if((geometry == 0) || (geometry->valid == 0U))
    {
        return 0U;
    }
    dx = (float)x - geometry->cx;
    dy = (float)y - geometry->cy;
    major = dx * geometry->cos_angle + dy * geometry->sin_angle;
    minor = -dx * geometry->sin_angle + dy * geometry->cos_angle;
    return ((fabsf(major) <= geometry->half_length) &&
            (fabsf(minor) <= geometry->half_width)) ? 1U : 0U;
}

static unsigned char gray_point_in_lamp_shadow(
    int x,
    int y,
    const component_t *lamp)
{
    const float x_pad = 5.0f;
    const float down_scale = 2.0f;

    if((lamp == 0) || (lamp->valid == 0U) ||
       ((float)y <= (float)lamp->max_y))
    {
        return 0U;
    }
    return (((float)x >= (float)lamp->min_x - x_pad) &&
            ((float)x <= (float)lamp->max_x + x_pad) &&
            ((float)y <= (float)lamp->max_y +
                         lamp->major * down_scale)) ? 1U : 0U;
}

#if defined(__ICCARM__)
#pragma inline=never
#endif
static unsigned char gray_local_features(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int center_x,
    int center_y,
    gray_beacon_features_t *features)
{
    unsigned char histogram[256];
    float background;
    float inner_sum = 0.0f;
    float middle_sum = 0.0f;
    float weight_sum = 0.0f;
    float inner_weight = 0.0f;
    float weight_x = 0.0f;
    float weight_y = 0.0f;
    float weight_xx = 0.0f;
    float weight_yy = 0.0f;
    float weight_xy = 0.0f;
    float centroid_x;
    float centroid_y;
    float covariance_xx;
    float covariance_yy;
    float covariance_xy;
    float trace;
    float discriminant;
    float eigen_major;
    float eigen_minor;
    int dx;
    int dy;
    int ring_count = 0;
    int low_rank;
    int high_rank;
    int cumulative = 0;
    int low = 0;
    int high = 0;
    int value;
    int half_area = 0;
    unsigned char peak = 0U;

    if(features == 0)
    {
        return 0U;
    }
    memset(features, 0, sizeof(*features));
    memset(histogram, 0, sizeof(histogram));

    for(dy = -GRAY_BEACON_PATCH_RADIUS;
        dy <= GRAY_BEACON_PATCH_RADIUS; dy++)
    {
        for(dx = -GRAY_BEACON_PATCH_RADIUS;
            dx <= GRAY_BEACON_PATCH_RADIUS; dx++)
        {
            int radius2 = dx * dx + dy * dy;
            unsigned char pixel;

            if(radius2 > 49)
            {
                continue;
            }
            pixel = gray_pixel(image, center_x + dx, center_y + dy);

            if(radius2 <= 4)
            {
                inner_sum += pixel;
            }
            else if(radius2 <= 16)
            {
                middle_sum += pixel;
            }
            if((radius2 <= 36) && (pixel > peak))
            {
                peak = pixel;
            }
            if(radius2 >= 25)
            {
                histogram[pixel]++;
                ring_count++;
            }
        }
    }
    low_rank = (ring_count - 1) / 2;
    high_rank = ring_count / 2;
    for(value = 0; value < 256; value++)
    {
        cumulative += histogram[value];
        if(cumulative > low_rank)
        {
            low = value;
            break;
        }
    }
    cumulative = 0;
    for(value = 0; value < 256; value++)
    {
        cumulative += histogram[value];
        if(cumulative > high_rank)
        {
            high = value;
            break;
        }
    }
    background = ((float)low + (float)high) * 0.5f;

    for(dy = -6; dy <= 6; dy++)
    {
        for(dx = -6; dx <= 6; dx++)
        {
            int radius2 = dx * dx + dy * dy;
            unsigned char pixel;
            float weight;

            if(radius2 > 36)
            {
                continue;
            }
            pixel = gray_pixel(image, center_x + dx, center_y + dy);
            weight = (float)pixel - background;
            if(weight < 0.0f)
            {
                weight = 0.0f;
            }
            weight_sum += weight;
            weight_x += weight * (float)dx;
            weight_y += weight * (float)dy;
            weight_xx += weight * (float)(dx * dx);
            weight_yy += weight * (float)(dy * dy);
            weight_xy += weight * (float)(dx * dy);
            if(radius2 <= 4)
            {
                inner_weight += weight;
            }
            if((float)pixel > (float)peak * 0.5f)
            {
                half_area++;
            }
        }
    }

    if(weight_sum <= 0.0001f)
    {
        return 0U;
    }
    centroid_x = weight_x / weight_sum;
    centroid_y = weight_y / weight_sum;
    covariance_xx = weight_xx / weight_sum - centroid_x * centroid_x;
    covariance_yy = weight_yy / weight_sum - centroid_y * centroid_y;
    covariance_xy = weight_xy / weight_sum - centroid_x * centroid_y;
    trace = covariance_xx + covariance_yy;
    discriminant = (covariance_xx - covariance_yy) *
                   (covariance_xx - covariance_yy) +
                   4.0f * covariance_xy * covariance_xy;
    if(discriminant < 0.0f)
    {
        discriminant = 0.0f;
    }
    discriminant = sqrtf(discriminant);
    eigen_major = (trace + discriminant) * 0.5f;
    eigen_minor = (trace - discriminant) * 0.5f;
    if(eigen_major < 0.0001f) eigen_major = 0.0001f;
    if(eigen_minor < 0.0001f) eigen_minor = 0.0001f;

    features->inner_contrast = inner_sum / 13.0f - background;
    features->background = background;
    features->radial_drop = inner_sum / 13.0f - middle_sum / 36.0f;
    features->concentration = inner_weight / weight_sum;
    features->elongation = sqrtf(eigen_major / eigen_minor);
    features->offset = sqrtf(centroid_x * centroid_x + centroid_y * centroid_y);
    features->centroid_x = centroid_x;
    features->centroid_y = centroid_y;
    features->peak = peak;
    features->half_area = (unsigned char)half_area;
    return 1U;
}

static unsigned char gray_large_saturated_shape_valid(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int center_x,
    int center_y)
{
    enum
    {
        PATCH_SIZE = GRAY_BEACON_LARGE_SHAPE_RADIUS * 2 + 1,
        PATCH_AREA = PATCH_SIZE * PATCH_SIZE
    };
    static const signed char neighbor_x[8] =
        { 1, -1, 0, 0, 1, 1, -1, -1 };
    static const signed char neighbor_y[8] =
        { 0, 0, 1, -1, 1, -1, 1, -1 };
    unsigned char visited[PATCH_SIZE][PATCH_SIZE];
    signed char queue_x[PATCH_AREA];
    signed char queue_y[PATCH_AREA];
    int head = 0;
    int tail = 0;
    int seed_x = 0;
    int seed_y = 0;
    int count = 0;
    int sum_x = 0;
    int sum_y = 0;
    int sum_xx = 0;
    int sum_yy = 0;
    int sum_xy = 0;
    int x;
    int y;
    unsigned char peak = 0U;
    float cx;
    float cy;
    float var_x;
    float var_y;
    float cov_xy;
    float trace;
    float discriminant;
    float eigen_major;
    float eigen_minor;

    memset(visited, 0, sizeof(visited));
    for(y = -2; y <= 2; y++)
    {
        for(x = -2; x <= 2; x++)
        {
            unsigned char pixel = gray_pixel(
                image, center_x + x, center_y + y);
            if(pixel > peak)
            {
                peak = pixel;
                seed_x = x;
                seed_y = y;
            }
        }
    }
    if(peak < 120U)
    {
        return 1U;
    }

    queue_x[tail] = (signed char)seed_x;
    queue_y[tail] = (signed char)seed_y;
    visited[seed_y + GRAY_BEACON_LARGE_SHAPE_RADIUS]
           [seed_x + GRAY_BEACON_LARGE_SHAPE_RADIUS] = 1U;
    tail++;
    while(head < tail)
    {
        int index;
        int current_x = queue_x[head];
        int current_y = queue_y[head];
        head++;
        count++;
        sum_x += current_x;
        sum_y += current_y;
        sum_xx += current_x * current_x;
        sum_yy += current_y * current_y;
        sum_xy += current_x * current_y;

        if((current_x == -GRAY_BEACON_LARGE_SHAPE_RADIUS) ||
           (current_x == GRAY_BEACON_LARGE_SHAPE_RADIUS) ||
           (current_y == -GRAY_BEACON_LARGE_SHAPE_RADIUS) ||
           (current_y == GRAY_BEACON_LARGE_SHAPE_RADIUS))
        {
            return 0U;
        }
        for(index = 0; index < 8; index++)
        {
            int next_x = current_x + neighbor_x[index];
            int next_y = current_y + neighbor_y[index];
            int visit_x = next_x + GRAY_BEACON_LARGE_SHAPE_RADIUS;
            int visit_y = next_y + GRAY_BEACON_LARGE_SHAPE_RADIUS;
            if((visit_x < 0) || (visit_x >= PATCH_SIZE) ||
               (visit_y < 0) || (visit_y >= PATCH_SIZE) ||
               (visited[visit_y][visit_x] != 0U) ||
               (gray_pixel(image, center_x + next_x,
                           center_y + next_y) < 120U))
            {
                continue;
            }
            visited[visit_y][visit_x] = 1U;
            queue_x[tail] = (signed char)next_x;
            queue_y[tail] = (signed char)next_y;
            tail++;
        }
    }
    if(count < GRAY_BEACON_LARGE_SHAPE_MIN_AREA)
    {
        return 1U;
    }

    cx = (float)sum_x / (float)count;
    cy = (float)sum_y / (float)count;
    var_x = (float)sum_xx / (float)count - cx * cx;
    var_y = (float)sum_yy / (float)count - cy * cy;
    cov_xy = (float)sum_xy / (float)count - cx * cy;
    trace = var_x + var_y;
    discriminant = (var_x - var_y) * (var_x - var_y) +
                   4.0f * cov_xy * cov_xy;
    discriminant = sqrtf(discriminant);
    eigen_major = (trace + discriminant) * 0.5f;
    eigen_minor = (trace - discriminant) * 0.5f;
    if(eigen_minor < 0.0001f)
    {
        eigen_minor = 0.0001f;
    }
    return (sqrtf(eigen_major / eigen_minor) <=
            GRAY_BEACON_LARGE_SHAPE_MAX_ELONGATION) ? 1U : 0U;
}

static unsigned char gray_compact_point_valid(
    const gray_beacon_features_t *features,
    float scene_mean)
{
    if(features == 0)
    {
        return 0U;
    }
    if(scene_mean < GRAY_BEACON_DARK_SCENE_MEAN)
    {
        return ((features->half_area >= 4U) &&
                (features->half_area <= 24U) &&
                (features->peak >= 230U) &&
                (features->inner_contrast >= 160.0f) &&
                (features->radial_drop >= 110.0f) &&
                (features->concentration >= 0.40f) &&
                (features->elongation <= 1.90f) &&
                (features->offset <= 1.25f)) ? 1U : 0U;
    }
    if((scene_mean >= GRAY_BEACON_LOW_LIGHT_MEAN) &&
       (features->peak < GRAY_BEACON_COMPACT_MIN_PEAK) &&
       (features->background < 40.0f))
    {
        return 0U;
    }
    return ((features->half_area >= 4U) &&
            (features->half_area <= 24U) &&
            (features->peak >= 100U) &&
            (features->inner_contrast >= 45.0f) &&
            (features->radial_drop >= 40.0f) &&
            (features->concentration >= 0.45f) &&
            (features->elongation <= 1.90f) &&
            (features->offset <= 1.25f)) ? 1U : 0U;
}

static unsigned char gray_large_point_valid(
    const gray_beacon_features_t *features)
{
    return ((features != 0) &&
            (features->half_area >= GRAY_BEACON_LARGE_SHAPE_MIN_AREA) &&
            (features->peak >= 250U) &&
            (features->inner_contrast >= 190.0f) &&
            (features->radial_drop >= 35.0f) &&
            (features->elongation <= 1.50f) &&
            (features->offset <= 0.75f)) ? 1U : 0U;
}

static unsigned char gray_horizon_small_point_valid(
    const gray_beacon_features_t *features,
    int x,
    int y)
{
    float depth;

    if((features == 0) || (g_image_horizon_valid == 0U) ||
       (x < BEACON_EDGE_LEFT_X) || (x >= BEACON_EDGE_RIGHT_X) ||
       (g_image_horizon_column_valid[x] == 0U))
    {
        return 0U;
    }
    depth = (float)y - g_image_horizon_y[x];
    return ((depth >= -BEACON_HORIZON_TOLERANCE_PX) &&
            (depth <= GRAY_BEACON_HORIZON_SMALL_MAX_DEPTH) &&
            (features->background >= GRAY_BEACON_HORIZON_SMALL_BG_MIN) &&
            (features->background <= GRAY_BEACON_HORIZON_SMALL_BG_MAX) &&
            (features->half_area >= 3U) &&
            (features->half_area <= 8U) &&
            (features->peak >= 165U) &&
            (features->inner_contrast >= 38.0f) &&
            (features->radial_drop >= 38.0f) &&
            (features->concentration >= 0.60f) &&
            (features->elongation <= 1.60f) &&
            (features->offset <= 1.10f)) ? 1U : 0U;
}

static unsigned char gray_is_point_source(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int x,
    int y,
    float scene_mean,
    float *area,
    float *refined_x,
    float *refined_y,
    gray_beacon_features_t *features_out)
{
    gray_beacon_features_t features;
    unsigned char accepted;

    if(gray_local_features(image, x, y, &features) == 0U)
    {
        return 0U;
    }
    if((features.half_area >= GRAY_BEACON_LARGE_SHAPE_MIN_AREA) &&
       (gray_large_saturated_shape_valid(image, x, y) == 0U))
    {
        return 0U;
    }
    accepted = ((gray_compact_point_valid(&features, scene_mean) != 0U) ||
                (gray_horizon_small_point_valid(&features, x, y) != 0U) ||
                ((features.concentration >= 0.70f) &&
                 (features.radial_drop >= 31.0f)) ||
                ((features.concentration >= 0.66f) &&
                 (features.radial_drop >= 68.0f)) ||
                ((features.concentration >= 0.62f) &&
                 (features.radial_drop >= 100.0f) &&
                 (features.elongation <= 1.55f) &&
                 (features.offset <= 1.0f)) ||
                ((features.inner_contrast >= 165.0f) &&
                 (features.radial_drop >= 110.0f) &&
                 (features.elongation <= 1.55f) &&
                 (features.offset <= 1.0f)) ||
                ((scene_mean < GRAY_BEACON_DARK_SCENE_MEAN) &&
                 (features.half_area >= 4.5f) &&
                 (features.half_area <= GRAY_BEACON_COMPACT_MAX_AREA) &&
                 (features.peak >= 230U) &&
                 (features.radial_drop >= 120.0f) &&
                 (features.elongation <= 1.50f) &&
                 (features.offset <= 2.0f)) ||
                ((scene_mean >= GRAY_BEACON_DARK_SCENE_MEAN) &&
                 (scene_mean < GRAY_BEACON_LOW_LIGHT_MEAN) &&
                 (features.half_area >= GRAY_BEACON_COMPACT_MIN_AREA) &&
                 (features.half_area <= GRAY_BEACON_COMPACT_MAX_AREA) &&
                 (features.peak >= GRAY_BEACON_COMPACT_MIN_PEAK) &&
                 (features.radial_drop >= 70.0f) &&
                 (features.elongation <= 1.80f) &&
                 (features.offset <= 2.0f)) ||
                ((features.inner_contrast >= 170.0f) &&
                 (features.elongation <= 1.48f) &&
                 (features.offset <= 2.15f))) ? 1U : 0U;
    if((accepted != 0U) && (area != 0))
    {
        *area = (features.half_area > 0U) ?
                    (float)features.half_area : 1.0f;
    }
    if((accepted != 0U) &&
       (features.half_area >= GRAY_BEACON_LARGE_SHAPE_MIN_AREA))
    {
        if(refined_x != 0) *refined_x = (float)x + features.centroid_x;
        if(refined_y != 0) *refined_y = (float)y + features.centroid_y;
    }
    if((accepted != 0U) && (features_out != 0))
    {
        *features_out = features;
    }
    return accepted;
}

static unsigned char gray_is_saturated_component(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp,
    float scene_mean)
{
    float inner_sum = 0.0f;
    float outer_sum = 0.0f;
    float left_sum = 0.0f;
    float right_sum = 0.0f;
    float top_sum = 0.0f;
    float bottom_sum = 0.0f;
    unsigned char peak = 0U;
    int center_x;
    int center_y;
    int dx;
    int dy;
    int left_count = 0;
    int right_count = 0;
    int top_count = 0;
    int bottom_count = 0;
    int width;
    int height;

    if((comp == 0) || (comp->valid == 0U) ||
       (scene_mean < GRAY_BEACON_BRIGHT_SCENE_MEAN) ||
       (comp->area < 30) || (comp->area > 120) ||
       (comp->elongation > 1.5f))
    {
        return 0U;
    }
    width = comp->max_x - comp->min_x + 1;
    height = comp->max_y - comp->min_y + 1;
    if(comp->area * 10 < width * height * 6)
    {
        return 0U;
    }
    center_x = (int)(comp->cx + 0.5f);
    center_y = (int)(comp->cy + 0.5f);

    for(dy = -6; dy <= 6; dy++)
    {
        for(dx = -6; dx <= 6; dx++)
        {
            int radius2 = dx * dx + dy * dy;
            unsigned char pixel = gray_pixel(image, center_x + dx, center_y + dy);

            if(radius2 <= 4)
            {
                inner_sum += pixel;
                if(dx < 0) { left_sum += pixel; left_count++; }
                if(dx > 0) { right_sum += pixel; right_count++; }
                if(dy < 0) { top_sum += pixel; top_count++; }
                if(dy > 0) { bottom_sum += pixel; bottom_count++; }
            }
            else if((radius2 > 16) && (radius2 <= 36))
            {
                outer_sum += pixel;
            }
            if((dx >= -2) && (dx <= 2) &&
               (dy >= -2) && (dy <= 2) && (pixel > peak))
            {
                peak = pixel;
            }
        }
    }

    return ((peak >= 250U) &&
            (inner_sum / 13.0f - outer_sum / 64.0f >= 120.0f) &&
            (fabsf(left_sum / (float)left_count -
                   right_sum / (float)right_count) <= 20.0f) &&
            (fabsf(top_sum / (float)top_count -
                   bottom_sum / (float)bottom_count) <= 20.0f)) ? 1U : 0U;
}

static void gray_add_candidate(
    gray_beacon_candidate_t candidates[GRAY_BEACON_MAX_CANDIDATES],
    unsigned char *count,
    float x,
    float y,
    float area,
    const gray_beacon_features_t *features)
{
    int index;

    if((count == 0) || (x < 2.0f) || (x > BEACON_IMAGE_W - 3.0f) ||
       (y < 2.0f) || (y > BEACON_IMAGE_H - 3.0f) ||
       (car_body_mask_contains(
            (int)(x + 0.5f), (int)(y + 0.5f)) != 0U))
    {
        return;
    }
    for(index = 0; index < *count; index++)
    {
        float dx = (float)x - candidates[index].x;
        float dy = (float)y - candidates[index].y;
        if(dx * dx + dy * dy <= 25.0f)
        {
            return;
        }
    }
    if(*count >= GRAY_BEACON_MAX_CANDIDATES)
    {
        return;
    }
    candidates[*count].x = (float)x;
    candidates[*count].y = (float)y;
    candidates[*count].area = (area > 0.0f) ? area : 1.0f;
    candidates[*count].features_valid = 0U;
    if(features != 0)
    {
        candidates[*count].features = *features;
        candidates[*count].features_valid = 1U;
    }
    (*count)++;
}

static unsigned char gray_top_edge_point_valid(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp,
    float scene_mean)
{
    int width;
    int height;
    int x;
    int y;
    unsigned char peak = 0U;

    if((image == 0) || (comp == 0) || (comp->valid == 0U) ||
       (scene_mean >= GRAY_BEACON_DARK_SCENE_MEAN) ||
       (comp->min_y > 2) || (comp->max_y > 6) ||
       (comp->cx < (float)BEACON_EDGE_LEFT_X) ||
       (comp->cx >= (float)BEACON_EDGE_RIGHT_X) ||
       (comp->area < 4) || (comp->area > 20))
    {
        return 0U;
    }
    width = comp->max_x - comp->min_x + 1;
    height = comp->max_y - comp->min_y + 1;
    if((width > 8) || (height > 5) ||
       (comp->area * 2 < width * height))
    {
        return 0U;
    }
    for(y = comp->min_y; y <= comp->max_y; y++)
    {
        for(x = comp->min_x; x <= comp->max_x; x++)
        {
            if(image[y][x] > peak)
            {
                peak = image[y][x];
            }
        }
    }
    return (peak >= 230U) ? 1U : 0U;
}

static void gray_add_component_candidates(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *lamp,
    const component_t *temporal_lamp,
    float scene_mean,
    unsigned char reusable_binary_threshold,
    gray_beacon_candidate_t candidates[GRAY_BEACON_MAX_CANDIDATES],
    unsigned char *count)
{
    gray_lamp_geometry_t lamp_geometry;
    gray_lamp_geometry_t temporal_lamp_geometry;
    gray_lamp_geometry_t secondary_lamp_geometry;
    gray_lamp_geometry_t secondary_temporal_lamp_geometry;
    unsigned short evaluated = 0U;
    unsigned char threshold;
    int minimum_area;
    int maximum_area;
    int x;
    int y;

    if(scene_mean < GRAY_BEACON_DARK_SCENE_MEAN)
    {
        threshold = 80U;
        minimum_area = 0;
        maximum_area = 500;
    }
    else if(scene_mean >= GRAY_BEACON_BRIGHT_SCENE_MEAN)
    {
        threshold = 120U;
        minimum_area = 30;
        maximum_area = 120;
    }
    else
    {
        return;
    }

    if(reusable_binary_threshold != threshold)
    {
        threshold_image(image, threshold);
    }
    erase_lamp_from_binary(lamp);
    erase_temporal_lamp_from_binary(temporal_lamp);
    erase_lamp_from_binary(&g_secondary_lamp_mask);
    erase_temporal_lamp_from_binary(&g_secondary_temporal_lamp_mask);
    erase_car_body_from_binary();
    gray_lamp_geometry_init(lamp, &lamp_geometry);
    gray_lamp_geometry_init(temporal_lamp, &temporal_lamp_geometry);
    gray_lamp_geometry_init(&g_secondary_lamp_mask,
                            &secondary_lamp_geometry);
    gray_lamp_geometry_init(&g_secondary_temporal_lamp_mask,
                            &secondary_temporal_lamp_geometry);
    for(y = 0; y < BEACON_IMAGE_H; y++)
    {
        for(x = 0; x < BEACON_IMAGE_W; x++)
        {
            component_t comp;
            int center_x;
            int center_y;
            float area;
            unsigned char accepted;
            gray_beacon_features_t features;
            unsigned char features_valid = 0U;

#if defined(__ICCARM__)
            if(((x & 3) == 0) &&
               (__UNALIGNED_UINT32_READ(&g_binary[y][x]) == 0U))
            {
                x += 3;
                continue;
            }
#endif
            if(g_binary[y][x] == 0U)
            {
                continue;
            }
            if(evaluated >= BEACON_MAX_EVALUATED_COMPONENTS)
            {
                (void)grow_component_consume(
                    (unsigned char)x, (unsigned char)y, 0, -1, 0);
                continue;
            }
            comp = grow_component_consume(
                (unsigned char)x, (unsigned char)y,
                minimum_area, maximum_area, 0);
            evaluated++;
            if((comp.area < minimum_area) || (comp.area > maximum_area))
            {
                continue;
            }
            center_x = (int)(comp.cx + 0.5f);
            center_y = (int)(comp.cy + 0.5f);
            if((gray_point_below_horizon(center_x, center_y) == 0U) ||
               (gray_point_in_lamp(
                    center_x, center_y, &lamp_geometry) != 0U) ||
               (gray_point_in_lamp(
                    center_x, center_y, &temporal_lamp_geometry) != 0U) ||
               (gray_point_in_lamp(
                    center_x, center_y, &secondary_lamp_geometry) != 0U) ||
               (gray_point_in_lamp(
                    center_x, center_y,
                    &secondary_temporal_lamp_geometry) != 0U) ||
               (gray_point_in_lamp_shadow(
                    center_x, center_y, lamp) != 0U) ||
               (gray_point_in_lamp_shadow(
                    center_x, center_y, temporal_lamp) != 0U) ||
               (gray_point_in_lamp_shadow(
                    center_x, center_y, &g_secondary_lamp_mask) != 0U) ||
               (gray_point_in_lamp_shadow(
                    center_x, center_y,
                    &g_secondary_temporal_lamp_mask) != 0U))
            {
                continue;
            }
            area = (float)comp.area;
            accepted = gray_top_edge_point_valid(
                image, &comp, scene_mean);
            if(accepted == 0U)
            {
                accepted = (scene_mean < GRAY_BEACON_DARK_SCENE_MEAN) ?
                    gray_is_point_source(
                        image, center_x, center_y, scene_mean,
                        &area, 0, 0, &features) :
                    gray_is_saturated_component(image, &comp, scene_mean);
                if((accepted != 0U) &&
                   (scene_mean < GRAY_BEACON_DARK_SCENE_MEAN))
                {
                    features_valid = 1U;
                }
            }
            if(accepted != 0U)
            {
                gray_add_candidate(candidates, count, center_x, center_y,
                                   (float)comp.area,
                                   (features_valid != 0U) ? &features : 0);
            }
        }
    }
}

static void gray_insert_confirmed_beacon(
    const gray_beacon_candidate_t *candidate,
    beacon_result_t *result)
{
    beacon_circle_t circle;
    int slot;
    int index;

    if((candidate == 0) || (result == 0))
    {
        return;
    }
    slot = result->beacon_count;
    if(slot >= BEACON_MAX_BEACON_COUNT)
    {
        slot = BEACON_MAX_BEACON_COUNT - 1;
        if(candidate->area <= result->beacons[slot].area)
        {
            return;
        }
    }
    else
    {
        result->beacon_count++;
    }
    for(index = slot; index > 0; index--)
    {
        if(candidate->area <= result->beacons[index - 1].area)
        {
            break;
        }
        result->beacons[index] = result->beacons[index - 1];
    }
    circle.x = candidate->x - (float)BEACON_IMAGE_W * 0.5f;
    circle.y = candidate->y - (float)BEACON_IMAGE_H * 0.5f;
    circle.area = candidate->area;
    circle.radius = sqrtf(candidate->area / PI_F);
    circle.valid = 1U;
    result->beacons[index] = circle;
}

static unsigned char gray_candidate_is_reliable(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const gray_beacon_candidate_t *candidate,
    float scene_mean)
{
    gray_beacon_features_t features;
    unsigned char peak = 0U;
    int center_x;
    int center_y;
    int x;
    int y;

    if((candidate == 0) || (scene_mean < GRAY_BEACON_DARK_SCENE_MEAN))
    {
        return (candidate != 0) ? 1U : 0U;
    }
    center_x = (int)(candidate->x + 0.5f);
    center_y = (int)(candidate->y + 0.5f);
    if(candidate->features_valid != 0U)
    {
        features = candidate->features;
    }
    else if(gray_local_features(image, center_x, center_y, &features) == 0U)
    {
        return 0U;
    }
    if((g_image_horizon_valid != 0U) &&
       (center_x >= 0) && (center_x < BEACON_IMAGE_W) &&
       (g_image_horizon_column_valid[center_x] != 0U) &&
       ((float)center_y <=
        g_image_horizon_y[center_x] + BEACON_HORIZON_SMALL_BAND_PX))
    {
        return gray_horizon_small_point_valid(
            &features, center_x, center_y);
    }
    if(gray_compact_point_valid(&features, scene_mean) != 0U)
    {
        return 1U;
    }
    if(gray_large_point_valid(&features) != 0U)
    {
        return 1U;
    }
    if(gray_horizon_small_point_valid(
           &features, center_x, center_y) != 0U)
    {
        return 1U;
    }
    if(candidate->area > GRAY_BEACON_COMPACT_MAX_AREA)
    {
        if(candidate->y >= GRAY_BEACON_LARGE_MIN_Y)
        {
            return 1U;
        }
        return (((candidate->area <= GRAY_BEACON_UPPER_LARGE_MAX_AREA) &&
                 (features.peak >= 250U) &&
                 (features.inner_contrast >= 180.0f) &&
                 (features.radial_drop >= 95.0f) &&
                 (features.elongation <= 1.60f) &&
                 (features.offset <= 1.10f)) ||
                ((candidate->area <= 60.0f) &&
                 (features.peak >= 250U) &&
                 (features.inner_contrast >= 190.0f) &&
                 (features.radial_drop >= 65.0f) &&
                 (features.elongation <= 1.60f) &&
                 (features.offset <= 1.10f))) ? 1U : 0U;
    }
    if(candidate->area < GRAY_BEACON_COMPACT_MIN_AREA)
    {
        return 0U;
    }

    for(y = center_y - 3; y <= center_y + 3; y++)
    {
        for(x = center_x - 3; x <= center_x + 3; x++)
        {
            unsigned char pixel = gray_pixel(image, x, y);
            if(pixel > peak)
            {
                peak = pixel;
            }
        }
    }
    return (peak >= GRAY_BEACON_COMPACT_MIN_PEAK) ? 1U : 0U;
}

static void gray_store_candidates(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const gray_beacon_candidate_t candidates[GRAY_BEACON_MAX_CANDIDATES],
    unsigned char count,
    float scene_mean,
    beacon_result_t *result)
{
    unsigned char current;

    result->beacon_count = 0U;
    for(current = 0; current < count; current++)
    {
        if(gray_candidate_is_reliable(
               image, &candidates[current], scene_mean) != 0U)
        {
            gray_insert_confirmed_beacon(&candidates[current], result);
        }
    }
}

static unsigned char is_component_in_lamp_core(
    const component_t *comp,
    const component_t *lamp)
{
    float angle;
    float cos_a;
    float sin_a;
    float dx;
    float dy;
    float major;
    float minor;

    if((comp == 0) || (comp->valid == 0) ||
       (lamp == 0) || (lamp->valid == 0))
    {
        return 0;
    }

    angle = lamp->angle * (PI_F / 180.0f);
    cos_a = cosf(angle);
    sin_a = sinf(angle);
    dx = comp->cx - lamp->cx;
    dy = comp->cy - lamp->cy;
    major = dx * cos_a + dy * sin_a;
    minor = -dx * sin_a + dy * cos_a;

    return ((fabsf(major) <= lamp->major * 0.5f + (float)CAR_LAMP_TEMPORAL_CORE_PAD) &&
            (fabsf(minor) <= lamp->minor * 0.5f + (float)CAR_LAMP_TEMPORAL_CORE_PAD)) ? 1 : 0;
}

static unsigned char is_near_lamp(const component_t *comp, const component_t *lamp)
{
    if((comp == 0) || (lamp == 0) || (lamp->valid == 0))
    {
        return 0;
    }

    return ((comp->max_x >= lamp->min_x - LAMP_NEAR_BEACON_PAD) &&
            (comp->min_x <= lamp->max_x + LAMP_NEAR_BEACON_PAD) &&
            (comp->max_y >= lamp->min_y - LAMP_NEAR_BEACON_PAD) &&
            (comp->min_y <= lamp->max_y + LAMP_NEAR_BEACON_PAD)) ? 1 : 0;
}

static unsigned char is_edge_beacon_area(const component_t *comp)
{
    if((comp == 0) || (comp->valid == 0))
    {
        return 0;
    }

    return ((comp->min_y < BEACON_EDGE_TOP_Y) ||
            (comp->max_y >= BEACON_EDGE_BOTTOM_Y) ||
            (comp->min_x < BEACON_EDGE_LEFT_X) ||
            (comp->max_x >= BEACON_EDGE_RIGHT_X)) ? 1 : 0;
}

static unsigned char is_side_edge_beacon_area(const component_t *comp)
{
    if((comp == 0) || (comp->valid == 0))
    {
        return 0;
    }

    return ((comp->min_x < BEACON_EDGE_LEFT_X) ||
            (comp->max_x >= BEACON_EDGE_RIGHT_X)) ? 1 : 0;
}

static unsigned char is_incomplete_border_component(const component_t *comp)
{
    return ((comp != 0) && (comp->valid != 0U) &&
            ((comp->min_x <= 0) ||
             (comp->min_y <= 0) ||
             (comp->max_x >= BEACON_IMAGE_W - 1) ||
             (comp->max_y >= BEACON_IMAGE_H - 1))) ? 1U : 0U;
}

static unsigned char matches_confirmed_beacon_track(const component_t *comp)
{
    float x;
    float y;
    float dx;
    float dy;

    if((comp == 0) || (comp->valid == 0U) ||
       (g_b0_track.confirmed == 0U))
    {
        return 0U;
    }

    x = comp->cx - (float)BEACON_IMAGE_W * 0.5f;
    y = comp->cy - (float)BEACON_IMAGE_H * 0.5f;
    dx = x - (g_b0_track.x + g_b0_track.vx);
    dy = y - (g_b0_track.y + g_b0_track.vy);
    return ((dx * dx + dy * dy) <=
            KALMAN_GATE_DISTANCE * KALMAN_GATE_DISTANCE) ? 1U : 0U;
}

static unsigned char is_untracked_top_corner_component(
    const component_t *comp)
{
    float side_distance;

    if((comp == 0) || (comp->valid == 0U))
    {
        return 0U;
    }
    side_distance = comp->cx;
    if(((float)(BEACON_IMAGE_W - 1) - comp->cx) < side_distance)
    {
        side_distance = (float)(BEACON_IMAGE_W - 1) - comp->cx;
    }
    if((side_distance * BEACON_TOP_CORNER_HEIGHT +
        comp->cy * BEACON_TOP_CORNER_WIDTH) >
       (BEACON_TOP_CORNER_WIDTH * BEACON_TOP_CORNER_HEIGHT))
    {
        return 0U;
    }
    return (matches_confirmed_beacon_track(comp) == 0U) ? 1U : 0U;
}

static unsigned char local_background_average(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp)
{
    int x;
    int y;
    int count = 0;
    int sum = 0;
    int cx;
    int cy;
    int min_x;
    int max_x;
    int min_y;
    int max_y;

    if((image == 0) || (comp == 0) || (comp->valid == 0))
    {
        return 0;
    }
    cx = (int)(comp->cx + 0.5f);
    cy = (int)(comp->cy + 0.5f);
    min_x = cx - (int)BEACON_LOCAL_RING_OUTER;
    max_x = cx + (int)BEACON_LOCAL_RING_OUTER;
    min_y = cy - (int)BEACON_LOCAL_RING_OUTER;
    max_y = cy + (int)BEACON_LOCAL_RING_OUTER;
    if(min_x < 0) min_x = 0;
    if(min_y < 0) min_y = 0;
    if(max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if(max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;

    for(y = min_y; y <= max_y; y++)
    {
        for(x = min_x; x <= max_x; x++)
        {
            int dx = x - cx;
            int dy = y - cy;
            int dist = (dx < 0) ? -dx : dx;
            int abs_dy = (dy < 0) ? -dy : dy;

            if(abs_dy > dist) dist = abs_dy;
            if((dist <= BEACON_LOCAL_RING_INNER) ||
               (dist > BEACON_LOCAL_RING_OUTER))
            {
                continue;
            }

            sum += image[y][x];
            count++;
        }
    }

    if(count == 0)
    {
        return 0;
    }
    return (unsigned char)(sum / count);
}

static unsigned char component_max_gray(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp)
{
    int x;
    int y;
    unsigned char max_gray = 0U;

    if((image == 0) || (comp == 0) || (comp->valid == 0))
    {
        return 0;
    }
    for(y = comp->min_y; y <= comp->max_y; y++)
    {
        for(x = comp->min_x; x <= comp->max_x; x++)
        {
            if((g_binary[y][x] != 0U) && (image[y][x] > max_gray))
            {
                max_gray = image[y][x];
            }
        }
    }
    return max_gray;
}

static unsigned char has_large_weak_footprint(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp)
{
    int x;
    int y;
    int count = 0;
    int cx = (int)(comp->cx + 0.5f);
    int cy = (int)(comp->cy + 0.5f);
    int min_x = cx - BEACON_WEAK_FOOTPRINT_RADIUS;
    int max_x = cx + BEACON_WEAK_FOOTPRINT_RADIUS;
    int min_y = cy - BEACON_WEAK_FOOTPRINT_RADIUS;
    int max_y = cy + BEACON_WEAK_FOOTPRINT_RADIUS;

    if(min_x < 0) min_x = 0;
    if(min_y < 0) min_y = 0;
    if(max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if(max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;

    for(y = min_y; y <= max_y; y++)
    {
        for(x = min_x; x <= max_x; x++)
        {
            if((image[y][x] >= BEACON_WEAK_FOOTPRINT_GRAY) &&
               (++count > BEACON_WEAK_FOOTPRINT_MAX))
            {
                return 1U;
            }
        }
    }
    return 0U;
}

static unsigned char is_isolated_small_beacon(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp)
{
    if((comp == 0) || (comp->valid == 0) ||
       (comp->area < BEACON_ISOLATED_MIN_AREA) ||
       (comp->area > BEACON_ISOLATED_MAX_AREA))
    {
        return 0;
    }
    if(component_max_gray(image, comp) < BEACON_ISOLATED_GRAY_MIN)
    {
        return 0;
    }

    return (local_background_average(image, comp) <=
            BEACON_ISOLATED_BG_MAX) ? 1 : 0;
}

static unsigned char is_isolated_near_lamp_beacon(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp)
{
    if((comp == 0) || (comp->valid == 0) ||
       (comp->area < LAMP_NEAR_BEACON_ISOLATED_MIN_AREA))
    {
        return 0;
    }
    if(component_max_gray(image, comp) < LAMP_NEAR_BEACON_GRAY_MIN)
    {
        return 0;
    }

    return (local_background_average(image, comp) <=
            LAMP_NEAR_BEACON_BACKGROUND_MAX) ? 1 : 0;
}

static component_t g_bad_shape_components[BEACON_BAD_SHAPE_MAX_COUNT];
static unsigned char g_bad_shape_count;

static unsigned char component_average_gray(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp)
{
    int x;
    int y;
    int count = 0;
    unsigned int sum = 0U;

    if((image == 0) || (comp == 0) || (comp->valid == 0U))
    {
        return 0U;
    }
    for(y = comp->min_y; y <= comp->max_y; y++)
    {
        for(x = comp->min_x; x <= comp->max_x; x++)
        {
            if(g_binary[y][x] != 0U)
            {
                sum += image[y][x];
                count++;
            }
        }
    }
    return (count > 0) ? (unsigned char)(sum / (unsigned int)count) : 0U;
}

static unsigned char is_obvious_reflection(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp)
{
    unsigned char max_gray;

    if((image == 0) || (comp == 0) || (comp->valid == 0U))
    {
        return 0U;
    }

    max_gray = component_max_gray(image, comp);
    if((comp->area >= BEACON_LINEAR_MIN_AREA) &&
       (comp->major >= BEACON_LINEAR_MIN_MAJOR) &&
       (comp->minor <= BEACON_LINEAR_MAX_MINOR) &&
       (comp->elongation >= BEACON_LINEAR_MIN_ELONGATION))
    {
        return 1U;
    }
    if((comp->area <= BEACON_TINY_LINE_MAX_AREA) &&
       (comp->minor <= BEACON_TINY_LINE_MAX_MINOR) &&
       (comp->elongation >= BEACON_TINY_LINE_MIN_ELONGATION) &&
       (max_gray < BEACON_TINY_LINE_MAX_GRAY))
    {
        return 1U;
    }

    return 0U;
}

static unsigned char is_beacon_shape_candidate(const component_t *comp)
{
    int width;
    int height;
    int box_area;

    if((comp == 0) || (comp->valid == 0U) ||
       (comp->area < BEACON_SHAPE_MIN_AREA))
    {
        return 1U;
    }
    if((comp->min_x <= 0) || (comp->max_x >= BEACON_IMAGE_W - 1) ||
       (comp->min_y <= 0) || (comp->max_y >= BEACON_IMAGE_H - 1))
    {
        return 1U;
    }

    width = comp->max_x - comp->min_x + 1;
    height = comp->max_y - comp->min_y + 1;
    if((comp->area >= BEACON_SHAPE_RATIO_MIN_AREA) &&
       ((width * BEACON_SHAPE_MAX_RATIO_DEN >
         height * BEACON_SHAPE_MAX_RATIO_NUM) ||
        (height * BEACON_SHAPE_MAX_RATIO_DEN >
         width * BEACON_SHAPE_MAX_RATIO_NUM)))
    {
        return 0U;
    }

    box_area = width * height;
    if((comp->area >= BEACON_SHAPE_FILL_MIN_AREA) &&
       (comp->area * 100 <
        box_area * BEACON_SHAPE_MIN_FILL_PERCENT))
    {
        return 0U;
    }
    if((comp->area < BEACON_SHAPE_FILL_MIN_AREA) &&
       (comp->area * 100 <
        box_area * BEACON_SHAPE_SMALL_MIN_FILL_PERCENT))
    {
        return 0U;
    }

    return 1U;
}

static void record_bad_shape_component(const component_t *comp)
{
    if((comp == 0) || (comp->valid == 0U) ||
       (g_bad_shape_count >= BEACON_BAD_SHAPE_MAX_COUNT))
    {
        return;
    }

    g_bad_shape_components[g_bad_shape_count++] = *comp;
}

static unsigned char output_local_max_gray(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    float image_x,
    float image_y)
{
    int x;
    int y;
    int cx = (int)(image_x + 0.5f);
    int cy = (int)(image_y + 0.5f);
    int min_x = cx - BEACON_OUTPUT_LOCAL_RADIUS;
    int max_x = cx + BEACON_OUTPUT_LOCAL_RADIUS;
    int min_y = cy - BEACON_OUTPUT_LOCAL_RADIUS;
    int max_y = cy + BEACON_OUTPUT_LOCAL_RADIUS;
    unsigned char max_gray = 0U;

    if(min_x < 0) min_x = 0;
    if(min_y < 0) min_y = 0;
    if(max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if(max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;
    for(y = min_y; y <= max_y; y++)
    {
        for(x = min_x; x <= max_x; x++)
        {
            if(image[y][x] > max_gray)
            {
                max_gray = image[y][x];
            }
        }
    }
    return max_gray;
}

static void suppress_bad_shape_beacons(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result)
{
    int input;
    int kept = 0;
    int count;

    if(result == 0)
    {
        return;
    }
    count = result->beacon_count;
    if(count > BEACON_MAX_BEACON_COUNT)
    {
        count = BEACON_MAX_BEACON_COUNT;
    }

    for(input = 0; input < count; input++)
    {
        int bad_index;
        unsigned char suppress = 0U;
        float image_x = result->beacons[input].x +
                        (float)BEACON_IMAGE_W * 0.5f;
        float image_y = result->beacons[input].y +
                        (float)BEACON_IMAGE_H * 0.5f;
        float area = beacon_area(&result->beacons[input]);
        unsigned char local_max = output_local_max_gray(
            image, image_x, image_y);

        if((area <= BEACON_OUTPUT_DIM_MID_MAX_AREA) &&
           (image_y >= BEACON_OUTPUT_DIM_MID_MIN_Y) &&
           (image_y < BEACON_OUTPUT_DIM_MID_SPLIT_Y) &&
           (local_max <= BEACON_OUTPUT_DIM_UPPER_MAX_GRAY))
        {
            suppress = 1U;
        }
        if((area <= BEACON_OUTPUT_DIM_MID_MAX_AREA) &&
           (image_y >= BEACON_OUTPUT_DIM_MID_SPLIT_Y) &&
           (image_y < BEACON_OUTPUT_DIM_MID_MAX_Y) &&
           (local_max <= BEACON_OUTPUT_DIM_MID_MAX_GRAY))
        {
            suppress = 1U;
        }
        if((area <= BEACON_OUTPUT_SIDE_MAX_AREA) &&
           (image_y >= BEACON_OUTPUT_SIDE_MIN_Y) &&
           ((image_x < BEACON_OUTPUT_SIDE_MARGIN) ||
            (image_x >= (float)BEACON_IMAGE_W -
                        BEACON_OUTPUT_SIDE_MARGIN)) &&
           (local_max <= BEACON_OUTPUT_SIDE_MAX_GRAY))
        {
            suppress = 1U;
        }

        for(bad_index = 0;
            (suppress == 0U) && (bad_index < g_bad_shape_count);
            bad_index++)
        {
            const component_t *bad = &g_bad_shape_components[bad_index];

            if((image_x >= (float)(bad->min_x - BEACON_BAD_SHAPE_MATCH_PAD)) &&
               (image_x <= (float)(bad->max_x + BEACON_BAD_SHAPE_MATCH_PAD)) &&
               (image_y >= (float)(bad->min_y - BEACON_BAD_SHAPE_MATCH_PAD)) &&
               (image_y <= (float)(bad->max_y + BEACON_BAD_SHAPE_MATCH_PAD)))
            {
                suppress = 1U;
                break;
            }
        }

        if(suppress == 0U)
        {
            if(kept != input)
            {
                result->beacons[kept] = result->beacons[input];
            }
            kept++;
        }
    }

    for(input = kept; input < count; input++)
    {
        memset(&result->beacons[input], 0, sizeof(result->beacons[input]));
    }
    result->beacon_count = (unsigned char)kept;
}

static unsigned char is_vertical_top_beacon_near_lamp(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp,
    const component_t *lamp)
{
    int width;

    if((image == 0) || (comp == 0) || (comp->valid == 0U) ||
       (lamp == 0) || (lamp->valid == 0U))
    {
        return 0U;
    }
    width = comp->max_x - comp->min_x + 1;
    return ((comp->area >= BEACON_TOP_VERTICAL_MIN_AREA) &&
            (comp->area <= BEACON_TOP_VERTICAL_MAX_AREA) &&
            (width <= BEACON_TOP_VERTICAL_MAX_WIDTH) &&
            (comp->elongation >= BEACON_TOP_VERTICAL_MIN_ELONGATION) &&
            (component_max_gray(image, comp) >=
             BEACON_TOP_VERTICAL_MIN_GRAY) &&
            (component_max_gray(image, comp) <=
             BEACON_TOP_VERTICAL_MAX_GRAY) &&
            (lamp->area >= BEACON_TOP_VERTICAL_LAMP_MIN_AREA) &&
            (fabsf(comp->cx - lamp->cx) <=
             BEACON_TOP_VERTICAL_LAMP_MAX_DX) &&
            ((lamp->cy - comp->cy) >=
             BEACON_TOP_VERTICAL_LAMP_MIN_DY) &&
            ((lamp->cy - comp->cy) <=
             BEACON_TOP_VERTICAL_LAMP_MAX_DY)) ? 1U : 0U;
}

static unsigned char is_saturated_top_beacon_near_lamp(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp,
    const component_t *lamp)
{
    int width;
    int height;

    if((image == 0) || (comp == 0) || (comp->valid == 0U) ||
       (lamp == 0) || (lamp->valid == 0U))
    {
        return 0U;
    }
    width = comp->max_x - comp->min_x + 1;
    height = comp->max_y - comp->min_y + 1;
    return ((comp->cy >= BEACON_SATURATED_TOP_MIN_Y) &&
            (comp->cy < BEACON_SATURATED_TOP_MAX_Y) &&
            (comp->area >= BEACON_SATURATED_TOP_MIN_AREA) &&
            (comp->area <= BEACON_SATURATED_TOP_MAX_AREA) &&
            (component_max_gray(image, comp) >=
             BEACON_SATURATED_TOP_MIN_GRAY) &&
            (comp->area * 100 >= width * height *
             BEACON_SATURATED_TOP_MIN_FILL_PERCENT) &&
            (fabsf(comp->cx - lamp->cx) >=
             BEACON_SATURATED_TOP_LAMP_MIN_DX) &&
            (fabsf(comp->cx - lamp->cx) <=
             BEACON_SATURATED_TOP_LAMP_MAX_DX) &&
            ((lamp->cy - comp->cy) >=
             BEACON_SATURATED_TOP_LAMP_MIN_DY) &&
            ((lamp->cy - comp->cy) <=
             BEACON_SATURATED_TOP_LAMP_MAX_DY)) ? 1U : 0U;
}
static void insert_beacon_by_area(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp,
    const component_t *lamp,
    const component_t *temporal_lamp,
    beacon_result_t *result)
{
    int i;
    int slot;
    int min_area;
    unsigned char is_edge;
    unsigned char is_side_edge;
    beacon_circle_t circle;

    if((comp == 0) || (comp->valid == 0))
    {
        return;
    }
    if(car_body_mask_contains(
           (int)(comp->cx + 0.5f),
           (int)(comp->cy + 0.5f)) != 0U)
    {
        return;
    }
    if((is_beacon_shape_candidate(comp) == 0U) &&
       (is_saturated_top_beacon_near_lamp(image, comp, lamp) == 0U) &&
       (is_saturated_top_beacon_near_lamp(
            image, comp, &g_secondary_lamp_mask) == 0U))
    {
        record_bad_shape_component(comp);
    }
    if((is_obvious_reflection(image, comp) != 0U) &&
       (is_vertical_top_beacon_near_lamp(image, comp, lamp) == 0U) &&
       (is_saturated_top_beacon_near_lamp(image, comp, lamp) == 0U) &&
       (is_vertical_top_beacon_near_lamp(
            image, comp, &g_secondary_lamp_mask) == 0U) &&
       (is_saturated_top_beacon_near_lamp(
            image, comp, &g_secondary_lamp_mask) == 0U))
    {
        return;
    }
    if(comp->max_y >= COMPONENT_BOTTOM_REJECT_Y)
    {
        return;
    }
    if((comp->cy >= BEACON_LOWER_SMALL_Y) &&
       (comp->area < BEACON_MIN_COMPONENT_AREA))
    {
        return;
    }

    is_edge = is_edge_beacon_area(comp);
    is_side_edge = is_side_edge_beacon_area(comp);
    if((is_edge != 0U) &&
       (comp->area <= BEACON_ISOLATED_MAX_AREA) &&
       (component_max_gray(image, comp) < BEACON_ISOLATED_GRAY_MIN) &&
       (has_large_weak_footprint(image, comp) != 0U))
    {
        return;
    }

    min_area = ((is_edge != 0) ||
                (is_isolated_small_beacon(image, comp) != 0) ||
                ((g_track_reinforced != 0U) &&
                 (matches_confirmed_beacon_track(comp) != 0U))) ?
                     BEACON_EDGE_MIN_AREA :
                     BEACON_MIN_COMPONENT_AREA;
    if((comp->min_y < BEACON_EDGE_TOP_Y) &&
       (comp->area > BEACON_TOP_EDGE_MAX_AREA))
    {
        return;
    }
    if((is_side_edge != 0) && (comp->area > BEACON_EDGE_MAX_AREA))
    {
        return;
    }
    if(comp->area < min_area)
    {
        return;
    }
    if((is_incomplete_border_component(comp) != 0U) &&
       (matches_confirmed_beacon_track(comp) == 0U))
    {
        return;
    }
    if((is_component_in_lamp_core(comp, lamp) != 0) ||
       (is_component_in_lamp_core(comp, temporal_lamp) != 0) ||
       (is_component_in_lamp_core(comp, &g_secondary_lamp_mask) != 0) ||
       (is_component_in_lamp_core(
            comp, &g_secondary_temporal_lamp_mask) != 0))
    {
        return;
    }
    if(((is_near_lamp(comp, lamp) != 0) ||
        (is_near_lamp(comp, temporal_lamp) != 0) ||
        (is_near_lamp(comp, &g_secondary_lamp_mask) != 0) ||
        (is_near_lamp(comp, &g_secondary_temporal_lamp_mask) != 0)) &&
       (comp->area < LAMP_NEAR_BEACON_MIN_AREA) &&
       (is_isolated_near_lamp_beacon(image, comp) == 0))
    {
        return;
    }
    if(is_untracked_top_corner_component(comp) != 0U)
    {
        return;
    }

    slot = result->beacon_count;
    if(slot >= BEACON_MAX_BEACON_COUNT)
    {
        slot = BEACON_MAX_BEACON_COUNT - 1;
        if((float)comp->area <= beacon_area(&result->beacons[slot]))
        {
            return;
        }
    }
    else
    {
        result->beacon_count++;
    }

    for(i = slot - 1; i >= 0; i--)
    {
        if((float)comp->area <= beacon_area(&result->beacons[i]))
        {
            break;
        }
        result->beacons[i + 1] = result->beacons[i];
    }

    circle.x = comp->cx - (float)BEACON_IMAGE_W * 0.5f;
    circle.y = comp->cy - (float)BEACON_IMAGE_H * 0.5f;
    circle.radius = sqrtf((float)comp->area / PI_F);
    circle.area = (float)comp->area;
    circle.valid = 1;
    result->beacons[i + 1] = circle;
}

static unsigned char is_near_existing_beacon(
    const component_t *comp,
    const beacon_result_t *result)
{
    int i;
    int count = result->beacon_count;
    float x = comp->cx - (float)BEACON_IMAGE_W * 0.5f;
    float y = comp->cy - (float)BEACON_IMAGE_H * 0.5f;
    float max_d2 = BEACON_WEAK_CENTER_DUPLICATE_DISTANCE *
                   BEACON_WEAK_CENTER_DUPLICATE_DISTANCE;

    if(count > BEACON_MAX_BEACON_COUNT)
    {
        count = BEACON_MAX_BEACON_COUNT;
    }
    for(i = 0; i < count; i++)
    {
        float dx;
        float dy;

        if(result->beacons[i].valid == 0U)
        {
            continue;
        }
        dx = result->beacons[i].x - x;
        dy = result->beacons[i].y - y;
        if((dx * dx + dy * dy) <= max_d2)
        {
            return 1U;
        }
    }

    return 0U;
}

static void find_weak_center_beacons(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *lamp,
    const component_t *temporal_lamp,
    beacon_result_t *result,
    unsigned short *evaluated_components)
{
    unsigned char x;
    unsigned char y;

    threshold_image(image, BEACON_WEAK_CENTER_THRESHOLD);
    erase_lamp_from_binary(lamp);
    erase_temporal_lamp_from_binary(temporal_lamp);
    erase_lamp_from_binary(&g_secondary_lamp_mask);
    erase_temporal_lamp_from_binary(&g_secondary_temporal_lamp_mask);
    erase_car_body_from_binary();

    for(y = BEACON_WEAK_CENTER_MIN_Y; y < BEACON_WEAK_CENTER_MAX_Y; y++)
    {
        for(x = BEACON_WEAK_CENTER_MIN_X; x < BEACON_WEAK_CENTER_MAX_X; x++)
        {
            component_t comp;

            if((g_binary[y][x] == 0U) ||
               (g_visit[y][x] == g_visit_generation))
            {
                continue;
            }

            if(*evaluated_components >= BEACON_MAX_EVALUATED_COMPONENTS)
            {
                (void)grow_component_visit(x, y, 0, -1, 0);
                continue;
            }
            comp = grow_component_visit(
                x, y, BEACON_WEAK_CENTER_MIN_AREA,
                BEACON_WEAK_CENTER_MAX_AREA, 0);
            (*evaluated_components)++;
            if((comp.valid == 0U) ||
               (comp.cy < (float)BEACON_WEAK_CENTER_MIN_Y) ||
               (comp.cy >= (float)BEACON_WEAK_CENTER_MAX_Y) ||
               (comp.area < BEACON_WEAK_CENTER_MIN_AREA) ||
               ((comp.cy < (float)BEACON_WEAK_CENTER_FULL_MIN_Y) &&
                (comp.cx <
                 (float)BEACON_WEAK_CENTER_NEAR_RIGHT_MAX_X) &&
                ((comp.area < BEACON_WEAK_CENTER_UPPER_MIN_AREA) ||
                 (comp.area > BEACON_WEAK_CENTER_UPPER_MAX_AREA) ||
                 (component_max_gray(image, &comp) <
                  BEACON_WEAK_CENTER_UPPER_MIN_GRAY) ||
                 (component_max_gray(image, &comp) >
                  BEACON_WEAK_CENTER_UPPER_MAX_GRAY) ||
                 (component_average_gray(image, &comp) >
                  BEACON_WEAK_CENTER_UPPER_MAX_MEAN))) ||
               (comp.area > BEACON_WEAK_CENTER_MAX_AREA) ||
               (comp.elongation > BEACON_WEAK_CENTER_MAX_ELONGATION) ||
               (component_max_gray(image, &comp) < BEACON_WEAK_CENTER_MIN_GRAY) ||
               ((comp.cx >= (float)BEACON_WEAK_CENTER_BASE_MAX_X) &&
                (((comp.cx <
                   (float)BEACON_WEAK_CENTER_NEAR_RIGHT_MAX_X) &&
                  ((comp.area > BEACON_WEAK_CENTER_RIGHT_MAX_AREA) ||
                   ((comp.max_y - comp.min_y) <=
                    (comp.max_x - comp.min_x)) ||
                   (component_max_gray(image, &comp) <
                    BEACON_WEAK_CENTER_RIGHT_MIN_GRAY))) ||
                 ((comp.cx >=
                   (float)BEACON_WEAK_CENTER_NEAR_RIGHT_MAX_X) &&
                  ((comp.cx <
                    (float)BEACON_WEAK_CENTER_FAR_RIGHT_MIN_X) ||
                   (comp.cy <
                    (float)BEACON_WEAK_CENTER_FAR_RIGHT_MIN_Y) ||
                   (comp.area <
                    BEACON_WEAK_CENTER_FAR_RIGHT_MIN_AREA) ||
                   (comp.area >
                    BEACON_WEAK_CENTER_FAR_RIGHT_MAX_AREA) ||
                   ((comp.max_y - comp.min_y) !=
                    (comp.max_x - comp.min_x)) ||
                   (comp.elongation >
                    BEACON_WEAK_CENTER_FAR_RIGHT_MAX_ELONGATION) ||
                   (component_max_gray(image, &comp) <
                    BEACON_WEAK_CENTER_FAR_RIGHT_MIN_GRAY))))) ||
               (local_background_average(image, &comp) > BEACON_WEAK_CENTER_MAX_BG) ||
               (is_near_existing_beacon(&comp, result) != 0U))
            {
                continue;
            }

            insert_beacon_by_area(image, &comp, lamp, temporal_lamp, result);
        }
    }
}
static void find_beacons(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *lamp,
    const component_t *temporal_lamp,
    float scene_mean,
    unsigned char reusable_binary_threshold,
    beacon_result_t *result)
{
    gray_beacon_peak_t peaks[GRAY_BEACON_MAX_PEAKS];
    gray_beacon_candidate_t candidates[GRAY_BEACON_MAX_CANDIDATES];
    uint32 step_start_us;
    unsigned char peak_count;
    unsigned char candidate_count = 0U;
    unsigned char index;

    memset(candidates, 0, sizeof(candidates));
    step_start_us = timer_get(TC_TIME2_CH0);
    peak_count = gray_find_box_peaks(image, lamp, temporal_lamp, peaks);
    g_image_profile.beacon_box_us = timer_get(TC_TIME2_CH0) - step_start_us;

    step_start_us = timer_get(TC_TIME2_CH0);
    for(index = 0U; index < peak_count; index++)
    {
        const gray_beacon_features_t *cached_features = 0;
        gray_beacon_features_t features;
        int x = peaks[index].x;
        int y = peaks[index].y;
        float area;
        float refined_x = (float)x;
        float refined_y = (float)y;

        if(gray_is_point_source(
               image, x, y, scene_mean,
               &area, &refined_x, &refined_y, &features) != 0U)
        {
            if(((int)(refined_x + 0.5f) == x) &&
               ((int)(refined_y + 0.5f) == y))
            {
                cached_features = &features;
            }
            gray_add_candidate(
                candidates, &candidate_count, refined_x, refined_y, area,
                cached_features);
        }
    }
    g_image_profile.beacon_peak_us = timer_get(TC_TIME2_CH0) - step_start_us;

    step_start_us = timer_get(TC_TIME2_CH0);
    gray_add_component_candidates(
        image, lamp, temporal_lamp, scene_mean, reusable_binary_threshold,
        candidates, &candidate_count);
    g_image_profile.beacon_component_us =
        timer_get(TC_TIME2_CH0) - step_start_us;

    step_start_us = timer_get(TC_TIME2_CH0);
    gray_store_candidates(
        image, candidates, candidate_count, scene_mean, result);
    g_image_profile.beacon_store_us = timer_get(TC_TIME2_CH0) - step_start_us;
}

static float square_distance(float ax, float ay, float bx, float by)
{
    float dx = ax - bx;
    float dy = ay - by;

    return dx * dx + dy * dy;
}

static float beacon_area(const beacon_circle_t *beacon)
{
    if((beacon == 0) || (beacon->valid == 0))
    {
        return 0.0f;
    }

    return PI_F * beacon->radius * beacon->radius;
}

static void reset_track(temporal_track_t *track)
{
    memset(track, 0, sizeof(*track));
}

static void start_pending_beacon(temporal_track_t *track, const beacon_circle_t *beacon)
{
    reset_track(track);
    track->active = 1U;
    track->hits = 1U;
    track->x = beacon->x;
    track->y = beacon->y;
    track->radius = beacon->radius;
    track->area = beacon_area(beacon);
}

static void start_pending_car(temporal_track_t *track, const beacon_rect_t *car)
{
    reset_track(track);
    track->active = 1U;
    track->hits = 1U;
    track->x = car->cx;
    track->y = car->cy;
    track->width = car->width;
    track->length = car->length;
    track->angle = car->angle;
    track->area = car->length * car->width;
}

static void update_beacon_track(temporal_track_t *track, const beacon_circle_t *beacon)
{
    float old_x = track->x;
    float old_y = track->y;
    float predict_x = track->x + track->vx;
    float predict_y = track->y + track->vy;

    track->vx = (1.0f - FILTER_VEL_ALPHA) * track->vx +
                FILTER_VEL_ALPHA * (beacon->x - old_x);
    track->vy = (1.0f - FILTER_VEL_ALPHA) * track->vy +
                FILTER_VEL_ALPHA * (beacon->y - old_y);
    track->x = FILTER_POS_ALPHA * beacon->x +
               (1.0f - FILTER_POS_ALPHA) * predict_x;
    track->y = FILTER_POS_ALPHA * beacon->y +
               (1.0f - FILTER_POS_ALPHA) * predict_y;
    track->radius = FILTER_POS_ALPHA * beacon->radius +
                    (1.0f - FILTER_POS_ALPHA) * track->radius;
    track->area = beacon_area(beacon);
    track->misses = 0U;
}

static void update_car_track(temporal_track_t *track, const beacon_rect_t *car)
{
    float old_x = track->x;
    float old_y = track->y;
    float predict_x = track->x + track->vx;
    float predict_y = track->y + track->vy;

    track->vx = (1.0f - FILTER_VEL_ALPHA) * track->vx +
                FILTER_VEL_ALPHA * (car->cx - old_x);
    track->vy = (1.0f - FILTER_VEL_ALPHA) * track->vy +
                FILTER_VEL_ALPHA * (car->cy - old_y);
    track->x = FILTER_POS_ALPHA * car->cx +
               (1.0f - FILTER_POS_ALPHA) * predict_x;
    track->y = FILTER_POS_ALPHA * car->cy +
               (1.0f - FILTER_POS_ALPHA) * predict_y;
    track->width = FILTER_POS_ALPHA * car->width +
                   (1.0f - FILTER_POS_ALPHA) * track->width;
    track->length = FILTER_POS_ALPHA * car->length +
                    (1.0f - FILTER_POS_ALPHA) * track->length;
    track->angle = car->angle;
    track->area = car->length * car->width;
    track->misses = 0U;
}

static void write_temporal_beacon(
    const temporal_track_t *track,
    beacon_result_t *result,
    int matched_index)
{
    beacon_circle_t old_beacons[BEACON_MAX_BEACON_COUNT];
    int old_count = result->beacon_count;
    int i;
    int out = 1;

    if(old_count > BEACON_MAX_BEACON_COUNT)
    {
        old_count = BEACON_MAX_BEACON_COUNT;
    }
    memcpy(old_beacons, result->beacons, sizeof(old_beacons));
    memset(result->beacons, 0, sizeof(result->beacons));

    result->beacons[0].x = track->x;
    result->beacons[0].y = track->y;
    result->beacons[0].radius = track->radius;
    result->beacons[0].area = track->area;
    result->beacons[0].valid = 1U;

    for(i = 0; (i < old_count) && (out < BEACON_MAX_BEACON_COUNT); i++)
    {
        if(i == matched_index)
        {
            continue;
        }
        result->beacons[out] = old_beacons[i];
        out++;
    }
    result->beacon_count = (unsigned char)out;
}

static void write_temporal_car(const temporal_track_t *track, beacon_result_t *result)
{
    result->car_lamps[0].cx = track->x;
    result->car_lamps[0].cy = track->y;
    result->car_lamps[0].width = track->width;
    result->car_lamps[0].length = track->length;
    result->car_lamps[0].angle = track->angle;
    result->car_lamps[0].valid = 1U;
    result->car_lamp_count = 1U;
    result->car_lamp_measured_mask = 0U;
}

static void write_current_car_as_temporal(
    const beacon_rect_t *car,
    beacon_result_t *result)
{
    result->car_lamps[0] = *car;
    result->car_lamp_count = 1U;
    result->car_lamp_measured_mask = 0x01U;
}

static unsigned char temporal_car_max_misses(const temporal_track_t *track)
{
    component_t lamp;

    if(component_from_temporal_car(track, &lamp, 1U) == 0)
    {
        return CAR_LAMP_EDGE_MAX_MISSES;
    }
    if((lamp.min_x <= CAR_LAMP_TEMPORAL_EDGE_MARGIN) ||
       (lamp.min_y <= CAR_LAMP_TEMPORAL_EDGE_MARGIN) ||
       (lamp.max_x >= BEACON_IMAGE_W - 1 - CAR_LAMP_TEMPORAL_EDGE_MARGIN) ||
       (lamp.max_y >= BEACON_IMAGE_H - 1 - CAR_LAMP_TEMPORAL_EDGE_MARGIN))
    {
        return CAR_LAMP_EDGE_MAX_MISSES;
    }
    return CAR_LAMP_CENTER_MAX_MISSES;
}

static unsigned char can_use_temporal_car_mask(const temporal_track_t *track)
{
    return ((track != 0) &&
            (track->confirmed != 0U) &&
            (track->misses < temporal_car_max_misses(track))) ? 1 : 0;
}

static unsigned char car_lamp_matches_track_shape_for(
    const component_t *comp,
    const temporal_track_t *track)
{
    float length_ratio;
    float width_ratio;

    if((comp == 0) || (track == 0) || (track->length <= 0.0f) ||
       (track->width <= 0.0f))
    {
        return 0U;
    }
    length_ratio = comp->major / track->length;
    width_ratio = comp->minor / track->width;
    return ((length_ratio >= CAR_LAMP_TRACK_SIZE_MIN_RATIO) &&
            (length_ratio <= CAR_LAMP_TRACK_SIZE_MAX_RATIO) &&
            (width_ratio >= CAR_LAMP_TRACK_SIZE_MIN_RATIO) &&
            (width_ratio <= CAR_LAMP_TRACK_SIZE_MAX_RATIO)) ? 1U : 0U;
}

static unsigned char car_lamp_matches_track_shape(const component_t *comp)
{
    return car_lamp_matches_track_shape_for(comp, &g_car_track);
}

static unsigned char find_temporal_car_lamp_for_track(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const temporal_track_t *track,
    component_t *best_lamp)
{
    component_t temporal_lamp;
    component_t best_comp;
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    int x;
    int y;
    float best_distance2 = 0.0f;
    unsigned char found = 0;

    if((image == 0) || (best_lamp == 0) ||
       (can_use_temporal_car_mask(track) == 0) ||
       (component_from_temporal_car(track, &temporal_lamp, 1U) == 0))
    {
        return 0;
    }

    memset(&best_comp, 0, sizeof(best_comp));
    min_x = temporal_lamp.min_x - CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
    max_x = temporal_lamp.max_x + CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
    min_y = temporal_lamp.min_y - CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
    max_y = temporal_lamp.max_y + CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
    if(min_x < 0) min_x = 0;
    if(min_y < 0) min_y = 0;
    if(max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if(max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;
    if((max_x < 0) || (max_y < 0) ||
       (min_x >= BEACON_IMAGE_W) || (min_y >= BEACON_IMAGE_H) ||
       (min_x > max_x) || (min_y > max_y))
    {
        return 0;
    }

    (void)threshold_car_lamp_image(image);
    for(y = min_y; y <= max_y; y++)
    {
        for(x = min_x; x <= max_x; x++)
        {
            component_t comp;
            component_orientation_t orientation;

            if((g_binary[y][x] == 0) ||
               (g_visit[y][x] == g_visit_generation))
            {
                continue;
            }

            comp = grow_component_visit(
                (unsigned char)x, (unsigned char)y,
                CAR_LAMP_MIN_AREA,
                CAR_LAMP_MAX_AREA, &orientation);
            if(is_component_in_lamp_core(&comp, &temporal_lamp) == 0)
            {
                continue;
            }
            if(is_lamp_geometry_candidate(&comp) == 0U)
            {
                continue;
            }
            if(car_lamp_matches_track_shape_for(&comp, track) == 0U)
            {
                continue;
            }
            comp.angle = component_orientation_angle(&orientation);
            if(car_lamp_uniform_strip_valid(image, &comp) == 0U)
            {
                continue;
            }
            {
                float dx = comp.cx - temporal_lamp.cx;
                float dy = comp.cy - temporal_lamp.cy;
                float distance2 = dx * dx + dy * dy;
                if((found == 0U) || (distance2 < best_distance2))
                {
                    best_comp = comp;
                    best_distance2 = distance2;
                    found = 1U;
                }
            }
        }
    }

    if(found != 0)
    {
        *best_lamp = best_comp;
    }
    return found;
}

static unsigned char find_temporal_car_lamp(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    component_t *best_lamp)
{
    return find_temporal_car_lamp_for_track(image, &g_car_track, best_lamp);
}

static unsigned char gray_car_lamp_valid(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    component_t *comp,
    const component_orientation_t *orientation)
{
    unsigned int sum = 0U;
    unsigned int background_sum = 0U;
    int count = 0;
    int background_count = 0;
    int strip_count = 0;
    int strip_bright = 0;
    int box_area;
    int x;
    int y;
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    unsigned char peak = 0U;
    float angle;
    float cos_a;
    float sin_a;
    float half_length;
    float half_width;
    float mean;
    float background;

    if((image == 0) || (comp == 0) || (comp->valid == 0U) ||
       (comp->cy < (float)CAR_LAMP_MIN_CENTER_Y) ||
       (comp->area < CAR_LAMP_GRAY_MIN_AREA) ||
       (comp->area > CAR_LAMP_GRAY_MAX_AREA) ||
       (comp->major < CAR_LAMP_GRAY_MIN_LENGTH) ||
       (comp->major > CAR_LAMP_GRAY_MAX_LENGTH) ||
       (comp->minor < CAR_LAMP_GRAY_MIN_WIDTH) ||
       (comp->elongation < CAR_LAMP_GRAY_MIN_ELONGATION) ||
       (car_lamp_below_horizon(comp) == 0U))
    {
        return 0U;
    }

    box_area = (comp->max_x - comp->min_x + 1) *
               (comp->max_y - comp->min_y + 1);
    if((box_area <= 0) ||
       (comp->area * 100 < box_area * CAR_LAMP_GRAY_MIN_FILL_PERCENT))
    {
        return 0U;
    }
    comp->angle = component_orientation_angle(orientation);
    angle = comp->angle * (PI_F / 180.0f);
    cos_a = cosf(angle);
    sin_a = sinf(angle);
    half_length = comp->major * 0.5f + 0.5f;
    half_width = comp->minor * 0.5f + 0.5f;
    min_x = (int)floorf(comp->cx - half_length - half_width);
    max_x = (int)ceilf(comp->cx + half_length + half_width);
    min_y = (int)floorf(comp->cy - half_length - half_width);
    max_y = (int)ceilf(comp->cy + half_length + half_width);
    if(min_x < 0) min_x = 0;
    if(min_y < 0) min_y = 0;
    if(max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if(max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;
    for(y = min_y; y <= max_y; y++)
    {
        float dy = (float)y - comp->cy;
        float dy_sin = dy * sin_a;
        float dy_cos = dy * cos_a;

        for(x = min_x; x <= max_x; x++)
        {
            float dx = (float)x - comp->cx;
            float major = dx * cos_a + dy_sin;
            float minor = -dx * sin_a + dy_cos;

            if((fabsf(major) > half_length) ||
               (fabsf(minor) > half_width))
            {
                continue;
            }
            strip_count++;
            if(image[y][x] >= CAR_LAMP_GRAY_STRIP_MIN_GRAY)
            {
                strip_bright++;
            }
        }
    }
    if((strip_count == 0) ||
       (strip_bright * 100 <
        strip_count * CAR_LAMP_GRAY_TRACK_MIN_FILL_PERCENT))
    {
        return 0U;
    }
    if(strip_bright * 100 <
       strip_count * CAR_LAMP_GRAY_STRIP_MIN_FILL_PERCENT)
    {
        component_t temporal_lamp;

        if((g_car_track.confirmed == 0U) ||
           (component_from_temporal_car(
                &g_car_track, &temporal_lamp, 1U) == 0U) ||
           (is_component_in_lamp_core(comp, &temporal_lamp) == 0U) ||
           (car_lamp_matches_track_shape(comp) == 0U))
        {
            return 0U;
        }
    }
    for(y = comp->min_y; y <= comp->max_y; y++)
    {
        for(x = comp->min_x; x <= comp->max_x; x++)
        {
            unsigned char gray;

            gray = image[y][x];
            if(g_binary[y][x] == 0U)
            {
                continue;
            }
            sum += gray;
            count++;
            if(gray > peak)
            {
                peak = gray;
            }
        }
    }
    min_x = comp->min_x - 3;
    max_x = comp->max_x + 3;
    min_y = comp->min_y - 3;
    max_y = comp->max_y + 3;
    if(min_x < 0) min_x = 0;
    if(min_y < 0) min_y = 0;
    if(max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if(max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;
    for(y = min_y; y <= max_y; y++)
    {
        for(x = min_x; x <= max_x; x++)
        {
            if((x >= comp->min_x) && (x <= comp->max_x) &&
               (y >= comp->min_y) && (y <= comp->max_y))
            {
                continue;
            }
            background_sum += image[y][x];
            background_count++;
        }
    }
    if((count == 0) || (background_count == 0))
    {
        return 0U;
    }
    mean = (float)sum / (float)count;
    background = (float)background_sum / (float)background_count;
    return ((peak >= CAR_LAMP_GRAY_MIN_PEAK) &&
            (mean >= CAR_LAMP_GRAY_MIN_MEAN) &&
            (mean - background >= CAR_LAMP_GRAY_MIN_CONTRAST)) ? 1U : 0U;
}

#if defined(__ICCARM__)
#pragma inline=never
#endif
static unsigned char find_gray_car_lamp(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    float scene_mean,
    component_t *best_lamp)
{
    int x;
    int y;
    float best_score = 0.0f;
    unsigned char found = 0U;

    if((image == 0) || (best_lamp == 0) ||
       (scene_mean >= CAR_LAMP_GRAY_SCENE_MAX))
    {
        return 0U;
    }
    memset(best_lamp, 0, sizeof(*best_lamp));
    threshold_image(image, CAR_LAMP_GRAY_THRESHOLD);
    for(y = CAR_LAMP_MIN_CENTER_Y; y < BEACON_IMAGE_H; y++)
    {
        for(x = 0; x < BEACON_IMAGE_W; x++)
        {
            component_t comp;
            component_orientation_t orientation;
            float score;

#if defined(__ICCARM__)
            if(((x & 3) == 0) &&
               (__UNALIGNED_UINT32_READ(&g_binary[y][x]) == 0U))
            {
                x += 3;
                continue;
            }
#endif
            if((g_binary[y][x] == 0U) ||
               (g_visit[y][x] == g_visit_generation))
            {
                continue;
            }
            comp = grow_component_visit((unsigned char)x, (unsigned char)y,
                                        CAR_LAMP_GRAY_MIN_AREA,
                                        CAR_LAMP_GRAY_MAX_AREA, &orientation);
            if(gray_car_lamp_valid(image, &comp, &orientation) == 0U)
            {
                continue;
            }
            score = (float)comp.area * comp.elongation;
            if((found == 0U) ||
               (comp.cy > best_lamp->cy + CAR_LAMP_Y_PRIORITY_MARGIN) ||
               ((fabsf(comp.cy - best_lamp->cy) <=
                 CAR_LAMP_Y_PRIORITY_MARGIN) &&
                (score > best_score)))
            {
                *best_lamp = comp;
                best_score = score;
                found = 1U;
            }
        }
    }
    return found;
}

static unsigned char find_compatible_gray_car_lamp(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    float scene_mean,
    const component_t *primary,
    component_t *best_lamp)
{
    int x;
    int y;
    float best_score = 0.0f;
    unsigned char found = 0U;

    if((image == 0) || (primary == 0) || (best_lamp == 0) ||
       (primary->valid == 0U) ||
       (scene_mean >= CAR_LAMP_GRAY_SCENE_MAX))
    {
        return 0U;
    }
    memset(best_lamp, 0, sizeof(*best_lamp));
    threshold_image(image, CAR_LAMP_GRAY_THRESHOLD);
    for(y = CAR_LAMP_MIN_CENTER_Y; y < BEACON_IMAGE_H; y++)
    {
        for(x = 0; x < BEACON_IMAGE_W; x++)
        {
            component_t comp;
            component_orientation_t orientation;
            float score;

            if((g_binary[y][x] == 0U) ||
               (g_visit[y][x] == g_visit_generation))
            {
                continue;
            }
            comp = grow_component_visit((unsigned char)x, (unsigned char)y,
                                        CAR_LAMP_GRAY_MIN_AREA,
                                        CAR_LAMP_GRAY_MAX_AREA, &orientation);
            if((gray_car_lamp_valid(image, &comp, &orientation) == 0U) ||
               (car_lamp_pair_compatible(primary, &comp) == 0U))
            {
                continue;
            }
            score = (float)comp.area * comp.elongation;
            if((found == 0U) ||
               (comp.cy > best_lamp->cy + CAR_LAMP_Y_PRIORITY_MARGIN) ||
               ((fabsf(comp.cy - best_lamp->cy) <=
                 CAR_LAMP_Y_PRIORITY_MARGIN) &&
                (score > best_score)))
            {
                *best_lamp = comp;
                best_score = score;
                found = 1U;
            }
        }
    }
    return found;
}

static int nearest_beacon_index(
    const beacon_result_t *result,
    float x,
    float y,
    float max_distance)
{
    int i;
    int best = -1;
    int count = result->beacon_count;
    float best_d2 = max_distance * max_distance;

    if(count > BEACON_MAX_BEACON_COUNT)
    {
        count = BEACON_MAX_BEACON_COUNT;
    }
    for(i = 0; i < count; i++)
    {
        float d2;
        const beacon_circle_t *beacon = &result->beacons[i];

        if(beacon->valid == 0U)
        {
            continue;
        }
        d2 = square_distance(beacon->x, beacon->y, x, y);
        if(d2 <= best_d2)
        {
            best_d2 = d2;
            best = i;
        }
    }

    return best;
}

static void update_temporal_beacon(beacon_result_t *result)
{
    int count = result->beacon_count;
    int selected = -1;
    beacon_circle_t *measurement;
    const float gate = B0_MATCH_DISTANCE;

    if(count > BEACON_MAX_BEACON_COUNT)
    {
        count = BEACON_MAX_BEACON_COUNT;
    }

    if(count <= 0)
    {
        if((g_b0_track.confirmed != 0U) &&
           (g_b0_track.area > BEACON_TINY_TRACK_MAX_AREA) &&
           (g_b0_track.misses < BEACON_MAX_MISSES))
        {
            g_b0_track.x += g_b0_track.vx;
            g_b0_track.y += g_b0_track.vy;
            g_b0_track.misses++;
            return;
        }
        reset_track(&g_b0_track);
        return;
    }

    if(g_b0_track.confirmed != 0U)
    {
        float predict_x = g_b0_track.x + g_b0_track.vx;
        float predict_y = g_b0_track.y + g_b0_track.vy;

        selected = nearest_beacon_index(result, predict_x, predict_y, gate);
        if(selected > 0)
        {
            float b0_area = beacon_area(&result->beacons[0]);
            float selected_area = beacon_area(&result->beacons[selected]);
            float switch_ratio = B0_SWITCH_AREA_RATIO;

            if((b0_area < B0_SMALL_SWITCH_AREA) &&
               (selected_area < B0_SMALL_SWITCH_AREA))
            {
                switch_ratio = B0_SMALL_SWITCH_RATIO;
            }
            if(b0_area > selected_area * switch_ratio)
            {
                selected = 0;
            }
        }
    }
    else
    {
        selected = 0;
    }

    if(selected < 0)
    {
        start_pending_beacon(&g_b0_track, &result->beacons[0]);
        return;
    }

    measurement = &result->beacons[selected];
    if(g_b0_track.active == 0U)
    {
        start_pending_beacon(&g_b0_track, measurement);
        return;
    }

    if(g_b0_track.confirmed == 0U)
    {
        if(square_distance(g_b0_track.x, g_b0_track.y,
                           measurement->x, measurement->y) >
           KALMAN_NEW_TARGET_DISTANCE * KALMAN_NEW_TARGET_DISTANCE)
        {
            start_pending_beacon(&g_b0_track, measurement);
            return;
        }
        update_beacon_track(&g_b0_track, measurement);
        g_b0_track.hits++;
        if(g_b0_track.hits >= B0_INIT_CONFIRM_FRAMES)
        {
            g_b0_track.confirmed = 1U;
            write_temporal_beacon(&g_b0_track, result, selected);
        }
        return;
    }

    if(square_distance(g_b0_track.x + g_b0_track.vx,
                       g_b0_track.y + g_b0_track.vy,
                       measurement->x,
                       measurement->y) >
       KALMAN_NEW_TARGET_DISTANCE * KALMAN_NEW_TARGET_DISTANCE)
    {
        start_pending_beacon(&g_b0_track, measurement);
        return;
    }

    update_beacon_track(&g_b0_track, measurement);
    write_temporal_beacon(&g_b0_track, result, selected);
}

static void update_temporal_car(beacon_result_t *result)
{
    beacon_rect_t *measurement = 0;

    if((result->car_lamp_count > 0U) &&
       (result->car_lamps[0].valid != 0U))
    {
        measurement = &result->car_lamps[0];
    }

    if(measurement == 0)
    {
        if((g_car_track.confirmed != 0U) &&
           (g_car_track.misses < temporal_car_max_misses(&g_car_track)))
        {
            component_t predicted_lamp;
            g_car_track.x += g_car_track.vx;
            g_car_track.y += g_car_track.vy;
            g_car_track.misses++;
            if(component_from_temporal_car(
                   &g_car_track, &predicted_lamp, 0U) == 0)
            {
                reset_track(&g_car_track);
                return;
            }
            write_temporal_car(&g_car_track, result);
            return;
        }
        reset_track(&g_car_track);
        return;
    }

    if(g_car_track.active == 0U)
    {
        start_pending_car(&g_car_track, measurement);
        write_current_car_as_temporal(measurement, result);
        return;
    }

    if((g_car_track.confirmed != 0U) &&
       (square_distance(g_car_track.x + g_car_track.vx,
                        g_car_track.y + g_car_track.vy,
                        measurement->cx, measurement->cy) >
        KALMAN_GATE_DISTANCE * KALMAN_GATE_DISTANCE))
    {
        start_pending_car(&g_car_track, measurement);
        write_current_car_as_temporal(measurement, result);
        return;
    }

    if((g_car_track.confirmed == 0U) &&
       (square_distance(g_car_track.x, g_car_track.y,
                        measurement->cx, measurement->cy) >
        KALMAN_NEW_TARGET_DISTANCE * KALMAN_NEW_TARGET_DISTANCE))
    {
        start_pending_car(&g_car_track, measurement);
        write_current_car_as_temporal(measurement, result);
        return;
    }

    update_car_track(&g_car_track, measurement);
    g_car_track.hits++;
    if((g_car_track.confirmed == 0U) &&
       (g_car_track.hits >= B0_INIT_CONFIRM_FRAMES))
    {
        g_car_track.confirmed = 1U;
    }
    if(g_car_track.confirmed != 0U)
    {
        write_current_car_as_temporal(measurement, result);
    }
}

static unsigned char update_dual_car_track(
    temporal_track_t *track,
    const beacon_rect_t *measurement)
{
    if(measurement == 0)
    {
        if((track->confirmed != 0U) &&
           (track->misses < temporal_car_max_misses(track)))
        {
            component_t predicted_lamp;

            track->x += track->vx;
            track->y += track->vy;
            track->misses++;
            if(component_from_temporal_car(track, &predicted_lamp, 0U) != 0U)
            {
                return 1U;
            }
        }
        reset_track(track);
        return 0U;
    }
    if(track->active == 0U)
    {
        start_pending_car(track, measurement);
        return 0U;
    }
    if((track->confirmed != 0U) &&
       (square_distance(track->x + track->vx, track->y + track->vy,
                        measurement->cx, measurement->cy) >
        KALMAN_GATE_DISTANCE * KALMAN_GATE_DISTANCE))
    {
        start_pending_car(track, measurement);
        return 0U;
    }
    if((track->confirmed == 0U) &&
       (square_distance(track->x, track->y,
                        measurement->cx, measurement->cy) >
        KALMAN_NEW_TARGET_DISTANCE * KALMAN_NEW_TARGET_DISTANCE))
    {
        start_pending_car(track, measurement);
        return 0U;
    }
    update_car_track(track, measurement);
    if(track->hits < 255U)
    {
        track->hits++;
    }
    if((track->confirmed == 0U) &&
       (track->hits >= B0_INIT_CONFIRM_FRAMES))
    {
        track->confirmed = 1U;
    }
    return 0U;
}

static void append_car_rect(
    const beacon_rect_t *car,
    unsigned char measured,
    beacon_result_t *result)
{
    if((car == 0) || (car->valid == 0U) ||
       (result->car_lamp_count >= BEACON_MAX_CAR_LAMP_COUNT))
    {
        return;
    }
    result->car_lamps[result->car_lamp_count] = *car;
    if(measured != 0U)
    {
        result->car_lamp_measured_mask |=
            (unsigned char)(1U << result->car_lamp_count);
    }
    result->car_lamp_count++;
}

static void update_temporal_cars_dual(
    const component_t lamps[BEACON_MAX_CAR_LAMP_COUNT],
    unsigned char lamp_count,
    beacon_result_t *result)
{
    temporal_track_t *tracks[BEACON_MAX_CAR_LAMP_COUNT];
    beacon_rect_t measurements[BEACON_MAX_CAR_LAMP_COUNT];
    beacon_rect_t outputs[BEACON_MAX_CAR_LAMP_COUNT];
    signed char assigned[BEACON_MAX_CAR_LAMP_COUNT] = {-1, -1};
    unsigned char output_measured[BEACON_MAX_CAR_LAMP_COUNT] = {0U, 0U};
    unsigned char primary_track = 0xFFU;
    unsigned char index;

    tracks[0] = &g_car_track;
    tracks[1] = &g_car_track_secondary;
    memset(measurements, 0, sizeof(measurements));
    memset(outputs, 0, sizeof(outputs));
    if(lamp_count > BEACON_MAX_CAR_LAMP_COUNT)
    {
        lamp_count = BEACON_MAX_CAR_LAMP_COUNT;
    }
    for(index = 0U; index < lamp_count; index++)
    {
        measurements[index].cx =
            lamps[index].cx - (float)BEACON_IMAGE_W * 0.5f;
        measurements[index].cy =
            lamps[index].cy - (float)BEACON_IMAGE_H * 0.5f;
        measurements[index].width = lamps[index].minor;
        measurements[index].length = lamps[index].major;
        measurements[index].angle = lamps[index].angle;
        measurements[index].valid = lamps[index].valid;
    }

    if(lamp_count > 0U)
    {
        assigned[0] = 0;
    }
    if(lamp_count > 1U)
    {
        assigned[1] = 1;
    }

    for(index = 0U; index < BEACON_MAX_CAR_LAMP_COUNT; index++)
    {
        const beacon_rect_t *measurement =
            (assigned[index] >= 0) ? &measurements[assigned[index]] : 0;
        unsigned char predicted = update_dual_car_track(
            tracks[index], measurement);

        if(measurement != 0)
        {
            outputs[index] = *measurement;
            output_measured[index] = 1U;
            if(assigned[index] == 0)
            {
                primary_track = index;
            }
        }
        else if(predicted != 0U)
        {
            outputs[index].cx = tracks[index]->x;
            outputs[index].cy = tracks[index]->y;
            outputs[index].width = tracks[index]->width;
            outputs[index].length = tracks[index]->length;
            outputs[index].angle = tracks[index]->angle;
            outputs[index].valid = 1U;
        }
    }

    memset(result->car_lamps, 0, sizeof(result->car_lamps));
    result->car_lamp_count = 0U;
    result->car_lamp_measured_mask = 0U;
    if(lamp_count == 0U)
    {
        if(outputs[0].valid != 0U)
        {
            append_car_rect(&outputs[0], output_measured[0], result);
            append_car_rect(&outputs[1], output_measured[1], result);
        }
        return;
    }

    if(primary_track < BEACON_MAX_CAR_LAMP_COUNT)
    {
        append_car_rect(&outputs[primary_track], 1U, result);
    }
    for(index = 0U; index < BEACON_MAX_CAR_LAMP_COUNT; index++)
    {
        if(index != primary_track)
        {
            append_car_rect(&outputs[index], output_measured[index], result);
        }
    }
}

static unsigned char beacon_output_embedded_in_structure(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const beacon_circle_t *beacon)
{
    unsigned char ring_histogram[256];
    int cx;
    int cy;
    int outer20 = 0;
    int outer40 = 0;
    int count40 = 0;
    int relative_outer10 = 0;
    int relative_outer20 = 0;
    int relative_outer30 = 0;
    int neighborhood_sum = 0;
    int neighborhood_count = 0;
    int ring_count = 0;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_xx = 0.0f;
    float sum_yy = 0.0f;
    float sum_xy = 0.0f;
    int dx;
    int dy;

    if((beacon == 0) || (beacon->area > BEACON_OUTPUT_TINY_AREA_MAX))
    {
        return 0U;
    }
    cx = (int)(beacon->x + (float)BEACON_IMAGE_W * 0.5f + 0.5f);
    cy = (int)(beacon->y + (float)BEACON_IMAGE_H * 0.5f + 0.5f);
    memset(ring_histogram, 0, sizeof(ring_histogram));
    for(dy = -12; dy <= 12; dy++)
    {
        for(dx = -12; dx <= 12; dx++)
        {
            int x = cx + dx;
            int y = cy + dy;
            int radius2 = dx * dx + dy * dy;
            int pixel;

            if((radius2 > 144) || (x < 0) || (x >= BEACON_IMAGE_W) ||
               (y < 0) || (y >= BEACON_IMAGE_H))
            {
                continue;
            }
            pixel = image[y][x];
            neighborhood_sum += pixel;
            neighborhood_count++;
            if((radius2 >= 25) && (radius2 <= 49))
            {
                ring_histogram[pixel]++;
                ring_count++;
            }
            if(radius2 > 49)
            {
                if(pixel >= 20) outer20++;
                if(pixel >= 40) outer40++;
            }
            if(pixel >= 40)
            {
                count40++;
                sum_x += (float)dx;
                sum_y += (float)dy;
                sum_xx += (float)(dx * dx);
                sum_yy += (float)(dy * dy);
                sum_xy += (float)(dx * dy);
            }
        }
    }
    if(ring_count > 0)
    {
        int cumulative = 0;
        int background;
        int rank = ring_count / 2;

        for(background = 0; background < 256; background++)
        {
            cumulative += ring_histogram[background];
            if(cumulative > rank)
            {
                break;
            }
        }
        if((background >= BEACON_OUTPUT_REFLECTION_BG_MIN) &&
           (background <= BEACON_OUTPUT_REFLECTION_BG_MAX))
        {
            for(dy = -12; dy <= 12; dy++)
            {
                for(dx = -12; dx <= 12; dx++)
                {
                    int x = cx + dx;
                    int y = cy + dy;
                    int radius2 = dx * dx + dy * dy;
                    int pixel;

                    if((radius2 <= 49) || (radius2 > 144) ||
                       (x < 0) || (x >= BEACON_IMAGE_W) ||
                       (y < 0) || (y >= BEACON_IMAGE_H))
                    {
                        continue;
                    }
                    pixel = image[y][x];
                    if(pixel >= background + 10) relative_outer10++;
                    if(pixel >= background + 20) relative_outer20++;
                    if(pixel >= background + 30) relative_outer30++;
                }
            }
            if((relative_outer10 >= BEACON_OUTPUT_OUTER_10_MIN) &&
               (relative_outer20 >= BEACON_OUTPUT_OUTER_20_MIN) &&
               (relative_outer30 <= BEACON_OUTPUT_OUTER_30_MAX))
            {
                return 1U;
            }
        }
    }
    if((neighborhood_count == 0) ||
       (neighborhood_sum >= 15 * neighborhood_count))
    {
        return 0U;
    }
    if(outer20 > 45)
    {
        return 1U;
    }
    if((outer40 >= 10) && (count40 >= 2))
    {
        float mean_x = sum_x / (float)count40;
        float mean_y = sum_y / (float)count40;
        float var_x = sum_xx / (float)count40 - mean_x * mean_x;
        float var_y = sum_yy / (float)count40 - mean_y * mean_y;
        float covariance = sum_xy / (float)count40 - mean_x * mean_y;
        float discriminant = sqrtf((var_x - var_y) * (var_x - var_y) +
                                   4.0f * covariance * covariance);
        float major = (var_x + var_y + discriminant) * 0.5f;
        float minor = (var_x + var_y - discriminant) * 0.5f;

        if((minor <= 0.0001f) || (major >= 16.0f * minor))
        {
            return 1U;
        }
    }
    return 0U;
}

static unsigned char beacon_output_matches_horizon(
    const beacon_circle_t *beacon)
{
    int column;
    float image_y;
    float depth;

    if(beacon == 0)
    {
        return 0U;
    }
    column = (int)(beacon->x + (float)BEACON_IMAGE_W * 0.5f + 0.5f);
    if((g_image_horizon_valid == 0U) ||
       (column < 0) || (column >= BEACON_IMAGE_W) ||
       (g_image_horizon_column_valid[column] == 0U))
    {
        return 1U;
    }
    image_y = beacon->y + (float)BEACON_IMAGE_H * 0.5f;
    depth = image_y - g_image_horizon_y[column];
    if(depth < -BEACON_OUTPUT_HORIZON_TOLERANCE)
    {
        return 0U;
    }
    if((depth <= BEACON_OUTPUT_HORIZON_SMALL_BAND) &&
       (beacon->area > BEACON_OUTPUT_TINY_AREA_MAX))
    {
        return 0U;
    }
    if((beacon->area <= BEACON_OUTPUT_TINY_AREA_MAX) &&
       (depth >= BEACON_OUTPUT_TINY_DEPTH_MIN))
    {
        return 0U;
    }
    return 1U;
}

static unsigned char beacon_output_secondary_reliable(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const beacon_circle_t *beacon)
{
    int center_x;
    int center_y;
    int x;
    int y;
    unsigned char peak = 0U;

    if((image == 0) || (beacon == 0))
    {
        return 0U;
    }
    if(beacon->area > 8.0f)
    {
        return 1U;
    }
    center_x = (int)(beacon->x + (float)BEACON_IMAGE_W * 0.5f + 0.5f);
    center_y = (int)(beacon->y + (float)BEACON_IMAGE_H * 0.5f + 0.5f);
    for(y = center_y - 3; y <= center_y + 3; y++)
    {
        for(x = center_x - 3; x <= center_x + 3; x++)
        {
            if((x >= 0) && (x < BEACON_IMAGE_W) &&
               (y >= 0) && (y < BEACON_IMAGE_H) &&
               (image[y][x] > peak))
            {
                peak = image[y][x];
            }
        }
    }
    if(peak >= 120U)
    {
        return 1U;
    }
    return ((center_y >= BEACON_IMAGE_H / 8) &&
            (center_y <= BEACON_IMAGE_H * 2 / 3)) ? 1U : 0U;
}

static void beacon_output_accept(const beacon_circle_t *beacon)
{
    if(g_b0_track.active != 0U)
    {
        float dx = beacon->x - g_b0_track.x;
        float dy = beacon->y - g_b0_track.y;

        if(dx * dx + dy * dy >
           BEACON_OUTPUT_TRACK_GATE * BEACON_OUTPUT_TRACK_GATE)
        {
            g_b0_track.vx = 0.0f;
            g_b0_track.vy = 0.0f;
        }
        else
        {
            g_b0_track.vx =
                (1.0f - FILTER_VEL_ALPHA) * g_b0_track.vx +
                FILTER_VEL_ALPHA * dx;
            g_b0_track.vy =
                (1.0f - FILTER_VEL_ALPHA) * g_b0_track.vy +
                FILTER_VEL_ALPHA * dy;
        }
    }
    else
    {
        g_b0_track.vx = 0.0f;
        g_b0_track.vy = 0.0f;
    }
    g_b0_track.active = 1U;
    g_b0_track.confirmed = 1U;
    g_b0_track.hits = 1U;
    g_b0_track.misses = 0U;
    g_b0_track.x = beacon->x;
    g_b0_track.y = beacon->y;
    g_b0_track.radius = beacon->radius;
    g_b0_track.area = beacon->area;
}

static void beacon_output_predict(beacon_result_t *result)
{
    beacon_circle_t *output;

    if((g_b0_track.active == 0U) ||
       (g_b0_track.misses >= BEACON_OUTPUT_PREDICT_FRAMES))
    {
        reset_track(&g_b0_track);
        return;
    }
    g_b0_track.x += g_b0_track.vx;
    g_b0_track.y += g_b0_track.vy;
    g_b0_track.vx *= 0.9f;
    g_b0_track.vy *= 0.9f;
    g_b0_track.misses++;

    output = &result->beacons[0];
    output->x = g_b0_track.x;
    output->y = g_b0_track.y;
    output->radius = g_b0_track.radius;
    output->area = g_b0_track.area;
    output->valid = 1U;
    result->beacon_count = 1U;
}

static void beacon_output_finalize(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result)
{
    beacon_circle_t candidates[BEACON_MAX_BEACON_COUNT];
    unsigned char candidate_count = 0U;
    unsigned char source_count = result->beacon_count;
    unsigned char source_index;

    if(source_count > BEACON_MAX_BEACON_COUNT)
    {
        source_count = BEACON_MAX_BEACON_COUNT;
    }
    for(source_index = 0U; source_index < source_count; source_index++)
    {
        const beacon_circle_t *source = &result->beacons[source_index];
        unsigned char insert_at;

        if((source->valid == 0U) ||
           (car_body_mask_contains(
                (int)(source->x + (float)BEACON_IMAGE_W * 0.5f + 0.5f),
                (int)(source->y + (float)BEACON_IMAGE_H * 0.5f + 0.5f)) != 0U) ||
           (beacon_output_embedded_in_structure(image, source) != 0U) ||
           (beacon_output_matches_horizon(source) == 0U))
        {
            continue;
        }
        insert_at = candidate_count;
        while((insert_at > 0U) &&
              (source->area > candidates[insert_at - 1U].area))
        {
            candidates[insert_at] = candidates[insert_at - 1U];
            insert_at--;
        }
        candidates[insert_at] = *source;
        candidate_count++;
    }

    memset(result->beacons, 0, sizeof(result->beacons));
    result->beacon_count = 0U;
    if(candidate_count == 0U)
    {
        beacon_output_predict(result);
        return;
    }

    result->beacons[0] = candidates[0];
    result->beacon_count = 1U;
    beacon_output_accept(&candidates[0]);
    if((candidate_count > 1U) &&
       (beacon_output_secondary_reliable(image, &candidates[1]) != 0U))
    {
        result->beacons[1] = candidates[1];
        result->beacon_count = 2U;
    }
}

static void beacon_image_process(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result)
{
    component_t lamp;
    component_t lamps[BEACON_MAX_CAR_LAMP_COUNT];
    component_t secondary_lamp;
    component_t temporal_lamp;
    component_t secondary_temporal_lamp;
    uint32 scene_sum;
    uint32 step_start_us;
    unsigned char has_lamp;
    unsigned char has_secondary_lamp = 0U;
    unsigned char lamp_count = 0U;
    unsigned char reusable_binary_threshold = 0U;
    float scene_mean;

    if(result == 0)
    {
        return;
    }

    clear_result(result);
    if(image == 0)
    {
        return;
    }

    memset(lamps, 0, sizeof(lamps));
    memset(&secondary_lamp, 0, sizeof(secondary_lamp));
    memset(&g_secondary_lamp_mask, 0, sizeof(g_secondary_lamp_mask));
    memset(&g_secondary_temporal_lamp_mask, 0,
           sizeof(g_secondary_temporal_lamp_mask));
    g_car_body_mask.valid = 0U;

    if((g_car_track.confirmed != 0U) &&
       (g_car_track.y + (float)BEACON_IMAGE_H * 0.5f + g_car_track.vy <
        (float)CAR_LAMP_MIN_CENTER_Y))
    {
        reset_track(&g_car_track);
    }
    if(g_car_track.confirmed != 0U)
    {
        component_t tracked_lamp;
        if((component_from_temporal_car(&g_car_track, &tracked_lamp, 1U) != 0) &&
           (car_lamp_below_horizon(&tracked_lamp) == 0U))
        {
            reset_track(&g_car_track);
        }
    }
    if((g_car_lamp_mode == CAR_LAMP_MODE_DUAL) &&
       (g_car_track_secondary.confirmed != 0U))
    {
        component_t tracked_lamp;

        if((g_car_track_secondary.y + (float)BEACON_IMAGE_H * 0.5f +
            g_car_track_secondary.vy < (float)CAR_LAMP_MIN_CENTER_Y) ||
           ((component_from_temporal_car(
                 &g_car_track_secondary, &tracked_lamp, 1U) != 0U) &&
            (car_lamp_below_horizon(&tracked_lamp) == 0U)))
        {
            reset_track(&g_car_track_secondary);
        }
    }

    memset(&temporal_lamp, 0, sizeof(temporal_lamp));
    memset(&secondary_temporal_lamp, 0, sizeof(secondary_temporal_lamp));
    step_start_us = timer_get(TC_TIME2_CH0);
    scene_sum = threshold_car_lamp_image(image);
    scene_mean = (float)scene_sum /
                 (float)(BEACON_IMAGE_W * BEACON_IMAGE_H);
    g_image_profile.lamp_threshold_us =
        timer_get(TC_TIME2_CH0) - step_start_us;

    step_start_us = timer_get(TC_TIME2_CH0);
    has_lamp = find_car_lamp(image, &lamp);
    g_image_profile.lamp_primary_us = timer_get(TC_TIME2_CH0) - step_start_us;
    if(has_lamp == 0U)
    {
        step_start_us = timer_get(TC_TIME2_CH0);
        if(scene_mean < CAR_LAMP_GRAY_SCENE_MAX)
        {
            reusable_binary_threshold = CAR_LAMP_GRAY_THRESHOLD;
        }
        has_lamp = find_gray_car_lamp(image, scene_mean, &lamp);
        g_image_profile.lamp_gray_us = timer_get(TC_TIME2_CH0) - step_start_us;
    }
    if(has_lamp == 0)
    {
        step_start_us = timer_get(TC_TIME2_CH0);
        if(can_use_temporal_car_mask(&g_car_track) != 0)
        {
            reusable_binary_threshold = 0U;
        }
        has_lamp = find_temporal_car_lamp(image, &lamp);
        g_image_profile.lamp_temporal_us =
            timer_get(TC_TIME2_CH0) - step_start_us;
    }
    if(has_lamp == 0)
    {
        memset(&lamp, 0, sizeof(lamp));
    }
    else
    {
        lamps[0] = lamp;
        lamp_count = 1U;
    }
    if((g_car_lamp_mode == CAR_LAMP_MODE_DUAL) &&
       (lamp_count != 0U))
    {
        has_secondary_lamp = find_compatible_car_lamp(
            image, &lamps[0], &secondary_lamp);
        if((has_secondary_lamp == 0U) &&
           (scene_mean < CAR_LAMP_GRAY_SCENE_MAX))
        {
            reusable_binary_threshold = CAR_LAMP_GRAY_THRESHOLD;
            has_secondary_lamp = find_compatible_gray_car_lamp(
                image, scene_mean, &lamps[0], &secondary_lamp);
        }
        if((has_secondary_lamp == 0U) &&
           (can_use_temporal_car_mask(&g_car_track_secondary) != 0U))
        {
            reusable_binary_threshold = 0U;
            has_secondary_lamp = find_temporal_car_lamp_for_track(
                image, &g_car_track_secondary, &secondary_lamp);
            if((has_secondary_lamp != 0U) &&
               (car_lamp_pair_compatible(
                    &lamps[0], &secondary_lamp) == 0U))
            {
                memset(&secondary_lamp, 0, sizeof(secondary_lamp));
                has_secondary_lamp = 0U;
            }
        }
        if(has_secondary_lamp != 0U)
        {
            lamps[lamp_count++] = secondary_lamp;
            g_secondary_lamp_mask = secondary_lamp;
        }
    }
    if(can_use_temporal_car_mask(&g_car_track) != 0)
    {
        (void)component_from_temporal_car(&g_car_track, &temporal_lamp, 1U);
    }
    if((g_car_lamp_mode == CAR_LAMP_MODE_DUAL) &&
       (can_use_temporal_car_mask(&g_car_track_secondary) != 0U))
    {
        (void)component_from_temporal_car(
            &g_car_track_secondary, &secondary_temporal_lamp, 1U);
        g_secondary_temporal_lamp_mask = secondary_temporal_lamp;
    }
    if(g_car_lamp_mode == CAR_LAMP_MODE_DUAL)
    {
        /* 车体连接区只使用本帧真实测量，单灯或预测补位时保持原遮罩。 */
        car_body_mask_build(&lamp, &secondary_lamp);
    }
    if(g_car_lamp_mode == CAR_LAMP_MODE_DUAL)
    {
        write_car_lamps(lamps, lamp_count, result);
    }
    else
    {
        write_car_lamp(&lamp, result);
    }
    find_beacons(image, &lamp, &temporal_lamp, scene_mean,
                 reusable_binary_threshold, result);

    step_start_us = timer_get(TC_TIME2_CH0);
    if(g_car_lamp_mode == CAR_LAMP_MODE_DUAL)
    {
        update_temporal_cars_dual(lamps, lamp_count, result);
    }
    else
    {
        update_temporal_car(result);
    }
    beacon_output_finalize(image, result);
    g_image_profile.post_us = timer_get(TC_TIME2_CH0) - step_start_us;
}

static uint8 s_image_frame[MT9V03X_H][MT9V03X_W];
static uint8 s_mt9v03x_initialized;

struct image_data g_image_data;

static uint8 image_latch_frame(void)
{
    if(0U == mt9v03x_finish_flag)
    {
        return 0U;
    }

    memcpy(s_image_frame[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    mt9v03x_finish_flag = 0U;
    return 1U;
}

static void image_clear_results(void)
{
    image_data_clear(&g_image_data);
}

static void image_store_result(const beacon_result_t *result)
{
    uint8 i;
    uint8 beacon_count = result->beacon_count;
    uint8 car_lamp_count = result->car_lamp_count;

    image_clear_results();

    if(beacon_count > IMAGE_MAX_BEACON_COUNT)
    {
        beacon_count = IMAGE_MAX_BEACON_COUNT;
    }
    for(i = 0U; i < beacon_count; i++)
    {
        g_image_data.beacon_data[i].valid = result->beacons[i].valid;
        g_image_data.beacon_data[i].x = result->beacons[i].x;
        g_image_data.beacon_data[i].y = result->beacons[i].y;
        g_image_data.beacon_data[i].area = result->beacons[i].area;
    }

    if(car_lamp_count > IMAGE_MAX_CAR_LAMP_COUNT)
    {
        car_lamp_count = IMAGE_MAX_CAR_LAMP_COUNT;
    }
    for(i = 0U; i < car_lamp_count; i++)
    {
        g_image_data.car_lamp_data[i].valid = result->car_lamps[i].valid;
        g_image_data.car_lamp_data[i].cx = result->car_lamps[i].cx;
        g_image_data.car_lamp_data[i].cy = result->car_lamps[i].cy;
        g_image_data.car_lamp_data[i].width = result->car_lamps[i].width;
        g_image_data.car_lamp_data[i].length = result->car_lamps[i].length;
        g_image_data.car_lamp_data[i].angle = result->car_lamps[i].angle;
    }
    g_image_data.car_lamp_measured_mask =
        result->car_lamp_measured_mask &
        (unsigned char)((1U << car_lamp_count) - 1U);
}

void image_init(void)
{
    memset(s_image_frame, 0, sizeof(s_image_frame));
    image_clear_results();
    beacon_image_init();
    g_car_lamp_mode = CAR_LAMP_MODE_SINGLE;
    beacon_image_reset_temporal();

    mt9v03x_finish_flag = 0U;
    s_mt9v03x_initialized = (0U == mt9v03x_init()) ? 1U : 0U;
}

void image_set_car_lamp_mode(uint8 mode)
{
    unsigned char normalized =
        (mode == CAR_LAMP_MODE_DUAL) ?
        CAR_LAMP_MODE_DUAL : CAR_LAMP_MODE_SINGLE;

    if(g_car_lamp_mode != normalized)
    {
        g_car_lamp_mode = normalized;
        beacon_image_reset_temporal();
        image_clear_results();
    }
}

void image_update(void)
{
    beacon_result_t result;
    uint32 step_start_us;

    memset(&g_image_profile, 0, sizeof(g_image_profile));

    step_start_us = timer_get(TC_TIME2_CH0);
    if(0U == image_latch_frame())
    {
        return;
    }
    g_image_profile.latch_us = timer_get(TC_TIME2_CH0) - step_start_us;

    beacon_image_process(s_image_frame, &result);
    image_store_result(&result);
    g_image_profile.stage_sum_us =
        g_image_profile.latch_us +
        g_image_profile.scene_mean_us +
        g_image_profile.lamp_threshold_us +
        g_image_profile.lamp_primary_us +
        g_image_profile.lamp_gray_us +
        g_image_profile.lamp_temporal_us +
        g_image_profile.beacon_box_us +
        g_image_profile.beacon_peak_us +
        g_image_profile.beacon_component_us +
        g_image_profile.beacon_store_us +
        g_image_profile.post_us;
}

uint8 *image_get_frame_buffer(void)
{
    return s_image_frame[0];
}

static void image_stream_set_pixel(int x, int y, uint8 value)
{
    if((x >= 0) && (x < MT9V03X_W) && (y >= 0) && (y < MT9V03X_H))
    {
        s_image_frame[y][x] = value;
    }
}

static void image_stream_draw_overlay(void)
{
    uint8 index;

    for(index = 0U; index < IMAGE_MAX_BEACON_COUNT; index++)
    {
        int x;
        int y;
        int offset;

        if(g_image_data.beacon_data[index].valid == 0U)
        {
            continue;
        }

        x = (int)(g_image_data.beacon_data[index].x +
                  ((float)MT9V03X_W * 0.5f) + 0.5f);
        y = (int)(g_image_data.beacon_data[index].y +
                  ((float)MT9V03X_H * 0.5f) + 0.5f);
        for(offset = -4; offset <= 4; offset++)
        {
            image_stream_set_pixel(x + offset, y, 255U);
            image_stream_set_pixel(x, y + offset, 255U);
        }
    }

    for(index = 0U; index < IMAGE_MAX_CAR_LAMP_COUNT; index++)
    {
        int x;
        int y;
        int half_width;
        int half_height;
        int offset;

        if(g_image_data.car_lamp_data[index].valid == 0U)
        {
            continue;
        }

        x = (int)(g_image_data.car_lamp_data[index].cx +
                  ((float)MT9V03X_W * 0.5f) + 0.5f);
        y = (int)(g_image_data.car_lamp_data[index].cy +
                  ((float)MT9V03X_H * 0.5f) + 0.5f);
        half_width = (int)(g_image_data.car_lamp_data[index].width * 0.5f + 1.0f);
        half_height = (int)(g_image_data.car_lamp_data[index].length * 0.5f + 1.0f);
        for(offset = -half_width; offset <= half_width; offset++)
        {
            image_stream_set_pixel(x + offset, y - half_height, 255U);
            image_stream_set_pixel(x + offset, y + half_height, 255U);
        }
        for(offset = -half_height; offset <= half_height; offset++)
        {
            image_stream_set_pixel(x - half_width, y + offset, 255U);
            image_stream_set_pixel(x + half_width, y + offset, 255U);
        }
    }
}

uint8 *image_prepare_stream_frame(void)
{
    if(stream_mode == IMAGE_STREAM_MODE_LAMP_BINARY)
    {
        (void)threshold_car_lamp_image(s_image_frame);
        memcpy(s_image_frame, g_binary, sizeof(s_image_frame));
    }
    else if(stream_mode == IMAGE_STREAM_MODE_BEACON_BINARY)
    {
        threshold_beacon_image(s_image_frame);
        memcpy(s_image_frame, g_binary, sizeof(s_image_frame));
    }
    else if(stream_mode == IMAGE_STREAM_MODE_DETECTED_OVERLAY)
    {
        image_stream_draw_overlay();
    }

    return s_image_frame[0];
}

void image_algorithm_params_changed(void)
{
    beacon_image_reset_temporal();
}

uint8 image_camera_param_get_exposure(int32 *actual_value)
{
    if(actual_value == NULL)
    {
        return IMAGE_PARAM_STATUS_ERROR;
    }

    *actual_value = (int32)g_mt9v03x_exp_time;
    return (s_mt9v03x_initialized != 0U) ?
           IMAGE_PARAM_STATUS_OK : IMAGE_PARAM_STATUS_ERROR;
}

uint8 image_camera_param_set_exposure(int32 value, int32 *actual_value)
{
    uint16 previous_exp_time;
    uint8 init_status;

    if(actual_value == NULL)
    {
        return IMAGE_PARAM_STATUS_ERROR;
    }
    if((value < IMAGE_EXP_TIME_MIN) || (value > IMAGE_EXP_TIME_MAX))
    {
        *actual_value = (int32)g_mt9v03x_exp_time;
        return IMAGE_PARAM_STATUS_OUT_OF_RANGE;
    }
    if(((int32)g_mt9v03x_exp_time == value) &&
       (s_mt9v03x_initialized != 0U))
    {
        *actual_value = value;
        return IMAGE_PARAM_STATUS_OK;
    }

    previous_exp_time = g_mt9v03x_exp_time;
    g_mt9v03x_exp_time = (uint16)value;
    mt9v03x_finish_flag = 0U;
    s_mt9v03x_initialized = 0U;
    init_status = mt9v03x_init();
    if((init_status == 0U) && ((int32)g_mt9v03x_exp_time == value))
    {
        s_mt9v03x_initialized = 1U;
        *actual_value = (int32)g_mt9v03x_exp_time;
        return IMAGE_PARAM_STATUS_OK;
    }

    g_mt9v03x_exp_time = previous_exp_time;
    mt9v03x_finish_flag = 0U;
    init_status = mt9v03x_init();
    *actual_value = (int32)g_mt9v03x_exp_time;
    if((init_status != 0U) || (g_mt9v03x_exp_time != previous_exp_time))
    {
        return IMAGE_PARAM_STATUS_ROLLBACK_FAIL;
    }

    s_mt9v03x_initialized = 1U;
    return IMAGE_PARAM_STATUS_ERROR;
}
