#include "image.h"

#include <math.h>
#include <string.h>

#include "zf_device_mt9v03x.h"
#include "image_horizon.h"

#ifndef IMAGE_USE_CM4_DSP
#if defined(__ICCARM__) || defined(__ARM_FEATURE_DSP)
#define IMAGE_USE_CM4_DSP 1
#else
#define IMAGE_USE_CM4_DSP 0
#endif
#endif

#define BEACON_IMAGE_W 188
#define BEACON_IMAGE_H 120
#define BEACON_MAX_BEACON_COUNT 3
#define BEACON_MAX_CAR_LAMP_COUNT 1

typedef struct
{
    beacon_circle_t beacons[BEACON_MAX_BEACON_COUNT];
    unsigned char beacon_count;
    beacon_rect_t car_lamps[BEACON_MAX_CAR_LAMP_COUNT];
    unsigned char car_lamp_count;
} beacon_result_t;

#if (MT9V03X_W != BEACON_IMAGE_W) || (MT9V03X_H != BEACON_IMAGE_H)
#error "Beacon image algorithm is tuned for MT9V03X 188x120 frames."
#endif

static float beacon_area(const beacon_circle_t *beacon);
static float gray_image_mean(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W]);
static unsigned int image_count_trailing_zeros(unsigned int value);
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
#define IMAGE_SPAN_QUEUE_SIZE     ((IMAGE_QUEUE_SIZE + 1) / 2)
#define IMAGE_BINARY_FOREGROUND   255U
#define IMAGE_BINARY_VISITED      1U
#define IMAGE_BINARY_WORD_COUNT   ((BEACON_IMAGE_W + 31) / 32)
#define GRAY_SPARSE_CACHE_ROWS     4
#define PI_F                      3.1415926f

#define GRAY_BEACON_MAX_PEAKS          16
#define GRAY_BEACON_MAX_CANDIDATES     24
#define GRAY_BEACON_PATCH_RADIUS       7
#define GRAY_BEACON_BOX_PAD            4
#define GRAY_BEACON_PADDED_W           \
    (BEACON_IMAGE_W + GRAY_BEACON_BOX_PAD * 2)
#define GRAY_BEACON_PADDED_H           \
    (BEACON_IMAGE_H + GRAY_BEACON_BOX_PAD * 2)
#define GRAY_INTEGRAL_STRIDE            (GRAY_BEACON_PADDED_W + 2)
#ifndef GRAY_BEACON_SPARSE_MAX_BRIGHT_PIXELS
#define GRAY_BEACON_SPARSE_MAX_BRIGHT_PIXELS \
    ((BEACON_IMAGE_W * BEACON_IMAGE_H * 3) / 5)
#endif
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
    float angle_x;
    float angle_y;
    float direction_cos;
    float direction_sin;
    int32 moment_xx;
    int32 moment_yy;
    int32 moment_xy;
    unsigned char valid;
    unsigned char shape_valid;
    unsigned char angle_valid;
    unsigned char direction_valid;
} component_t;

static unsigned char gray_car_lamp_valid(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    component_t *comp);
static unsigned char gray_point_in_lamp(
    int x,
    int y,
    const component_t *lamp);
static unsigned char gray_point_in_lamp_with_trig(
    int x,
    int y,
    const component_t *lamp,
    float cos_a,
    float sin_a);
static unsigned char gray_point_in_lamp_shadow(
    int x,
    int y,
    const component_t *lamp);
static unsigned int image_count_trailing_zeros(unsigned int value);

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
    float direction_cos;
    float direction_sin;
    float area;
    unsigned char direction_valid;
} temporal_track_t;

typedef struct
{
    signed short response;
    unsigned char x;
    unsigned char y;
} gray_beacon_peak_t;

typedef struct
{
    float x;
    float y;
    float area;
} gray_beacon_candidate_t;

typedef struct
{
    float background;
    float inner_contrast;
    float radial_drop;
    float concentration;
    float elongation_sq;
    float offset_sq;
    float centroid_x;
    float centroid_y;
    unsigned char peak;
    unsigned char half_area;
} gray_beacon_features_t;

typedef union
{
    struct
    {
        unsigned char binary[BEACON_IMAGE_H][BEACON_IMAGE_W];
        unsigned int span_queue[IMAGE_SPAN_QUEUE_SIZE];
    } components;
    struct
    {
        unsigned int source_words[BEACON_IMAGE_H]
                                 [IMAGE_BINARY_WORD_COUNT];
        unsigned char reserved[
            BEACON_IMAGE_H * BEACON_IMAGE_W -
            BEACON_IMAGE_H * IMAGE_BINARY_WORD_COUNT *
            (int)sizeof(unsigned int)];
        unsigned int span_queue[IMAGE_SPAN_QUEUE_SIZE];
    } bit_components;
    unsigned short integral[GRAY_BEACON_PADDED_H + 1]
                           [GRAY_INTEGRAL_STRIDE];
} image_scratch_t;

static image_scratch_t g_image_scratch;
#define g_binary  (g_image_scratch.components.binary)
#define g_span_queue (g_image_scratch.components.span_queue)
#define g_component_temporary_words \
    (g_image_scratch.bit_components.source_words)
#define g_component_source_words g_component_temporary_words
#if IMAGE_USE_CM4_DSP
/*
 * CM4预处理积分图必须跨越车灯连通域阶段存活，不能再与连通域工作区
 * 复用。每行增加一个未使用半字，使相邻行保持32位对齐。
 */
static unsigned short g_preprocessed_integral[GRAY_BEACON_PADDED_H + 1]
                                              [GRAY_INTEGRAL_STRIDE];
#define g_gray_integral g_preprocessed_integral
#else
#define g_gray_integral (g_image_scratch.integral)
#endif
static temporal_track_t g_b0_track;
static temporal_track_t g_car_track;
static unsigned char g_track_reinforced;
#if IMAGE_USE_CM4_DSP
static unsigned int g_gray_vertical39_sum[
    BEACON_IMAGE_W + GRAY_BEACON_BOX_PAD * 2];
#else
static unsigned short g_gray_vertical3_sum[
    BEACON_IMAGE_W + GRAY_BEACON_BOX_PAD * 2];
static unsigned short g_gray_vertical9_sum[
    BEACON_IMAGE_W + GRAY_BEACON_BOX_PAD * 2];
#endif
static signed short g_gray_response_rows[3][BEACON_IMAGE_W];
typedef union
{
    signed short sparse_response[GRAY_SPARSE_CACHE_ROWS][BEACON_IMAGE_W];
    unsigned int foreground_words[BEACON_IMAGE_H][IMAGE_BINARY_WORD_COUNT];
} gray_stage_scratch_t;

static gray_stage_scratch_t g_gray_stage_scratch;
#define g_gray_sparse_response (g_gray_stage_scratch.sparse_response)
#define g_component_remaining_words (g_gray_stage_scratch.foreground_words)
#define g_binary_foreground_words g_component_remaining_words
static unsigned int g_preprocessed_car_words[BEACON_IMAGE_H]
                                                 [IMAGE_BINARY_WORD_COUNT];
#if IMAGE_USE_CM4_DSP
static unsigned int g_preprocessed_bright_words[BEACON_IMAGE_H]
                                                    [IMAGE_BINARY_WORD_COUNT];
static unsigned int g_preprocessed_beacon_dark_words[BEACON_IMAGE_H]
                                                         [IMAGE_BINARY_WORD_COUNT];
static unsigned int g_preprocessed_beacon_bright_words[BEACON_IMAGE_H]
                                                           [IMAGE_BINARY_WORD_COUNT];
#else
static unsigned int g_preprocessed_beacon_words[BEACON_IMAGE_H]
                                                    [IMAGE_BINARY_WORD_COUNT];
#endif
static unsigned int g_gray_sparse_response_valid[GRAY_SPARSE_CACHE_ROWS]
                                                [IMAGE_BINARY_WORD_COUNT];
static signed short g_gray_sparse_response_row[GRAY_SPARSE_CACHE_ROWS];

#if defined(IMAGE_HOST_PROFILE)
static unsigned int g_profile_component_count;
static unsigned int g_profile_shape_count;
static unsigned int g_profile_angle_count;
static unsigned int g_profile_direction_count;

void image_host_component_profile_reset(void)
{
    g_profile_component_count = 0U;
    g_profile_shape_count = 0U;
    g_profile_angle_count = 0U;
    g_profile_direction_count = 0U;
}

void image_host_component_profile_read(
    unsigned int *component_count,
    unsigned int *shape_count,
    unsigned int *angle_count,
    unsigned int *direction_count)
{
    if(component_count != 0) *component_count = g_profile_component_count;
    if(shape_count != 0) *shape_count = g_profile_shape_count;
    if(angle_count != 0) *angle_count = g_profile_angle_count;
    if(direction_count != 0) *direction_count = g_profile_direction_count;
}
#endif

int32 g_beacon_binary_threshold = IMAGE_BEACON_BINARY_THRESHOLD_DEFAULT;

static void beacon_image_init(void)
{
    memset(g_binary, 0, sizeof(g_binary));
}

static void beacon_image_reset_temporal(void)
{
    memset(&g_b0_track, 0, sizeof(g_b0_track));
    memset(&g_car_track, 0, sizeof(g_car_track));
}

static void clear_result(beacon_result_t *result)
{
    if(result != 0)
    {
        memset(result, 0, sizeof(*result));
    }
}

#if IMAGE_USE_CM4_DSP
static void threshold_row_dsp(
    const unsigned char *source,
    unsigned char *destination,
    int count,
    unsigned char threshold)
{
    unsigned int threshold4 = (unsigned int)threshold * 0x01010101U;

    while((count > 0) &&
          ((((uintptr_t)source | (uintptr_t)destination) & 3U) != 0U))
    {
        *destination++ = (*source++ >= threshold) ? 255U : 0U;
        count--;
    }
    while(count >= 4)
    {
        unsigned int pixels = *(const unsigned int *)source;

        (void)__USUB8(pixels, threshold4);
        *(unsigned int *)destination = __SEL(0xFFFFFFFFU, 0U);
        source += 4;
        destination += 4;
        count -= 4;
    }
    while(count > 0)
    {
        *destination++ = (*source++ >= threshold) ? 255U : 0U;
        count--;
    }
}
#endif

static void threshold_row_with_foreground_bits(
    const unsigned char *source,
    unsigned char *destination,
    unsigned int foreground_words[IMAGE_BINARY_WORD_COUNT],
    unsigned char threshold)
{
    int x;

    memset(foreground_words, 0,
           sizeof(unsigned int) * IMAGE_BINARY_WORD_COUNT);
#if IMAGE_USE_CM4_DSP
    {
        unsigned int threshold4 =
            (unsigned int)threshold * 0x01010101U;

        for(x = 0; x < BEACON_IMAGE_W; x += 4)
        {
            unsigned int pixels = *(const unsigned int *)&source[x];
            unsigned int selected;
            unsigned int bits;

            (void)__USUB8(pixels, threshold4);
            selected = __SEL(0xFFFFFFFFU, 0U);
            *(unsigned int *)&destination[x] = selected;
            bits = ((selected & 0x01010101U) * 0x01020408U) >> 24;
            foreground_words[x >> 5] |= bits << (x & 31);
        }
    }
#else
    for(x = 0; x < BEACON_IMAGE_W; x++)
    {
        unsigned char foreground =
            (source[x] >= threshold) ? IMAGE_BINARY_FOREGROUND : 0U;

        destination[x] = foreground;
        if(foreground != 0U)
        {
            foreground_words[x >> 5] |= 1UL << (x & 31);
        }
    }
#endif
}

static unsigned char component_word_pixel(
    const unsigned int words[BEACON_IMAGE_H][IMAGE_BINARY_WORD_COUNT],
    int x,
    int y)
{
    return ((words[y][x >> 5] & (1UL << (x & 31))) != 0U) ? 1U : 0U;
}

static unsigned char component_source_pixel(int x, int y)
{
    return component_word_pixel(g_component_source_words, x, y);
}

static unsigned char component_remaining_pixel(int x, int y)
{
    return component_word_pixel(g_component_remaining_words, x, y);
}

static void component_use_words(
    const unsigned int words[BEACON_IMAGE_H][IMAGE_BINARY_WORD_COUNT])
{
    memcpy(g_component_source_words, words,
           sizeof(g_component_source_words));
    memcpy(g_component_remaining_words, g_component_source_words,
           sizeof(g_component_remaining_words));
}

static void component_set_pixel(int x, int y)
{
    unsigned int bit = 1UL << (x & 31);

    g_component_source_words[y][x >> 5] |= bit;
    g_component_remaining_words[y][x >> 5] |= bit;
}

static void component_clear_pixel(int x, int y)
{
    unsigned int mask = ~(1UL << (x & 31));

    g_component_source_words[y][x >> 5] &= mask;
    g_component_remaining_words[y][x >> 5] &= mask;
}

static void component_clear_remaining_range(int y, int left, int right)
{
    int first_word = left >> 5;
    int last_word = right >> 5;
    unsigned int first_mask = 0xFFFFFFFFUL << (left & 31);
    unsigned int last_mask = 0xFFFFFFFFUL >> (31 - (right & 31));

    if(first_word == last_word)
    {
        g_component_remaining_words[y][first_word] &=
            ~(first_mask & last_mask);
        return;
    }

    g_component_remaining_words[y][first_word] &= ~first_mask;
    first_word++;
    while(first_word < last_word)
    {
        g_component_remaining_words[y][first_word++] = 0U;
    }
    g_component_remaining_words[y][last_word] &= ~last_mask;
}

static int component_next_remaining_x(int y, int left, int right)
{
    int word_index = left >> 5;
    int last_word = right >> 5;
    unsigned int first_mask = 0xFFFFFFFFUL << (left & 31);
    unsigned int last_mask = 0xFFFFFFFFUL >> (31 - (right & 31));

    while(word_index <= last_word)
    {
        unsigned int bits = g_component_remaining_words[y][word_index];

        if(word_index == (left >> 5))
        {
            bits &= first_mask;
        }
        if(word_index == last_word)
        {
            bits &= last_mask;
        }
        if(bits != 0U)
        {
            return word_index * 32 +
                   (int)image_count_trailing_zeros(bits);
        }
        word_index++;
    }
    return -1;
}

static void threshold_row_to_component_words(
    const unsigned char *source,
    unsigned int source_words[IMAGE_BINARY_WORD_COUNT],
    unsigned int remaining_words[IMAGE_BINARY_WORD_COUNT],
    unsigned char threshold)
{
    int x;

    memset(source_words, 0,
           sizeof(unsigned int) * IMAGE_BINARY_WORD_COUNT);
#if IMAGE_USE_CM4_DSP
    {
        unsigned int threshold4 =
            (unsigned int)threshold * 0x01010101U;

        for(x = 0; x < BEACON_IMAGE_W; x += 4)
        {
            unsigned int pixels = *(const unsigned int *)&source[x];
            unsigned int selected;
            unsigned int bits;

            (void)__USUB8(pixels, threshold4);
            selected = __SEL(0x01010101U, 0U);
            bits = (selected * 0x01020408U) >> 24;
            source_words[x >> 5] |= bits << (x & 31);
        }
    }
#else
    for(x = 0; x < BEACON_IMAGE_W; x++)
    {
        if(source[x] >= threshold)
        {
            source_words[x >> 5] |= 1UL << (x & 31);
        }
    }
#endif
    memcpy(remaining_words, source_words,
           sizeof(unsigned int) * IMAGE_BINARY_WORD_COUNT);
}

static void threshold_component_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char threshold)
{
    int y;

    for(y = 0; y < BEACON_IMAGE_H; y++)
    {
        threshold_row_to_component_words(
            image[y], g_component_source_words[y],
            g_component_remaining_words[y], threshold);
    }
}

static unsigned char gray_component_configuration(
    float scene_mean,
    unsigned char *threshold,
    int *minimum_area,
    int *maximum_area)
{
    if(scene_mean < GRAY_BEACON_DARK_SCENE_MEAN)
    {
        if(threshold != 0) *threshold = 80U;
        if(minimum_area != 0) *minimum_area = 1;
        if(maximum_area != 0) *maximum_area = 500;
        return 1U;
    }
    if(scene_mean >= GRAY_BEACON_BRIGHT_SCENE_MEAN)
    {
        if(threshold != 0) *threshold = 120U;
        if(minimum_area != 0) *minimum_area = 30;
        if(maximum_area != 0) *maximum_area = 120;
        return 1U;
    }
    return 0U;
}

#if !IMAGE_USE_CM4_DSP
static void preprocess_detection_words(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    float scene_mean,
    unsigned short *bright_pixel_count)
{
    unsigned int bright = 0U;
    unsigned char beacon_threshold = 0U;
    int unused_minimum_area;
    int unused_maximum_area;
    unsigned char has_beacon_components = gray_component_configuration(
        scene_mean, &beacon_threshold,
        &unused_minimum_area, &unused_maximum_area);
    int y;

    for(y = 0; y < BEACON_IMAGE_H; y++)
    {
        const unsigned char *source = image[y];
        unsigned int *car_words = g_preprocessed_car_words[y];
        unsigned int *beacon_words = g_preprocessed_beacon_words[y];
        unsigned char car_threshold = CAR_LAMP_BINARY_THRESHOLD;
        int x;

        if(y < CAR_LAMP_UPPER_Y)
        {
            car_threshold = CAR_LAMP_UPPER_THRESHOLD;
        }
        memset(car_words, 0,
               sizeof(unsigned int) * IMAGE_BINARY_WORD_COUNT);
        memset(beacon_words, 0,
               sizeof(unsigned int) * IMAGE_BINARY_WORD_COUNT);
#if IMAGE_USE_CM4_DSP
        {
            unsigned int bright_threshold4 =
                (unsigned int)GRAY_BEACON_MIN_PEAK_GRAY * 0x01010101U;
            unsigned int car_threshold4 =
                (unsigned int)car_threshold * 0x01010101U;
            unsigned int beacon_threshold4 =
                (unsigned int)beacon_threshold * 0x01010101U;

            for(x = 0; x < BEACON_IMAGE_W; x += 4)
            {
                unsigned int pixels = *(const unsigned int *)&source[x];
                unsigned int selected;
                unsigned int bits;

                (void)__USUB8(pixels, bright_threshold4);
                selected = __SEL(0x01010101U, 0U);
                bright = __USADA8(selected, 0U, bright);

                (void)__USUB8(pixels, car_threshold4);
                selected = __SEL(0x01010101U, 0U);
                bits = (selected * 0x01020408U) >> 24;
                car_words[x >> 5] |= bits << (x & 31);

                if(has_beacon_components != 0U)
                {
                    (void)__USUB8(pixels, beacon_threshold4);
                    selected = __SEL(0x01010101U, 0U);
                    bits = (selected * 0x01020408U) >> 24;
                    beacon_words[x >> 5] |= bits << (x & 31);
                }
            }
        }
#else
        for(x = 0; x < BEACON_IMAGE_W; x++)
        {
            unsigned char gray = source[x];

            if(gray >= GRAY_BEACON_MIN_PEAK_GRAY)
            {
                bright++;
            }
            if(gray >= car_threshold)
            {
                car_words[x >> 5] |= 1UL << (x & 31);
            }
            if((has_beacon_components != 0U) &&
               (gray >= beacon_threshold))
            {
                beacon_words[x >> 5] |= 1UL << (x & 31);
            }
        }
#endif
    }

    if(bright_pixel_count != 0)
    {
        *bright_pixel_count = (unsigned short)bright;
    }
}
#endif

#if IMAGE_USE_CM4_DSP
static inline void preprocess_integral_write_pair(
    const unsigned short *previous,
    unsigned short *current,
    int output_x,
    unsigned int *row_sum,
    unsigned char first,
    unsigned char second)
{
    unsigned int first_sum = *row_sum + first;
    unsigned int second_sum = first_sum + second;
    unsigned int packed_prefix =
        (first_sum & 0xFFFFU) | (second_sum << 16);
    unsigned int previous_pair =
        *(const unsigned int *)&previous[output_x];

    *(unsigned int *)&current[output_x] =
        __SADD16(previous_pair, packed_prefix);
    *row_sum = second_sum;
}

static void preprocess_padded_integral_row(
    const unsigned short *previous,
    unsigned short *current,
    const unsigned char *source)
{
    unsigned int row_sum = 0U;
    int x;

    current[0] = 0U;
    preprocess_integral_write_pair(
        previous, current, 1, &row_sum, source[4], source[3]);
    preprocess_integral_write_pair(
        previous, current, 3, &row_sum, source[2], source[1]);
    for(x = 0; x < BEACON_IMAGE_W; x += 4)
    {
        unsigned int pixels = *(const unsigned int *)&source[x];

        preprocess_integral_write_pair(
            previous, current, x + 5, &row_sum,
            (unsigned char)(pixels & 0xFFU),
            (unsigned char)((pixels >> 8) & 0xFFU));
        preprocess_integral_write_pair(
            previous, current, x + 7, &row_sum,
            (unsigned char)((pixels >> 16) & 0xFFU),
            (unsigned char)(pixels >> 24));
    }
    preprocess_integral_write_pair(
        previous, current, BEACON_IMAGE_W + 5, &row_sum,
        source[BEACON_IMAGE_W - 2], source[BEACON_IMAGE_W - 3]);
    preprocess_integral_write_pair(
        previous, current, BEACON_IMAGE_W + 7, &row_sum,
        source[BEACON_IMAGE_W - 4], source[BEACON_IMAGE_W - 5]);
}

static float preprocess_detection_words_dsp(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned short *bright_pixel_count)
{
    const unsigned int bright_threshold4 =
        (unsigned int)GRAY_BEACON_MIN_PEAK_GRAY * 0x01010101U;
    const unsigned int beacon_dark_threshold4 = 80U * 0x01010101U;
    const unsigned int beacon_bright_threshold4 = 120U * 0x01010101U;
    unsigned int sum = 0U;
    unsigned int bright = 0U;
    int y;

    memset(g_gray_integral[0], 0, sizeof(g_gray_integral[0]));
    for(y = 0; y < GRAY_BEACON_BOX_PAD; y++)
    {
        preprocess_padded_integral_row(
            g_gray_integral[y], g_gray_integral[y + 1],
            image[GRAY_BEACON_BOX_PAD - y]);
    }
    for(y = 0; y < BEACON_IMAGE_H; y++)
    {
        const unsigned char *source = image[y];
        const unsigned short *integral_previous =
            g_gray_integral[y + GRAY_BEACON_BOX_PAD];
        unsigned short *integral_current =
            g_gray_integral[y + GRAY_BEACON_BOX_PAD + 1];
        unsigned int *car_words = g_preprocessed_car_words[y];
        unsigned int *bright_words = g_preprocessed_bright_words[y];
        unsigned int *beacon_dark_words =
            g_preprocessed_beacon_dark_words[y];
        unsigned int *beacon_bright_words =
            g_preprocessed_beacon_bright_words[y];
        unsigned char car_threshold = CAR_LAMP_BINARY_THRESHOLD;
        unsigned int car_threshold4;
        unsigned int car_bits = 0U;
        unsigned int bright_bits = 0U;
        unsigned int beacon_dark_bits = 0U;
        unsigned int beacon_bright_bits = 0U;
        unsigned int integral_row_sum = 0U;
        unsigned char build_integral =
            (bright <= GRAY_BEACON_SPARSE_MAX_BRIGHT_PIXELS) ? 1U : 0U;
        int x;

        if(y < CAR_LAMP_UPPER_Y)
        {
            car_threshold = CAR_LAMP_UPPER_THRESHOLD;
        }
        car_threshold4 = (unsigned int)car_threshold * 0x01010101U;
        if(build_integral != 0U)
        {
            integral_current[0] = 0U;
            preprocess_integral_write_pair(
                integral_previous, integral_current, 1,
                &integral_row_sum, source[4], source[3]);
            preprocess_integral_write_pair(
                integral_previous, integral_current, 3,
                &integral_row_sum, source[2], source[1]);
        }

        for(x = 0; x < BEACON_IMAGE_W; x += 4)
        {
            unsigned int pixels = *(const unsigned int *)&source[x];
            unsigned int selected;
            unsigned int bits;
            unsigned int shift = (unsigned int)x & 31U;

            sum = __USADA8(pixels, 0U, sum);

            if(build_integral != 0U)
            {
                preprocess_integral_write_pair(
                    integral_previous, integral_current, x + 5,
                    &integral_row_sum,
                    (unsigned char)(pixels & 0xFFU),
                    (unsigned char)((pixels >> 8) & 0xFFU));
                preprocess_integral_write_pair(
                    integral_previous, integral_current, x + 7,
                    &integral_row_sum,
                    (unsigned char)((pixels >> 16) & 0xFFU),
                    (unsigned char)(pixels >> 24));
            }

            (void)__USUB8(pixels, bright_threshold4);
            selected = __SEL(0x01010101U, 0U);
            bright = __USADA8(selected, 0U, bright);
            bits = (selected * 0x01020408U) >> 24;
            bright_bits |= bits << shift;

            (void)__USUB8(pixels, car_threshold4);
            selected = __SEL(0x01010101U, 0U);
            bits = (selected * 0x01020408U) >> 24;
            car_bits |= bits << shift;

            (void)__USUB8(pixels, beacon_dark_threshold4);
            selected = __SEL(0x01010101U, 0U);
            bits = (selected * 0x01020408U) >> 24;
            beacon_dark_bits |= bits << shift;

            (void)__USUB8(pixels, beacon_bright_threshold4);
            selected = __SEL(0x01010101U, 0U);
            bits = (selected * 0x01020408U) >> 24;
            beacon_bright_bits |= bits << shift;

            if((shift == 28U) || (x == BEACON_IMAGE_W - 4))
            {
                unsigned int word_index = (unsigned int)x >> 5;

                car_words[word_index] = car_bits;
                bright_words[word_index] = bright_bits;
                beacon_dark_words[word_index] = beacon_dark_bits;
                beacon_bright_words[word_index] = beacon_bright_bits;
                car_bits = 0U;
                bright_bits = 0U;
                beacon_dark_bits = 0U;
                beacon_bright_bits = 0U;
            }
        }
        if(build_integral != 0U)
        {
            preprocess_integral_write_pair(
                integral_previous, integral_current,
                BEACON_IMAGE_W + 5, &integral_row_sum,
                source[BEACON_IMAGE_W - 2],
                source[BEACON_IMAGE_W - 3]);
            preprocess_integral_write_pair(
                integral_previous, integral_current,
                BEACON_IMAGE_W + 7, &integral_row_sum,
                source[BEACON_IMAGE_W - 4],
                source[BEACON_IMAGE_W - 5]);
        }
    }

    if(bright <= GRAY_BEACON_SPARSE_MAX_BRIGHT_PIXELS)
    {
        int offset;

        for(offset = 1; offset <= GRAY_BEACON_BOX_PAD; offset++)
        {
            int integral_y =
                GRAY_BEACON_BOX_PAD + BEACON_IMAGE_H + offset;

            preprocess_padded_integral_row(
                g_gray_integral[integral_y - 1],
                g_gray_integral[integral_y],
                image[BEACON_IMAGE_H - 1 - offset]);
        }
    }

    if(bright_pixel_count != 0)
    {
        *bright_pixel_count = (unsigned short)bright;
    }
    return (float)sum / (float)(BEACON_IMAGE_W * BEACON_IMAGE_H);
}
#endif

static void threshold_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char threshold)
{
    int y;

    for(y = 0; y < BEACON_IMAGE_H; y++)
    {
        threshold_row_with_foreground_bits(
            image[y], g_binary[y],
            g_binary_foreground_words[y], threshold);
    }
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
                    g_binary[y][fill_x] = IMAGE_BINARY_FOREGROUND;
                    g_binary_foreground_words[y][fill_x >> 5] |=
                        1UL << (fill_x & 31);
                }
            }
        }
    }
}

static void bridge_upper_car_lamp_component_gaps(void)
{
    int y;
    int limit_y = (int)CAR_LAMP_UPPER_Y;
    float maximum_gap = CAR_LAMP_BRIDGE_MAX_GAP;

    if((limit_y <= 0) || (maximum_gap < 1.0f))
    {
        return;
    }
    if(limit_y > BEACON_IMAGE_H)
    {
        limit_y = BEACON_IMAGE_H;
    }

    for(y = 0; y < limit_y; y++)
    {
        int previous_x = -1;
        int word_index;

        for(word_index = 0; word_index < IMAGE_BINARY_WORD_COUNT;
            word_index++)
        {
            unsigned int foreground =
                g_component_source_words[y][word_index];

            while(foreground != 0U)
            {
                unsigned int bit =
                    image_count_trailing_zeros(foreground);
                int x = word_index * 32 + (int)bit;

                foreground &= foreground - 1U;
                if(x >= BEACON_IMAGE_W)
                {
                    continue;
                }

                if(previous_x >= 0)
                {
                    int gap = x - previous_x - 1;

                    if((gap > 0) && ((float)gap <= maximum_gap))
                    {
                        int fill_x;

                        for(fill_x = previous_x + 1; fill_x < x; fill_x++)
                        {
                            component_set_pixel(fill_x, y);
                        }
                    }
                }
                previous_x = x;
            }
        }
    }
}

static void threshold_car_lamp_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    int y;

    for(y = 0; y < BEACON_IMAGE_H; y++)
    {
        unsigned char threshold = CAR_LAMP_BINARY_THRESHOLD;

        if(y < CAR_LAMP_UPPER_Y)
        {
            threshold = CAR_LAMP_UPPER_THRESHOLD;
        }

        threshold_row_with_foreground_bits(
            image[y], g_binary[y],
            g_binary_foreground_words[y], threshold);
    }
    bridge_upper_car_lamp_gaps();
}

static void threshold_car_lamp_components(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    int y;

    for(y = 0; y < BEACON_IMAGE_H; y++)
    {
        unsigned char threshold = CAR_LAMP_BINARY_THRESHOLD;

        if(y < CAR_LAMP_UPPER_Y)
        {
            threshold = CAR_LAMP_UPPER_THRESHOLD;
        }

        threshold_row_to_component_words(
            image[y], g_component_source_words[y],
            g_component_remaining_words[y], threshold);
    }
    bridge_upper_car_lamp_component_gaps();
}

static void threshold_beacon_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
#if !IMAGE_USE_CM4_DSP
    int x;
#endif
    int y;

    for(y = 0; y < BEACON_IMAGE_H; y++)
    {
#if IMAGE_USE_CM4_DSP
        if(y < BEACON_TOP_THRESHOLD_Y)
        {
            threshold_row_dsp(image[y], g_binary[y],
                              BEACON_IMAGE_W,
                              (unsigned char)BEACON_EDGE_THRESHOLD);
        }
        else
        {
            threshold_row_dsp(image[y], g_binary[y],
                              BEACON_EDGE_LEFT_X,
                              (unsigned char)BEACON_EDGE_THRESHOLD);
            threshold_row_dsp(&image[y][BEACON_EDGE_LEFT_X],
                              &g_binary[y][BEACON_EDGE_LEFT_X],
                              BEACON_EDGE_RIGHT_X - BEACON_EDGE_LEFT_X,
                              (unsigned char)g_beacon_binary_threshold);
            threshold_row_dsp(&image[y][BEACON_EDGE_RIGHT_X],
                              &g_binary[y][BEACON_EDGE_RIGHT_X],
                              BEACON_IMAGE_W - BEACON_EDGE_RIGHT_X,
                              (unsigned char)BEACON_EDGE_THRESHOLD);
        }
#else
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
#endif
    }
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
            if((component_source_pixel(x, y) != 0U) &&
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
                component_set_pixel(x, y);
            }
        }
    }
    g_track_reinforced = 1U;
}

static int component_enqueue_span(
    int y,
    int seed_x,
    unsigned short *tail)
{
    int left;
    int right;

    if((tail == 0) || (*tail >= IMAGE_SPAN_QUEUE_SIZE) ||
       (y < 0) || (y >= BEACON_IMAGE_H) ||
       (seed_x < 0) || (seed_x >= BEACON_IMAGE_W) ||
       (component_remaining_pixel(seed_x, y) == 0U))
    {
        return -1;
    }

    left = seed_x;
    while((left > 0) &&
          (component_remaining_pixel(left - 1, y) != 0U))
    {
        left--;
    }
    right = seed_x;
    while((right + 1 < BEACON_IMAGE_W) &&
          (component_remaining_pixel(right + 1, y) != 0U))
    {
        right++;
    }
    component_clear_remaining_range(y, left, right);

    g_span_queue[*tail] = (unsigned int)y |
                          ((unsigned int)left << 8) |
                          ((unsigned int)right << 16);
    (*tail)++;
    return right;
}

static int32 component_square_sum_to(int value)
{
    if(value <= 0)
    {
        return 0;
    }
    return (int32)value * (int32)(value + 1) *
           (int32)(value * 2 + 1) / 6;
}

static component_t grow_component(
    unsigned char start_x,
    unsigned char start_y)
{
    unsigned short head = 0U;
    unsigned short tail = 0U;
    int sum_x = 0;
    int sum_y = 0;
    int32 sum_xx = 0;
    int32 sum_yy = 0;
    int32 sum_xy = 0;
    component_t comp;

#if defined(IMAGE_HOST_PROFILE)
    g_profile_component_count++;
#endif

    memset(&comp, 0, sizeof(comp));
    comp.min_x = start_x;
    comp.max_x = start_x;
    comp.min_y = start_y;
    comp.max_y = start_y;
    (void)component_enqueue_span(start_y, start_x, &tail);

    while(head < tail)
    {
        unsigned int packed = g_span_queue[head++];
        int y = (int)(packed & 0xFFU);
        int left = (int)((packed >> 8) & 0xFFU);
        int right = (int)((packed >> 16) & 0xFFU);
        int span_area = right - left + 1;
        int span_sum_x = (left + right) * span_area / 2;
        int neighbor_index;

        comp.area += span_area;
        sum_x += span_sum_x;
        sum_y += y * span_area;
        sum_xx += component_square_sum_to(right) -
                  component_square_sum_to(left - 1);
        sum_yy += (int32)y * (int32)y * (int32)span_area;
        sum_xy += (int32)y * (int32)span_sum_x;

        if(left < comp.min_x) comp.min_x = left;
        if(right > comp.max_x) comp.max_x = right;
        if(y < comp.min_y) comp.min_y = y;
        if(y > comp.max_y) comp.max_y = y;

        for(neighbor_index = 0; neighbor_index < 2; neighbor_index++)
        {
            int neighbor_y = (neighbor_index == 0) ? y - 1 : y + 1;
            int scan_x;
            int scan_right;

            if((neighbor_y < 0) || (neighbor_y >= BEACON_IMAGE_H))
            {
                continue;
            }
            scan_x = (left > 0) ? left - 1 : 0;
            scan_right = (right + 1 < BEACON_IMAGE_W) ?
                         right + 1 : BEACON_IMAGE_W - 1;
            scan_x = component_next_remaining_x(
                neighbor_y, scan_x, scan_right);
            while(scan_x >= 0)
            {
                int run_right;

                run_right = component_enqueue_span(
                    neighbor_y, scan_x, &tail);
                scan_x = component_next_remaining_x(
                    neighbor_y,
                    (run_right >= scan_x) ? run_right + 1 : scan_x + 1,
                    scan_right);
            }
        }
    }

    if(comp.area > 0)
    {
        float inv_area = 1.0f / (float)comp.area;

        comp.cx = (float)sum_x * inv_area;
        comp.cy = (float)sum_y * inv_area;
        comp.moment_xx = sum_xx;
        comp.moment_yy = sum_yy;
        comp.moment_xy = sum_xy;
        comp.valid = 1;
    }

    return comp;
}

static void component_calculate_shape(component_t *comp)
{
    float inv_area;
    float var_x;
    float var_y;
    float cov_xy;
    float trace;
    float det;
    float disc;
    float sqrt_disc;
    float eig_major;
    float eig_minor;

    if((comp == 0) || (comp->valid == 0U) ||
       (comp->area <= 0) || (comp->shape_valid != 0U))
    {
        return;
    }

#if defined(IMAGE_HOST_PROFILE)
    g_profile_shape_count++;
#endif

    inv_area = 1.0f / (float)comp->area;
    var_x = (float)comp->moment_xx * inv_area - comp->cx * comp->cx;
    var_y = (float)comp->moment_yy * inv_area - comp->cy * comp->cy;
    cov_xy = (float)comp->moment_xy * inv_area - comp->cx * comp->cy;
    trace = var_x + var_y;
    det = var_x * var_y - cov_xy * cov_xy;
    disc = trace * trace * 0.25f - det;
    if(disc < 0.0f) disc = 0.0f;
    sqrt_disc = sqrtf(disc);
    eig_major = trace * 0.5f + sqrt_disc;
    eig_minor = trace * 0.5f - sqrt_disc;
    if(eig_minor < 0.0f) eig_minor = 0.0f;

    comp->major = 4.0f * sqrtf(eig_major + 0.0001f);
    comp->minor = 4.0f * sqrtf(eig_minor + 0.0001f);
    if(comp->minor < 1.0f) comp->minor = 1.0f;
    comp->elongation = comp->major / comp->minor;
    comp->angle_x = var_x - var_y;
    comp->angle_y = 2.0f * cov_xy;
    comp->shape_valid = 1U;
}

static void component_calculate_angle(component_t *comp)
{
    if((comp == 0) || (comp->valid == 0U) ||
       (comp->angle_valid != 0U))
    {
        return;
    }
    component_calculate_shape(comp);
    if(comp->shape_valid == 0U)
    {
        return;
    }

#if defined(IMAGE_HOST_PROFILE)
    g_profile_angle_count++;
#endif
    comp->angle = 0.5f * atan2f(comp->angle_y, comp->angle_x) *
                  180.0f / PI_F;
    comp->angle_valid = 1U;
}

static void component_calculate_direction(component_t *comp)
{
    float angle;

    if((comp == 0) || (comp->valid == 0U) ||
       (comp->direction_valid != 0U))
    {
        return;
    }
    component_calculate_angle(comp);
    if(comp->angle_valid == 0U)
    {
        return;
    }

#if defined(IMAGE_HOST_PROFILE)
    g_profile_direction_count++;
#endif
    angle = comp->angle * (PI_F / 180.0f);
    comp->direction_cos = cosf(angle);
    comp->direction_sin = sinf(angle);
    comp->direction_valid = 1U;
}

static void component_get_direction(
    const component_t *comp,
    float *cos_a,
    float *sin_a)
{
    if((comp != 0) && (comp->direction_valid != 0U))
    {
        *cos_a = comp->direction_cos;
        *sin_a = comp->direction_sin;
    }
    else if(comp != 0)
    {
        float angle = comp->angle * (PI_F / 180.0f);

        *cos_a = cosf(angle);
        *sin_a = sinf(angle);
    }
    else
    {
        *cos_a = 1.0f;
        *sin_a = 0.0f;
    }
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

static unsigned char is_lamp_geometry_candidate(component_t *comp)
{
    if((comp == 0) || (comp->valid == 0) ||
       (comp->min_y <= COMPONENT_TOP_REJECT_Y) ||
       (comp->area < CAR_LAMP_MIN_AREA) ||
       (comp->area > CAR_LAMP_MAX_AREA) ||
       (car_lamp_below_horizon(comp) == 0U))
    {
        return 0U;
    }
    component_calculate_shape(comp);
    if((comp->elongation < CAR_LAMP_MIN_ELONGATION) ||
       (comp->major < CAR_LAMP_MIN_LENGTH))
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
    component_t *comp)
{
    unsigned int sum = 0U;
    unsigned int sum_sq = 0U;
    int count = 0;
    int full_count = 0;
    int saturated = 0;
    int threshold;
    int x;
    int y;
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
    component_calculate_direction(comp);
    if(((comp->min_x <= 0) || (comp->max_x >= BEACON_IMAGE_W - 1)) &&
       (fabsf(comp->angle) >= 60.0f))
    {
        return 0U;
    }
    cos_a = comp->direction_cos;
    sin_a = comp->direction_sin;
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
        for(x = min_x; x <= max_x; x++)
        {
            float dx = (float)x - comp->cx;
            float dy = (float)y - comp->cy;
            float major = dx * cos_a + dy * sin_a;
            float minor = -dx * sin_a + dy * cos_a;
            unsigned int gray = image[y][x];
            if((fabsf(major) > half_length) ||
               (fabsf(minor) > half_width))
            {
                continue;
            }
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
    int y;
    float best_score = 0.0f;
    unsigned char found = 0U;

    memset(best_lamp, 0, sizeof(*best_lamp));

    for(y = 0; y < BEACON_IMAGE_H; y++)
    {
        int word_index;

        for(word_index = 0; word_index < IMAGE_BINARY_WORD_COUNT;
            word_index++)
        {
            unsigned int foreground =
                g_binary_foreground_words[y][word_index];

            while(foreground != 0U)
            {
                unsigned int bit =
                    image_count_trailing_zeros(foreground);
                int x = word_index * 32 + (int)bit;
                component_t comp;
                float score;

                foreground &= foreground - 1U;
                if((x >= BEACON_IMAGE_W) ||
                   (component_remaining_pixel(x, y) == 0U))
                {
                    continue;
                }
                comp = grow_component(
                    (unsigned char)x, (unsigned char)y);
                if(is_lamp_geometry_candidate(&comp) == 0U)
                {
                    continue;
                }
                if(car_lamp_uniform_strip_valid(image, &comp) == 0U)
                {
                    continue;
                }

                score = (float)comp.area * comp.elongation;
                if((found == 0U) ||
                   (comp.elongation > best_lamp->elongation + 0.5f) ||
                   ((fabsf(comp.elongation - best_lamp->elongation) <=
                     0.5f) &&
                    ((comp.cy >
                      best_lamp->cy + CAR_LAMP_Y_PRIORITY_MARGIN) ||
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
            component_clear_pixel(x, y);
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
    lamp->direction_cos = track->direction_cos;
    lamp->direction_sin = track->direction_sin;
    lamp->direction_valid = track->direction_valid;
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
    lamp->shape_valid = 1U;
    lamp->angle_valid = 1U;
    return 1;
}

static void erase_temporal_lamp_from_binary(const component_t *lamp)
{
    int x;
    int y;
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

    component_get_direction(lamp, &cos_a, &sin_a);
    half_len = lamp->major * 0.5f + (float)CAR_LAMP_TEMPORAL_MASK_PAD;
    half_wid = lamp->minor * 0.5f + (float)CAR_LAMP_TEMPORAL_MASK_PAD;

    for(y = lamp->min_y; y <= lamp->max_y; y++)
    {
        for(x = lamp->min_x; x <= lamp->max_x; x++)
        {
            float dx = (float)x - lamp->cx;
            float dy = (float)y - lamp->cy;
            float major = dx * cos_a + dy * sin_a;
            float minor = -dx * sin_a + dy * cos_a;

            if((fabsf(major) <= half_len) && (fabsf(minor) <= half_wid))
            {
                component_clear_pixel(x, y);
            }
        }
    }
}

static int gray_reflect_index(int index, int limit)
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

static void gray_build_integral_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
#if IMAGE_USE_CM4_DSP
    /* CM4在单遍阈值预处理中已经同步构建原图积分图。 */
    (void)image;
#else
    int padded_y;

    memset(g_gray_integral[0], 0, sizeof(g_gray_integral[0]));
    for(padded_y = 0; padded_y < GRAY_BEACON_PADDED_H; padded_y++)
    {
        const unsigned char *source = image[gray_reflect_index(
            padded_y - GRAY_BEACON_BOX_PAD, BEACON_IMAGE_H)];
        const unsigned short *previous = g_gray_integral[padded_y];
        unsigned short *current = g_gray_integral[padded_y + 1];
        unsigned int row_sum = 0U;
        int output_x = 1;
        int x;

        current[0] = 0U;
        for(x = GRAY_BEACON_BOX_PAD; x > 0; x--)
        {
            row_sum += source[x];
            current[output_x] = (unsigned short)(
                previous[output_x] + row_sum);
            output_x++;
        }
        for(x = 0; x < BEACON_IMAGE_W; x++)
        {
            row_sum += source[x];
            current[output_x] = (unsigned short)(
                previous[output_x] + row_sum);
            output_x++;
        }
        for(x = BEACON_IMAGE_W - 2;
            x >= BEACON_IMAGE_W - 1 - GRAY_BEACON_BOX_PAD;
            x--)
        {
            row_sum += source[x];
            current[output_x] = (unsigned short)(
                previous[output_x] + row_sum);
            output_x++;
        }
    }
#endif
}

static unsigned int gray_integral_box_sum(int x, int y, int radius)
{
    int left = x + GRAY_BEACON_BOX_PAD - radius;
    int right = x + GRAY_BEACON_BOX_PAD + radius + 1;
    int top = y + GRAY_BEACON_BOX_PAD - radius;
    int bottom = y + GRAY_BEACON_BOX_PAD + radius + 1;

    return (unsigned short)(
        (int)g_gray_integral[bottom][right] -
        (int)g_gray_integral[top][right] -
        (int)g_gray_integral[bottom][left] +
        (int)g_gray_integral[top][left]);
}

static signed short gray_integral_response(int x, int y)
{
    unsigned int box3 = gray_integral_box_sum(x, y, 1);
    unsigned int box9 = gray_integral_box_sum(x, y, 4);

    return (signed short)(9 * (int)box3 - (int)box9);
}

static signed short gray_cached_integral_response(int x, int y)
{
    unsigned int slot = (unsigned int)y & (GRAY_SPARSE_CACHE_ROWS - 1U);
    unsigned int word_index = (unsigned int)x >> 5;
    unsigned int bit = 1UL << ((unsigned int)x & 31U);

    if((g_gray_sparse_response_valid[slot][word_index] & bit) == 0U)
    {
        g_gray_sparse_response[slot][x] = gray_integral_response(x, y);
        g_gray_sparse_response_valid[slot][word_index] |= bit;
    }
    return g_gray_sparse_response[slot][x];
}

static void gray_prepare_sparse_response_row(int y)
{
    unsigned int slot = (unsigned int)y & (GRAY_SPARSE_CACHE_ROWS - 1U);

    if(g_gray_sparse_response_row[slot] != (signed short)y)
    {
        g_gray_sparse_response_row[slot] = (signed short)y;
        memset(g_gray_sparse_response_valid[slot], 0,
               sizeof(g_gray_sparse_response_valid[slot]));
    }
}

static unsigned int image_count_trailing_zeros(unsigned int value)
{
#if defined(__ICCARM__)
    return __CLZ(__RBIT(value));
#elif defined(__GNUC__)
    return (unsigned int)__builtin_ctz(value);
#else
    unsigned int count = 0U;

    while((value & 1U) == 0U)
    {
        value >>= 1;
        count++;
    }
    return count;
#endif
}

static unsigned char gray_pixel(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int x,
    int y)
{
    return image[gray_reflect_index(y, BEACON_IMAGE_H)]
                [gray_reflect_index(x, BEACON_IMAGE_W)];
}

static void gray_refresh_vertical_padding(void)
{
#if IMAGE_USE_CM4_DSP
    unsigned int *vertical39 =
        &g_gray_vertical39_sum[GRAY_BEACON_BOX_PAD];
    int offset;

    for(offset = 1; offset <= GRAY_BEACON_BOX_PAD; offset++)
    {
        vertical39[-offset] = vertical39[offset];
        vertical39[BEACON_IMAGE_W - 1 + offset] =
            vertical39[BEACON_IMAGE_W - 1 - offset];
    }
#else
    unsigned short *vertical3 =
        &g_gray_vertical3_sum[GRAY_BEACON_BOX_PAD];
    unsigned short *vertical9 =
        &g_gray_vertical9_sum[GRAY_BEACON_BOX_PAD];
    int offset;

    for(offset = 1; offset <= GRAY_BEACON_BOX_PAD; offset++)
    {
        vertical3[-offset] = vertical3[offset];
        vertical9[-offset] = vertical9[offset];
        vertical3[BEACON_IMAGE_W - 1 + offset] =
            vertical3[BEACON_IMAGE_W - 1 - offset];
        vertical9[BEACON_IMAGE_W - 1 + offset] =
            vertical9[BEACON_IMAGE_W - 1 - offset];
    }
#endif
}

static void gray_initialize_vertical_box_sums(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int center_y)
{
#if IMAGE_USE_CM4_DSP
    unsigned int *vertical39 =
        &g_gray_vertical39_sum[GRAY_BEACON_BOX_PAD];
    int offset;
    int x;

    memset(g_gray_vertical39_sum, 0, sizeof(g_gray_vertical39_sum));
    for(offset = -4; offset <= 4; offset++)
    {
        const unsigned char *row =
            image[gray_reflect_index(center_y + offset, BEACON_IMAGE_H)];
        unsigned int include3 =
            ((offset >= -1) && (offset <= 1)) ? 1U : 0U;

        for(x = 0; x < BEACON_IMAGE_W; x++)
        {
            unsigned int gray = row[x];
            unsigned int packed = (gray << 16) | (gray * include3);

            vertical39[x] = __SADD16(vertical39[x], packed);
        }
    }
#else
    unsigned short *vertical3 =
        &g_gray_vertical3_sum[GRAY_BEACON_BOX_PAD];
    unsigned short *vertical9 =
        &g_gray_vertical9_sum[GRAY_BEACON_BOX_PAD];
    int offset;
    int x;

    memset(g_gray_vertical3_sum, 0, sizeof(g_gray_vertical3_sum));
    memset(g_gray_vertical9_sum, 0, sizeof(g_gray_vertical9_sum));

    for(offset = -4; offset <= 4; offset++)
    {
        const unsigned char *row =
            image[gray_reflect_index(center_y + offset, BEACON_IMAGE_H)];

        for(x = 0; x < BEACON_IMAGE_W; x++)
        {
            vertical9[x] += row[x];
            if((offset >= -1) && (offset <= 1))
            {
                vertical3[x] += row[x];
            }
        }
    }
#endif
    gray_refresh_vertical_padding();
}

static void gray_calculate_response_row(int response_slot)
{
#if IMAGE_USE_CM4_DSP
    const unsigned int *vertical39 =
        &g_gray_vertical39_sum[GRAY_BEACON_BOX_PAD];
    unsigned int horizontal39 = 0U;
    int x;

    for(x = -4; x <= 4; x++)
    {
        horizontal39 = __SADD16(
            horizontal39, vertical39[x] & 0xFFFF0000U);
        if((x >= -1) && (x <= 1))
        {
            horizontal39 = __SADD16(
                horizontal39, vertical39[x] & 0x0000FFFFU);
        }
    }
    g_gray_response_rows[response_slot][0] = (signed short)(
        9 * (int)(horizontal39 & 0xFFFFU) -
        (int)(horizontal39 >> 16));

    for(x = 1; x < BEACON_IMAGE_W - 1; x++)
    {
        unsigned int enter =
            (vertical39[x + 4] & 0xFFFF0000U) |
            (vertical39[x + 1] & 0x0000FFFFU);
        unsigned int leave =
            (vertical39[x - 5] & 0xFFFF0000U) |
            (vertical39[x - 2] & 0x0000FFFFU);

        horizontal39 = __SADD16(horizontal39, enter);
        horizontal39 = __SSUB16(horizontal39, leave);
        g_gray_response_rows[response_slot][x] = (signed short)(
            9 * (int)(horizontal39 & 0xFFFFU) -
            (int)(horizontal39 >> 16));
    }
#else
    const unsigned short *vertical3 =
        &g_gray_vertical3_sum[GRAY_BEACON_BOX_PAD];
    const unsigned short *vertical9 =
        &g_gray_vertical9_sum[GRAY_BEACON_BOX_PAD];
    unsigned int horizontal3 = 0U;
    unsigned int horizontal9 = 0U;
    int x;

    for(x = -4; x <= 4; x++)
    {
        horizontal9 += vertical9[x];
        if((x >= -1) && (x <= 1))
        {
            horizontal3 += vertical3[x];
        }
    }
    g_gray_response_rows[response_slot][0] =
        (signed short)(9 * (int)horizontal3 - (int)horizontal9);

    for(x = 1; x < BEACON_IMAGE_W - 1; x++)
    {
        horizontal3 += vertical3[x + 1];
        horizontal3 -= vertical3[x - 2];
        horizontal9 += vertical9[x + 4];
        horizontal9 -= vertical9[x - 5];
        g_gray_response_rows[response_slot][x] =
            (signed short)(9 * (int)horizontal3 - (int)horizontal9);
    }
#endif
}

static void gray_advance_vertical_box_sums(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int y)
{
#if IMAGE_USE_CM4_DSP
    unsigned int *vertical39 =
        &g_gray_vertical39_sum[GRAY_BEACON_BOX_PAD];
    const unsigned char *leave3 =
        image[gray_reflect_index(y - 1, BEACON_IMAGE_H)];
    const unsigned char *enter3 =
        image[gray_reflect_index(y + 2, BEACON_IMAGE_H)];
    const unsigned char *leave9 =
        image[gray_reflect_index(y - 4, BEACON_IMAGE_H)];
    const unsigned char *enter9 =
        image[gray_reflect_index(y + 5, BEACON_IMAGE_H)];
    int x;

    for(x = 0; x < BEACON_IMAGE_W; x++)
    {
        unsigned int enter =
            ((unsigned int)enter9[x] << 16) | enter3[x];
        unsigned int leave =
            ((unsigned int)leave9[x] << 16) | leave3[x];

        vertical39[x] = __SADD16(
            __SSUB16(vertical39[x], leave), enter);
    }
#else
    unsigned short *vertical3 =
        &g_gray_vertical3_sum[GRAY_BEACON_BOX_PAD];
    unsigned short *vertical9 =
        &g_gray_vertical9_sum[GRAY_BEACON_BOX_PAD];
    const unsigned char *leave3 =
        image[gray_reflect_index(y - 1, BEACON_IMAGE_H)];
    const unsigned char *enter3 =
        image[gray_reflect_index(y + 2, BEACON_IMAGE_H)];
    const unsigned char *leave9 =
        image[gray_reflect_index(y - 4, BEACON_IMAGE_H)];
    const unsigned char *enter9 =
        image[gray_reflect_index(y + 5, BEACON_IMAGE_H)];
    int x;

    for(x = 0; x < BEACON_IMAGE_W; x++)
    {
        vertical3[x] = (unsigned short)(
            (int)vertical3[x] + enter3[x] - leave3[x]);
        vertical9[x] = (unsigned short)(
            (int)vertical9[x] + enter9[x] - leave9[x]);
    }
#endif
    gray_refresh_vertical_padding();
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

#if IMAGE_USE_CM4_DSP
static unsigned int gray_sparse_center_word(int y, int word_index)
{
    unsigned int bits;

    if((y < 2) || (y > BEACON_IMAGE_H - 3) ||
       (word_index < 0) || (word_index >= IMAGE_BINARY_WORD_COUNT))
    {
        return 0U;
    }
    bits = g_preprocessed_bright_words[y][word_index];
    if(word_index == 0)
    {
        bits &= 0xFFFFFFFCU;
    }
    else if(word_index == IMAGE_BINARY_WORD_COUNT - 1)
    {
        bits &= (1UL << ((BEACON_IMAGE_W - 2) & 31)) - 1UL;
    }
    return bits;
}

static unsigned int gray_sparse_response_demand_word(
    int response_y,
    int word_index)
{
    unsigned int demand = 0U;
    int center_y;

    for(center_y = response_y - 1;
        center_y <= response_y + 1; center_y++)
    {
        unsigned int previous =
            gray_sparse_center_word(center_y, word_index - 1);
        unsigned int center =
            gray_sparse_center_word(center_y, word_index);
        unsigned int next =
            gray_sparse_center_word(center_y, word_index + 1);

        demand |= center | (center << 1) | (center >> 1);
        demand |= previous >> 31;
        demand |= (next & 1U) << 31;
    }
    return demand;
}

static void gray_generate_sparse_response_row(int response_y)
{
    signed short *responses =
        g_gray_sparse_response[(unsigned int)response_y &
                               (GRAY_SPARSE_CACHE_ROWS - 1U)];
    int word_index;

    for(word_index = 0; word_index < IMAGE_BINARY_WORD_COUNT; word_index++)
    {
        unsigned int demand = gray_sparse_response_demand_word(
            response_y, word_index);

        while(demand != 0U)
        {
            unsigned int bit = image_count_trailing_zeros(demand);
            int x = word_index * 32 + (int)bit;

            demand &= demand - 1U;
            if((x > 0) && (x < BEACON_IMAGE_W - 1))
            {
                responses[x] = gray_integral_response(x, response_y);
            }
        }
    }
}

static inline void gray_consider_batched_sparse_peak(
    int x,
    int y,
    const component_t *lamp,
    const component_t *temporal_lamp,
    float lamp_cos,
    float lamp_sin,
    float temporal_lamp_cos,
    float temporal_lamp_sin,
    gray_beacon_peak_t peaks[GRAY_BEACON_MAX_PEAKS],
    unsigned char *count)
{
    const signed short *top =
        g_gray_sparse_response[(unsigned int)(y - 1) &
                               (GRAY_SPARSE_CACHE_ROWS - 1U)];
    const signed short *center =
        g_gray_sparse_response[(unsigned int)y &
                               (GRAY_SPARSE_CACHE_ROWS - 1U)];
    const signed short *bottom =
        g_gray_sparse_response[(unsigned int)(y + 1) &
                               (GRAY_SPARSE_CACHE_ROWS - 1U)];
    signed short response = center[x];

    if(gray_point_below_horizon(x, y) == 0U)
    {
        return;
    }
    if((response <= 0) ||
       ((*count >= GRAY_BEACON_MAX_PEAKS) &&
        (response <= peaks[GRAY_BEACON_MAX_PEAKS - 1].response)))
    {
        return;
    }
    if((response < top[x - 1]) ||
       (response < center[x - 1]) ||
       (response < bottom[x - 1]) ||
       (response < top[x]) ||
       (response < bottom[x]) ||
       (response < top[x + 1]) ||
       (response < center[x + 1]) ||
       (response < bottom[x + 1]))
    {
        return;
    }
    if((gray_point_in_lamp_with_trig(
            x, y, lamp, lamp_cos, lamp_sin) == 0U) &&
       (gray_point_in_lamp_with_trig(
            x, y, temporal_lamp,
            temporal_lamp_cos, temporal_lamp_sin) == 0U) &&
       (gray_point_in_lamp_shadow(x, y, lamp) == 0U) &&
       (gray_point_in_lamp_shadow(x, y, temporal_lamp) == 0U))
    {
        gray_insert_peak(peaks, count, response, x, y);
    }
}
#endif

static unsigned char gray_find_sparse_integral_peaks(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *lamp,
    const component_t *temporal_lamp,
    float lamp_cos,
    float lamp_sin,
    float temporal_lamp_cos,
    float temporal_lamp_sin,
    gray_beacon_peak_t peaks[GRAY_BEACON_MAX_PEAKS])
{
    unsigned char count = 0U;
    int x;
    int y;
    gray_build_integral_image(image);
#if IMAGE_USE_CM4_DSP
    gray_generate_sparse_response_row(1);
    gray_generate_sparse_response_row(2);
    gray_generate_sparse_response_row(3);
#else
    {
        int slot;

        for(slot = 0; slot < GRAY_SPARSE_CACHE_ROWS; slot++)
        {
            g_gray_sparse_response_row[slot] = -1;
        }
    }
#endif
    for(y = 2; y <= BEACON_IMAGE_H - 3; y++)
    {
#if !IMAGE_USE_CM4_DSP
        gray_prepare_sparse_response_row(y - 1);
        gray_prepare_sparse_response_row(y);
        gray_prepare_sparse_response_row(y + 1);
#endif
#if IMAGE_USE_CM4_DSP
        for(x = 0; x < IMAGE_BINARY_WORD_COUNT; x++)
        {
            unsigned int bits = g_preprocessed_bright_words[y][x];

            if(x == 0)
            {
                bits &= 0xFFFFFFFCU;
            }
            else if(x == IMAGE_BINARY_WORD_COUNT - 1)
            {
                bits &= (1UL << ((BEACON_IMAGE_W - 2) & 31)) - 1UL;
            }
            while(bits != 0U)
            {
                unsigned int bit = image_count_trailing_zeros(bits);
                int peak_x = x * 32 + (int)bit;

                bits &= bits - 1U;
                gray_consider_batched_sparse_peak(
                    peak_x, y, lamp, temporal_lamp,
                    lamp_cos, lamp_sin,
                    temporal_lamp_cos, temporal_lamp_sin,
                    peaks, &count);
            }
        }
#else
        for(x = 2; x <= BEACON_IMAGE_W - 3; x++)
        {
            signed short response;

            if((image[y][x] < GRAY_BEACON_MIN_PEAK_GRAY) ||
               (gray_point_below_horizon(x, y) == 0U))
            {
                continue;
            }
            response = gray_cached_integral_response(x, y);
            if((response <= 0) ||
               ((count >= GRAY_BEACON_MAX_PEAKS) &&
                (response <= peaks[GRAY_BEACON_MAX_PEAKS - 1].response)))
            {
                continue;
            }
            if((response < gray_cached_integral_response(x - 1, y - 1)) ||
               (response < gray_cached_integral_response(x - 1, y)) ||
               (response < gray_cached_integral_response(x - 1, y + 1)) ||
               (response < gray_cached_integral_response(x, y - 1)) ||
               (response < gray_cached_integral_response(x, y + 1)) ||
               (response < gray_cached_integral_response(x + 1, y - 1)) ||
               (response < gray_cached_integral_response(x + 1, y)) ||
               (response < gray_cached_integral_response(x + 1, y + 1)))
            {
                continue;
            }
            if((gray_point_in_lamp_with_trig(
                    x, y, lamp, lamp_cos, lamp_sin) == 0U) &&
               (gray_point_in_lamp_with_trig(
                    x, y, temporal_lamp,
                    temporal_lamp_cos, temporal_lamp_sin) == 0U) &&
               (gray_point_in_lamp_shadow(x, y, lamp) == 0U) &&
               (gray_point_in_lamp_shadow(
                    x, y, temporal_lamp) == 0U))
            {
                gray_insert_peak(peaks, &count, response, x, y);
            }
        }
#endif
#if IMAGE_USE_CM4_DSP
        if(y < BEACON_IMAGE_H - 3)
        {
            gray_generate_sparse_response_row(y + 2);
        }
#endif
    }
    return count;
}

static unsigned char gray_find_box_peaks(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *lamp,
    const component_t *temporal_lamp,
    unsigned short bright_pixel_count,
    gray_beacon_peak_t peaks[GRAY_BEACON_MAX_PEAKS])
{
    unsigned char count = 0U;
    float lamp_cos = 1.0f;
    float lamp_sin = 0.0f;
    float temporal_lamp_cos = 1.0f;
    float temporal_lamp_sin = 0.0f;
    int x;
    int y;

    if((lamp != 0) && (lamp->valid != 0U))
    {
        component_get_direction(lamp, &lamp_cos, &lamp_sin);
    }
    if((temporal_lamp != 0) && (temporal_lamp->valid != 0U))
    {
        component_get_direction(
            temporal_lamp, &temporal_lamp_cos, &temporal_lamp_sin);
    }

    memset(peaks, 0, sizeof(gray_beacon_peak_t) * GRAY_BEACON_MAX_PEAKS);
    if(bright_pixel_count <= GRAY_BEACON_SPARSE_MAX_BRIGHT_PIXELS)
    {
        return gray_find_sparse_integral_peaks(
            image, lamp, temporal_lamp,
            lamp_cos, lamp_sin,
            temporal_lamp_cos, temporal_lamp_sin,
            peaks);
    }
    gray_initialize_vertical_box_sums(image, 1);

    for(y = 1; y <= BEACON_IMAGE_H - 2; y++)
    {
        int response_slot = y % 3;

        gray_calculate_response_row(response_slot);

        if((y >= 3) && (y <= BEACON_IMAGE_H - 2))
        {
            int center_y = y - 1;
            int top_slot = (y - 2) % 3;
            int center_slot = (y - 1) % 3;
            int bottom_slot = y % 3;

            for(x = 2; x <= BEACON_IMAGE_W - 3; x++)
            {
                signed short response = g_gray_response_rows[center_slot][x];
                if((response <= 0) ||
                   (image[center_y][x] < GRAY_BEACON_MIN_PEAK_GRAY) ||
                   ((count >= GRAY_BEACON_MAX_PEAKS) &&
                    (response <=
                     peaks[GRAY_BEACON_MAX_PEAKS - 1].response)) ||
                   (gray_point_below_horizon(x, center_y) == 0U))
                {
                    continue;
                }
                if((response < g_gray_response_rows[top_slot][x - 1]) ||
                   (response < g_gray_response_rows[center_slot][x - 1]) ||
                   (response < g_gray_response_rows[bottom_slot][x - 1]) ||
                   (response < g_gray_response_rows[top_slot][x]) ||
                   (response < g_gray_response_rows[bottom_slot][x]) ||
                   (response < g_gray_response_rows[top_slot][x + 1]) ||
                   (response < g_gray_response_rows[center_slot][x + 1]) ||
                   (response < g_gray_response_rows[bottom_slot][x + 1]))
                {
                    continue;
                }
                if((gray_point_in_lamp_with_trig(
                        x, center_y, lamp, lamp_cos, lamp_sin) == 0U) &&
                   (gray_point_in_lamp_with_trig(
                        x, center_y, temporal_lamp,
                        temporal_lamp_cos, temporal_lamp_sin) == 0U) &&
                   (gray_point_in_lamp_shadow(
                        x, center_y, lamp) == 0U) &&
                   (gray_point_in_lamp_shadow(
                        x, center_y, temporal_lamp) == 0U))
                {
                    gray_insert_peak(peaks, &count, response, x, center_y);
                }
            }
        }

        if(y < BEACON_IMAGE_H - 2)
        {
            gray_advance_vertical_box_sums(image, y);
        }
    }

    return count;
}

static float gray_image_mean(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    const unsigned char *pixel = &image[0][0];
    unsigned int sum = 0U;
    int index;

#if IMAGE_USE_CM4_DSP
    for(index = 0;
        index < (BEACON_IMAGE_W * BEACON_IMAGE_H) / 4;
        index++)
    {
        unsigned int pixels = *(const unsigned int *)pixel;

        sum = __USADA8(pixels, 0U, sum);
        pixel += 4;
    }
#else
    for(index = 0; index < BEACON_IMAGE_W * BEACON_IMAGE_H; index++)
    {
        sum += pixel[index];
    }
#endif
    return (float)sum / (float)(BEACON_IMAGE_W * BEACON_IMAGE_H);
}

static unsigned char gray_point_in_lamp_with_trig(
    int x,
    int y,
    const component_t *lamp,
    float cos_a,
    float sin_a)
{
    float dx;
    float dy;
    float major;
    float minor;

    if((lamp == 0) || (lamp->valid == 0U))
    {
        return 0U;
    }
    dx = (float)x - lamp->cx;
    dy = (float)y - lamp->cy;
    major = dx * cos_a + dy * sin_a;
    minor = -dx * sin_a + dy * cos_a;
    return ((fabsf(major) <= lamp->major * 0.5f + 2.0f) &&
            (fabsf(minor) <= lamp->minor * 0.5f + 2.0f)) ? 1U : 0U;
}

static unsigned char gray_point_in_lamp(
    int x,
    int y,
    const component_t *lamp)
{
    float cos_a;
    float sin_a;

    if((lamp == 0) || (lamp->valid == 0U))
    {
        return 0U;
    }
    component_get_direction(lamp, &cos_a, &sin_a);
    return gray_point_in_lamp_with_trig(
        x, y, lamp, cos_a, sin_a);
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

static unsigned char gray_local_features(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int center_x,
    int center_y,
    gray_beacon_features_t *features,
    unsigned char point_source_mode)
{
    static const signed char patch_half_width[15] =
        { 0, 3, 4, 5, 6, 6, 6, 7, 6, 6, 6, 5, 4, 3, 0 };
    static const signed char core_half_width[13] =
        { 0, 3, 4, 5, 5, 5, 6, 5, 5, 5, 4, 3, 0 };
    float background;
    unsigned int inner_sum = 0U;
    unsigned int middle_sum = 0U;
    int weight_sum_twice = 0;
    int inner_weight_twice = 0;
    int weight_x_twice = 0;
    int weight_y_twice = 0;
    int weight_xx_twice = 0;
    int weight_yy_twice = 0;
    int weight_xy_twice = 0;
    float weight_sum;
    float inner_weight;
    float weight_x;
    float weight_y;
    float weight_xx;
    float weight_yy;
    float weight_xy;
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
    int half_area = 0;
    int ring_count = 0;
    int low_rank;
    int high_rank;
    int cumulative;
    int low = 0;
    int high = 0;
    int low_found = 0;
    int ring_min = 255;
    int ring_max = 0;
    int background_twice;
    int value;
    unsigned char core_pixels[13][13];
    unsigned char histogram[256];
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
        int half_width = patch_half_width[dy + GRAY_BEACON_PATCH_RADIUS];

        for(dx = -half_width; dx <= half_width; dx++)
        {
            int radius2 = dx * dx + dy * dy;
            unsigned char pixel = gray_pixel(image, center_x + dx, center_y + dy);

            if(radius2 <= 36)
            {
                core_pixels[dy + 6][dx + 6] = pixel;
            }
            if((radius2 >= 25) && (radius2 <= 49))
            {
                histogram[pixel]++;
                ring_count++;
                if((int)pixel < ring_min) ring_min = pixel;
                if((int)pixel > ring_max) ring_max = pixel;
            }
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
        }
    }

    low_rank = (ring_count - 1) / 2;
    high_rank = ring_count / 2;
    cumulative = 0;
    for(value = ring_min; value <= ring_max; value++)
    {
        cumulative += histogram[value];
        if((low_found == 0) && (cumulative > low_rank))
        {
            low = value;
            low_found = 1;
        }
        if(cumulative > high_rank)
        {
            high = value;
            break;
        }
    }
    background = ((float)low + (float)high) * 0.5f;
    background_twice = low + high;

    features->inner_contrast = (float)inner_sum / 13.0f - background;
    features->background = background;
    features->radial_drop = (float)inner_sum / 13.0f -
                            (float)middle_sum / 36.0f;
    features->peak = peak;
    if((point_source_mode != 0U) &&
       (features->radial_drop < 31.0f) &&
       (features->inner_contrast < 170.0f))
    {
        return 0U;
    }

    for(dy = -6; dy <= 6; dy++)
    {
        int half_width = core_half_width[dy + 6];

        for(dx = -half_width; dx <= half_width; dx++)
        {
            int radius2 = dx * dx + dy * dy;
            unsigned char pixel = core_pixels[dy + 6][dx + 6];
            int weight_twice;

            weight_twice = (int)pixel * 2 - background_twice;
            if(weight_twice < 0)
            {
                weight_twice = 0;
            }
            weight_sum_twice += weight_twice;
            weight_x_twice += weight_twice * dx;
            weight_y_twice += weight_twice * dy;
            weight_xx_twice += weight_twice * dx * dx;
            weight_yy_twice += weight_twice * dy * dy;
            weight_xy_twice += weight_twice * dx * dy;
            if(radius2 <= 4)
            {
                inner_weight_twice += weight_twice;
            }
            if((int)pixel * 2 > (int)peak)
            {
                half_area++;
            }
        }
    }

    if(weight_sum_twice <= 0)
    {
        return 0U;
    }
    weight_sum = (float)weight_sum_twice * 0.5f;
    inner_weight = (float)inner_weight_twice * 0.5f;
    weight_x = (float)weight_x_twice * 0.5f;
    weight_y = (float)weight_y_twice * 0.5f;
    weight_xx = (float)weight_xx_twice * 0.5f;
    weight_yy = (float)weight_yy_twice * 0.5f;
    weight_xy = (float)weight_xy_twice * 0.5f;
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

    features->concentration = inner_weight / weight_sum;
    features->elongation_sq = eigen_major / eigen_minor;
    features->offset_sq = centroid_x * centroid_x + centroid_y * centroid_y;
    features->centroid_x = centroid_x;
    features->centroid_y = centroid_y;
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
                (features->elongation_sq <= 1.90f * 1.90f) &&
                (features->offset_sq <= 1.25f * 1.25f)) ? 1U : 0U;
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
            (features->elongation_sq <= 1.90f * 1.90f) &&
            (features->offset_sq <= 1.25f * 1.25f)) ? 1U : 0U;
}

static unsigned char gray_large_point_valid(
    const gray_beacon_features_t *features)
{
    return ((features != 0) &&
            (features->half_area >= GRAY_BEACON_LARGE_SHAPE_MIN_AREA) &&
            (features->peak >= 250U) &&
            (features->inner_contrast >= 190.0f) &&
            (features->radial_drop >= 35.0f) &&
            (features->elongation_sq <= 1.50f * 1.50f) &&
            (features->offset_sq <= 0.75f * 0.75f)) ? 1U : 0U;
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
            (features->elongation_sq <= 1.60f * 1.60f) &&
            (features->offset_sq <= 1.10f * 1.10f)) ? 1U : 0U;
}

static unsigned char gray_is_point_source(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int x,
    int y,
    float scene_mean,
    float *area,
    float *refined_x,
    float *refined_y)
{
    gray_beacon_features_t features;
    unsigned char accepted;

    if(gray_local_features(image, x, y, &features, 1U) == 0U)
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
                 (features.elongation_sq <= 1.55f * 1.55f) &&
                 (features.offset_sq <= 1.0f * 1.0f)) ||
                ((features.inner_contrast >= 165.0f) &&
                 (features.radial_drop >= 110.0f) &&
                 (features.elongation_sq <= 1.55f * 1.55f) &&
                 (features.offset_sq <= 1.0f * 1.0f)) ||
                ((scene_mean < GRAY_BEACON_DARK_SCENE_MEAN) &&
                 (features.half_area >= 4.5f) &&
                 (features.half_area <= GRAY_BEACON_COMPACT_MAX_AREA) &&
                 (features.peak >= 230U) &&
                 (features.radial_drop >= 120.0f) &&
                 (features.elongation_sq <= 1.50f * 1.50f) &&
                 (features.offset_sq <= 2.0f * 2.0f)) ||
                ((scene_mean >= GRAY_BEACON_DARK_SCENE_MEAN) &&
                 (scene_mean < GRAY_BEACON_LOW_LIGHT_MEAN) &&
                 (features.half_area >= GRAY_BEACON_COMPACT_MIN_AREA) &&
                 (features.half_area <= GRAY_BEACON_COMPACT_MAX_AREA) &&
                 (features.peak >= GRAY_BEACON_COMPACT_MIN_PEAK) &&
                 (features.radial_drop >= 70.0f) &&
                 (features.elongation_sq <= 1.80f * 1.80f) &&
                 (features.offset_sq <= 2.0f * 2.0f)) ||
                ((features.inner_contrast >= 170.0f) &&
                 (features.elongation_sq <= 1.48f * 1.48f) &&
                 (features.offset_sq <= 2.15f * 2.15f))) ? 1U : 0U;
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
    return accepted;
}

static unsigned char gray_is_saturated_component(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    component_t *comp,
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
       (comp->area < 30) || (comp->area > 120))
    {
        return 0U;
    }
    width = comp->max_x - comp->min_x + 1;
    height = comp->max_y - comp->min_y + 1;
    if(comp->area * 10 < width * height * 6)
    {
        return 0U;
    }
    component_calculate_shape(comp);
    if(comp->elongation > 1.5f)
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
    float area)
{
    int index;

    if((count == 0) || (x < 2.0f) || (x > BEACON_IMAGE_W - 3.0f) ||
       (y < 2.0f) || (y > BEACON_IMAGE_H - 3.0f))
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
    gray_beacon_candidate_t candidates[GRAY_BEACON_MAX_CANDIDATES],
    unsigned char *count)
{
    unsigned short evaluated = 0U;
    int minimum_area;
    int maximum_area;
    int y;

    if(gray_component_configuration(
           scene_mean, 0,
           &minimum_area, &maximum_area) == 0U)
    {
        return;
    }

#if IMAGE_USE_CM4_DSP
    component_use_words(
        (scene_mean < GRAY_BEACON_DARK_SCENE_MEAN) ?
            g_preprocessed_beacon_dark_words :
            g_preprocessed_beacon_bright_words);
#else
    component_use_words(g_preprocessed_beacon_words);
#endif
    erase_lamp_from_binary(lamp);
    erase_temporal_lamp_from_binary(temporal_lamp);
    for(y = 0; y < BEACON_IMAGE_H; y++)
    {
        int word_index;

        for(word_index = 0; word_index < IMAGE_BINARY_WORD_COUNT;
            word_index++)
        {
            unsigned int foreground =
                g_binary_foreground_words[y][word_index];

            while(foreground != 0U)
            {
                unsigned int bit =
                    image_count_trailing_zeros(foreground);
                int x = word_index * 32 + (int)bit;
                component_t comp;
                int center_x;
                int center_y;
                float area;
                unsigned char accepted;

                foreground &= foreground - 1U;
                if((x >= BEACON_IMAGE_W) ||
                   (component_remaining_pixel(x, y) == 0U))
                {
                    continue;
                }
                if(evaluated >= BEACON_MAX_EVALUATED_COMPONENTS)
                {
                    (void)grow_component(
                        (unsigned char)x, (unsigned char)y);
                    continue;
                }
                comp = grow_component(
                    (unsigned char)x, (unsigned char)y);
                evaluated++;
                if((comp.area < minimum_area) ||
                   (comp.area > maximum_area))
                {
                    continue;
                }
                center_x = (int)(comp.cx + 0.5f);
                center_y = (int)(comp.cy + 0.5f);
                if((gray_point_below_horizon(center_x, center_y) == 0U) ||
                   (gray_point_in_lamp(center_x, center_y, lamp) != 0U) ||
                   (gray_point_in_lamp(
                        center_x, center_y, temporal_lamp) != 0U) ||
                   (gray_point_in_lamp_shadow(
                        center_x, center_y, lamp) != 0U) ||
                   (gray_point_in_lamp_shadow(
                        center_x, center_y, temporal_lamp) != 0U))
                {
                    continue;
                }
                area = (float)comp.area;
                accepted = gray_top_edge_point_valid(
                    image, &comp, scene_mean);
                if(accepted == 0U)
                {
                    accepted =
                        (scene_mean < GRAY_BEACON_DARK_SCENE_MEAN) ?
                        gray_is_point_source(
                            image, center_x, center_y, scene_mean,
                            &area, 0, 0) :
                        gray_is_saturated_component(
                            image, &comp, scene_mean);
                }
                if(accepted != 0U)
                {
                    gray_add_candidate(
                        candidates, count, center_x, center_y,
                        (float)comp.area);
                }
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
    if(gray_local_features(
           image, center_x, center_y, &features, 0U) == 0U)
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
                 (features.elongation_sq <= 1.60f * 1.60f) &&
                 (features.offset_sq <= 1.10f * 1.10f)) ||
                ((candidate->area <= 60.0f) &&
                 (features.peak >= 250U) &&
                 (features.inner_contrast >= 190.0f) &&
                 (features.radial_drop >= 65.0f) &&
                 (features.elongation_sq <= 1.60f * 1.60f) &&
                 (features.offset_sq <= 1.10f * 1.10f))) ? 1U : 0U;
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

    component_get_direction(lamp, &cos_a, &sin_a);
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
            if((component_source_pixel(x, y) != 0U) &&
               (image[y][x] > max_gray))
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
            if(component_source_pixel(x, y) != 0U)
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
    if((is_beacon_shape_candidate(comp) == 0U) &&
       (is_saturated_top_beacon_near_lamp(image, comp, lamp) == 0U))
    {
        record_bad_shape_component(comp);
    }
    if((is_obvious_reflection(image, comp) != 0U) &&
       (is_vertical_top_beacon_near_lamp(image, comp, lamp) == 0U) &&
       (is_saturated_top_beacon_near_lamp(image, comp, lamp) == 0U))
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
       (is_component_in_lamp_core(comp, temporal_lamp) != 0))
    {
        return;
    }
    if(((is_near_lamp(comp, lamp) != 0) ||
        (is_near_lamp(comp, temporal_lamp) != 0)) &&
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

    threshold_component_image(image, BEACON_WEAK_CENTER_THRESHOLD);
    erase_lamp_from_binary(lamp);
    erase_temporal_lamp_from_binary(temporal_lamp);

    for(y = BEACON_WEAK_CENTER_MIN_Y; y < BEACON_WEAK_CENTER_MAX_Y; y++)
    {
        for(x = BEACON_WEAK_CENTER_MIN_X; x < BEACON_WEAK_CENTER_MAX_X; x++)
        {
            component_t comp;

            if(component_remaining_pixel(x, y) == 0U)
            {
                continue;
            }

            if(*evaluated_components >= BEACON_MAX_EVALUATED_COMPONENTS)
            {
                (void)grow_component(x, y);
                continue;
            }
            comp = grow_component(
                x, y);
            (*evaluated_components)++;
            component_calculate_shape(&comp);
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
    unsigned short bright_pixel_count,
    beacon_result_t *result)
{
    gray_beacon_peak_t peaks[GRAY_BEACON_MAX_PEAKS];
    gray_beacon_candidate_t candidates[GRAY_BEACON_MAX_CANDIDATES];
    unsigned char peak_count;
    unsigned char candidate_count = 0U;
    unsigned char index;

    memset(candidates, 0, sizeof(candidates));
    peak_count = gray_find_box_peaks(
        image, lamp, temporal_lamp, bright_pixel_count, peaks);
    for(index = 0U; index < peak_count; index++)
    {
        int x = peaks[index].x;
        int y = peaks[index].y;
        float area;
        float refined_x = (float)x;
        float refined_y = (float)y;

        if(gray_is_point_source(
               image, x, y, scene_mean,
               &area, &refined_x, &refined_y) != 0U)
        {
            gray_add_candidate(
                candidates, &candidate_count, refined_x, refined_y, area);
        }
    }
    gray_add_component_candidates(
        image, lamp, temporal_lamp, scene_mean,
        candidates, &candidate_count);
    gray_store_candidates(
        image, candidates, candidate_count, scene_mean, result);
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

static void temporal_track_set_car_direction(
    temporal_track_t *track,
    component_t *component)
{
    if((track == 0) || (component == 0) || (component->valid == 0U))
    {
        return;
    }

    component_calculate_direction(component);
    track->direction_cos = component->direction_cos;
    track->direction_sin = component->direction_sin;
    track->direction_valid = component->direction_valid;
}

static void start_pending_car(
    temporal_track_t *track,
    const beacon_rect_t *car,
    component_t *component)
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
    temporal_track_set_car_direction(track, component);
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

static void update_car_track(
    temporal_track_t *track,
    const beacon_rect_t *car,
    component_t *component)
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
    temporal_track_set_car_direction(track, component);
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
}

static void write_current_car_as_temporal(
    const beacon_rect_t *car,
    beacon_result_t *result)
{
    result->car_lamps[0] = *car;
    result->car_lamp_count = 1U;
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

static unsigned char car_lamp_matches_track_shape(const component_t *comp)
{
    float length_ratio;
    float width_ratio;

    if((comp == 0) || (g_car_track.length <= 0.0f) ||
       (g_car_track.width <= 0.0f))
    {
        return 0U;
    }
    length_ratio = comp->major / g_car_track.length;
    width_ratio = comp->minor / g_car_track.width;
    return ((length_ratio >= CAR_LAMP_TRACK_SIZE_MIN_RATIO) &&
            (length_ratio <= CAR_LAMP_TRACK_SIZE_MAX_RATIO) &&
            (width_ratio >= CAR_LAMP_TRACK_SIZE_MIN_RATIO) &&
            (width_ratio <= CAR_LAMP_TRACK_SIZE_MAX_RATIO)) ? 1U : 0U;
}

static unsigned char find_temporal_car_lamp(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    component_t *best_lamp)
{
    component_t temporal_lamp;
    component_t best_comp;
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    int y;
    float best_distance2 = 0.0f;
    unsigned char found = 0;

    if((image == 0) || (best_lamp == 0) ||
       (can_use_temporal_car_mask(&g_car_track) == 0) ||
       (component_from_temporal_car(&g_car_track, &temporal_lamp, 1U) == 0))
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

    component_use_words(g_preprocessed_car_words);
    for(y = min_y; y <= max_y; y++)
    {
        int word_index;
        int first_word = min_x >> 5;
        int last_word = max_x >> 5;

        for(word_index = first_word; word_index <= last_word;
            word_index++)
        {
            unsigned int foreground =
                g_binary_foreground_words[y][word_index];

            while(foreground != 0U)
            {
                unsigned int bit =
                    image_count_trailing_zeros(foreground);
                int candidate_x = word_index * 32 + (int)bit;
                component_t comp;

                foreground &= foreground - 1U;
                if((candidate_x < min_x) || (candidate_x > max_x) ||
                   (candidate_x >= BEACON_IMAGE_W) ||
                   (component_remaining_pixel(candidate_x, y) == 0U))
                {
                    continue;
                }
                comp = grow_component(
                    (unsigned char)candidate_x, (unsigned char)y);
                if(is_component_in_lamp_core(
                       &comp, &temporal_lamp) == 0)
                {
                    continue;
                }
                if(is_lamp_geometry_candidate(&comp) == 0U)
                {
                    continue;
                }
                if(car_lamp_matches_track_shape(&comp) == 0U)
                {
                    continue;
                }
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
    }

    if(found != 0)
    {
        *best_lamp = best_comp;
    }
    return found;
}

static unsigned char gray_car_lamp_valid(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    component_t *comp)
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
    int min_strip_fill_percent = CAR_LAMP_GRAY_STRIP_MIN_FILL_PERCENT;
    unsigned char peak = 0U;
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
    component_calculate_shape(comp);
    if((comp->major < CAR_LAMP_GRAY_MIN_LENGTH) ||
       (comp->major > CAR_LAMP_GRAY_MAX_LENGTH) ||
       (comp->minor < CAR_LAMP_GRAY_MIN_WIDTH) ||
       (comp->elongation < CAR_LAMP_GRAY_MIN_ELONGATION))
    {
        return 0U;
    }
    component_calculate_direction(comp);
    cos_a = comp->direction_cos;
    sin_a = comp->direction_sin;
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
        for(x = min_x; x <= max_x; x++)
        {
            float dx = (float)x - comp->cx;
            float dy = (float)y - comp->cy;
            float major = dx * cos_a + dy * sin_a;
            float minor = -dx * sin_a + dy * cos_a;

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
    for(y = comp->min_y; y <= comp->max_y; y++)
    {
        for(x = comp->min_x; x <= comp->max_x; x++)
        {
            unsigned char gray;

            gray = image[y][x];
            if(component_source_pixel(x, y) == 0U)
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
    if(g_car_track.confirmed != 0U)
    {
        component_t temporal_lamp;

        if((component_from_temporal_car(
                &g_car_track, &temporal_lamp, 1U) != 0U) &&
           (is_component_in_lamp_core(comp, &temporal_lamp) != 0U) &&
           (car_lamp_matches_track_shape(comp) != 0U))
        {
            min_strip_fill_percent = CAR_LAMP_GRAY_TRACK_MIN_FILL_PERCENT;
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
    if((count == 0) || (background_count == 0) ||
       (strip_count == 0) ||
       (strip_bright * 100 <
        strip_count * min_strip_fill_percent))
    {
        return 0U;
    }
    mean = (float)sum / (float)count;
    background = (float)background_sum / (float)background_count;
    return ((peak >= CAR_LAMP_GRAY_MIN_PEAK) &&
            (mean >= CAR_LAMP_GRAY_MIN_MEAN) &&
            (mean - background >= CAR_LAMP_GRAY_MIN_CONTRAST)) ? 1U : 0U;
}

static unsigned char find_gray_car_lamp(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    float scene_mean,
    component_t *best_lamp)
{
    int y;
    float best_score = 0.0f;
    unsigned char found = 0U;

    if((image == 0) || (best_lamp == 0) ||
       (scene_mean >= CAR_LAMP_GRAY_SCENE_MAX))
    {
        return 0U;
    }
    memset(best_lamp, 0, sizeof(*best_lamp));
    threshold_component_image(image, CAR_LAMP_GRAY_THRESHOLD);
    for(y = CAR_LAMP_MIN_CENTER_Y; y < BEACON_IMAGE_H; y++)
    {
        int word_index;

        for(word_index = 0; word_index < IMAGE_BINARY_WORD_COUNT;
            word_index++)
        {
            unsigned int foreground =
                g_binary_foreground_words[y][word_index];

            while(foreground != 0U)
            {
                unsigned int bit =
                    image_count_trailing_zeros(foreground);
                int x = word_index * 32 + (int)bit;
                component_t comp;
                float score;

                foreground &= foreground - 1U;
                if((x >= BEACON_IMAGE_W) ||
                   (component_remaining_pixel(x, y) == 0U))
                {
                    continue;
                }
                comp = grow_component(
                    (unsigned char)x, (unsigned char)y);
                if(gray_car_lamp_valid(image, &comp) == 0U)
                {
                    continue;
                }
                score = (float)comp.area * comp.elongation;
                if((found == 0U) ||
                   (comp.cy >
                    best_lamp->cy + CAR_LAMP_Y_PRIORITY_MARGIN) ||
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

static void update_temporal_car(
    beacon_result_t *result,
    component_t *measurement_component)
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
        start_pending_car(&g_car_track, measurement, measurement_component);
        write_current_car_as_temporal(measurement, result);
        return;
    }

    if((g_car_track.confirmed != 0U) &&
       (square_distance(g_car_track.x + g_car_track.vx,
                        g_car_track.y + g_car_track.vy,
                        measurement->cx, measurement->cy) >
        KALMAN_GATE_DISTANCE * KALMAN_GATE_DISTANCE))
    {
        start_pending_car(&g_car_track, measurement, measurement_component);
        write_current_car_as_temporal(measurement, result);
        return;
    }

    if((g_car_track.confirmed == 0U) &&
       (square_distance(g_car_track.x, g_car_track.y,
                        measurement->cx, measurement->cy) >
        KALMAN_NEW_TARGET_DISTANCE * KALMAN_NEW_TARGET_DISTANCE))
    {
        start_pending_car(&g_car_track, measurement, measurement_component);
        write_current_car_as_temporal(measurement, result);
        return;
    }

    update_car_track(&g_car_track, measurement, measurement_component);
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
    component_t temporal_lamp;
    unsigned char has_lamp;
    unsigned short bright_pixel_count;
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

    memset(&temporal_lamp, 0, sizeof(temporal_lamp));
#if IMAGE_USE_CM4_DSP
    scene_mean = preprocess_detection_words_dsp(
        image, &bright_pixel_count);
#else
    scene_mean = gray_image_mean(image);
    preprocess_detection_words(image, scene_mean, &bright_pixel_count);
#endif
    component_use_words(g_preprocessed_car_words);
    bridge_upper_car_lamp_component_gaps();
    memcpy(g_preprocessed_car_words, g_component_source_words,
           sizeof(g_preprocessed_car_words));
    has_lamp = find_car_lamp(image, &lamp);
    if(has_lamp == 0U)
    {
        has_lamp = find_gray_car_lamp(image, scene_mean, &lamp);
    }
    if(has_lamp == 0)
    {
        has_lamp = find_temporal_car_lamp(image, &lamp);
    }
    if(has_lamp == 0)
    {
        memset(&lamp, 0, sizeof(lamp));
    }
    if(can_use_temporal_car_mask(&g_car_track) != 0)
    {
        (void)component_from_temporal_car(&g_car_track, &temporal_lamp, 1U);
    }
    write_car_lamp(&lamp, result);
    find_beacons(image, &lamp, &temporal_lamp,
                 scene_mean, bright_pixel_count, result);
    update_temporal_car(result, (has_lamp != 0U) ? &lamp : 0);
    beacon_output_finalize(image, result);
}

#if defined(__ICCARM__)
#pragma data_alignment=4
#elif defined(__ARM_FEATURE_DSP)
__attribute__((aligned(4)))
#endif
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
}

void image_init(void)
{
    memset(s_image_frame, 0, sizeof(s_image_frame));
    image_clear_results();
    beacon_image_init();
    beacon_image_reset_temporal();

    mt9v03x_finish_flag = 0U;
    s_mt9v03x_initialized = (0U == mt9v03x_init()) ? 1U : 0U;
}

void image_update(void)
{
    beacon_result_t result;

    if(0U == image_latch_frame())
    {
        return;
    }

    beacon_image_process(s_image_frame, &result);
    image_store_result(&result);
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
        threshold_car_lamp_image(s_image_frame);
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
