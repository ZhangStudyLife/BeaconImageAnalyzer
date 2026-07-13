#include "beacon_image.h"
#include "beacon_image_config.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define CAR_LAMP_BINARY_THRESHOLD 200
#define CAR_LAMP_UPPER_THRESHOLD  150
#define CAR_LAMP_UPPER_Y          64
#define CAR_LAMP_BRIDGE_MAX_GAP   4
#define BEACON_BINARY_THRESHOLD   120
#define BEACON_OUTPUT_MAX_COUNT   3
#define BEACON_TRACK_THRESHOLD    105
#define BEACON_EDGE_THRESHOLD     80
#define BEACON_TOP_THRESHOLD_Y    45
#define BEACON_EDGE_MIN_AREA      2
#define BEACON_EDGE_MAX_AREA      60
#define BEACON_TOP_EDGE_MAX_AREA  50
#define BEACON_EDGE_TOP_Y         36
#define BEACON_EDGE_BOTTOM_Y      104
#define BEACON_EDGE_LEFT_X        19
#define BEACON_EDGE_RIGHT_X       172
#define COMPONENT_BOTTOM_REJECT_Y (BEACON_IMAGE_H - 1)
#define BEACON_LOCAL_RING_INNER   3
#define BEACON_LOCAL_RING_OUTER   8
#define BEACON_TOP_WEAK_Y         19
#define BEACON_TOP_WEAK_AREA_MAX  5
#define BEACON_TOP_DIFFUSE_AREA_MIN 6
#define BEACON_SIDE_DIFFUSE_AREA_MIN 12
#define BEACON_TOP_WEAK_GRAY_MIN  150
#define BEACON_ISOLATED_MIN_AREA  2
#define BEACON_ISOLATED_MAX_AREA  5
#define BEACON_ISOLATED_GRAY_MIN  120
#define BEACON_ISOLATED_BG_MAX    2
#define BEACON_LOWER_SMALL_Y      90
#define BEACON_WEAK_FOOTPRINT_RADIUS 4
#define BEACON_WEAK_FOOTPRINT_GRAY 40
#define BEACON_WEAK_FOOTPRINT_MAX 15
#define LAMP_MASK_PAD             2
#define LAMP_MASK_DOWN_PAD        6
#define LAMP_NEAR_BEACON_PAD      8
#define LAMP_NEAR_BEACON_MIN_AREA 21
#define LAMP_NEAR_BEACON_ISOLATED_MIN_AREA 3
#define LAMP_NEAR_BEACON_BACKGROUND_MAX 20
#define LAMP_NEAR_BEACON_GRAY_MIN 150
#define CAR_LAMP_MIN_AREA         24
#define CAR_LAMP_MAX_AREA         1200
#define CAR_LAMP_MIN_ELONGATION   1.6f
#define CAR_LAMP_MIN_LENGTH       12.0f
#define CAR_LAMP_EDGE_MAX_MISSES  3
#define CAR_LAMP_CENTER_MAX_MISSES 24
#define CAR_LAMP_TEMPORAL_EDGE_MARGIN 8
#define CAR_LAMP_TEMPORAL_MASK_PAD 4
#define CAR_LAMP_TEMPORAL_CORE_PAD 2
#define CAR_LAMP_TEMPORAL_TAKEOVER_PAD 10
#define CAR_LAMP_TEMPORAL_MIN_BRIGHT_AREA 3
#define B0_MATCH_DISTANCE         18.0f
#define B0_SWITCH_AREA_RATIO      1.70f
#define B0_SMALL_SWITCH_AREA      12.0f
#define B0_SMALL_SWITCH_RATIO     2.50f
#define B0_INIT_CONFIRM_FRAMES    2
#define BEACON_MAX_MISSES         3
#define KALMAN_GATE_DISTANCE      24.0f
#define KALMAN_NEW_TARGET_DISTANCE 36.0f
#define FILTER_POS_ALPHA          0.65f
#define FILTER_VEL_ALPHA          0.30f
#define ADAPTIVE_ENTER_BACKGROUND 3
#define ADAPTIVE_EXIT_BACKGROUND  2
#define ADAPTIVE_ENTER_FRAMES     3
#define ADAPTIVE_EXIT_FRAMES      15
#define ADAPTIVE_LOCAL_RADIUS     4
#define ADAPTIVE_OFFSET_MIN       (-40)
#define ADAPTIVE_OFFSET_MAX       60
#define ADAPTIVE_THRESHOLD_MIN    40
#define ADAPTIVE_BEACON_MAX       230
#define ADAPTIVE_CAR_LAMP_MAX     250
#define ADAPTIVE_BEACON_STRONG_CONTRAST 24
#define ADAPTIVE_CAR_STRONG_CONTRAST    30
#define ADAPTIVE_TRACK_CONTRAST         12
#define ADAPTIVE_REGION_COUNT      4
#define ADAPTIVE_HISTOGRAM_SIZE    256
#define IMAGE_QUEUE_SIZE          (BEACON_IMAGE_W * BEACON_IMAGE_H)
#define PI_F                      3.1415926f

typedef enum
{
    ADAPTIVE_REGION_TOP = 0,
    ADAPTIVE_REGION_LEFT,
    ADAPTIVE_REGION_RIGHT,
    ADAPTIVE_REGION_CENTER
} adaptive_region_t;

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

static unsigned char local_background_average(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp);
static unsigned char component_max_gray(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp);

static unsigned char g_binary[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_visit[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_queue_x[IMAGE_QUEUE_SIZE];
static unsigned char g_queue_y[IMAGE_QUEUE_SIZE];
static temporal_track_t g_b0_track;
static temporal_track_t g_car_track;
static unsigned char g_track_reinforced;
static unsigned char g_adaptive_active;
static unsigned char g_adaptive_enter_count;
static unsigned char g_adaptive_exit_count;
static unsigned char g_global_background;
static unsigned char g_region_background[ADAPTIVE_REGION_COUNT];
static signed char g_global_threshold_offset;
static signed char g_region_threshold_offset[ADAPTIVE_REGION_COUNT];
static uint16_t g_global_histogram[ADAPTIVE_HISTOGRAM_SIZE];
static uint16_t g_region_histogram[ADAPTIVE_REGION_COUNT][ADAPTIVE_HISTOGRAM_SIZE];

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static signed char clamp_offset(signed char value)
{
    int clamped = clamp_int((int)value,
                            ADAPTIVE_OFFSET_MIN,
                            ADAPTIVE_OFFSET_MAX);
    return (signed char)clamped;
}

static adaptive_region_t adaptive_region_for_pixel(int x, int y)
{
    if (y < BEACON_TOP_THRESHOLD_Y)
    {
        return ADAPTIVE_REGION_TOP;
    }
    if (x < BEACON_EDGE_LEFT_X)
    {
        return ADAPTIVE_REGION_LEFT;
    }
    if (x >= BEACON_EDGE_RIGHT_X)
    {
        return ADAPTIVE_REGION_RIGHT;
    }
    return ADAPTIVE_REGION_CENTER;
}

static unsigned char histogram_percentile(
    const uint16_t histogram[ADAPTIVE_HISTOGRAM_SIZE],
    int pixel_count,
    int numerator,
    int denominator)
{
    int gray;
    int cumulative = 0;
    int target;

    if (pixel_count <= 0 || denominator <= 0)
    {
        return 0;
    }
    target = (pixel_count * numerator + denominator - 1) / denominator;
    for (gray = 0; gray < ADAPTIVE_HISTOGRAM_SIZE; gray++)
    {
        cumulative += histogram[gray];
        if (cumulative >= target)
        {
            return (unsigned char)gray;
        }
    }
    return 255;
}

static void calculate_adaptive_backgrounds(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    int x;
    int y;
    int region_count[ADAPTIVE_REGION_COUNT] = { 0, 0, 0, 0 };

    memset(g_global_histogram, 0, sizeof(g_global_histogram));
    memset(g_region_histogram, 0, sizeof(g_region_histogram));

    for (y = 0; y < BEACON_TOP_THRESHOLD_Y; y++)
    {
        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            unsigned char gray = image[y][x];
            g_global_histogram[gray]++;
            g_region_histogram[ADAPTIVE_REGION_TOP][gray]++;
            region_count[ADAPTIVE_REGION_TOP]++;
        }
    }
    for (y = BEACON_TOP_THRESHOLD_Y; y < BEACON_IMAGE_H; y++)
    {
        for (x = 0; x < BEACON_EDGE_LEFT_X; x++)
        {
            unsigned char gray = image[y][x];
            g_global_histogram[gray]++;
            g_region_histogram[ADAPTIVE_REGION_LEFT][gray]++;
            region_count[ADAPTIVE_REGION_LEFT]++;
        }
        for (x = BEACON_EDGE_LEFT_X; x < BEACON_EDGE_RIGHT_X; x++)
        {
            unsigned char gray = image[y][x];
            g_global_histogram[gray]++;
            g_region_histogram[ADAPTIVE_REGION_CENTER][gray]++;
            region_count[ADAPTIVE_REGION_CENTER]++;
        }
        for (x = BEACON_EDGE_RIGHT_X; x < BEACON_IMAGE_W; x++)
        {
            unsigned char gray = image[y][x];
            g_global_histogram[gray]++;
            g_region_histogram[ADAPTIVE_REGION_RIGHT][gray]++;
            region_count[ADAPTIVE_REGION_RIGHT]++;
        }
    }

    g_global_background = histogram_percentile(
        g_global_histogram,
        BEACON_IMAGE_W * BEACON_IMAGE_H,
        1,
        4);
    for (x = 0; x < ADAPTIVE_REGION_COUNT; x++)
    {
        g_region_background[x] = histogram_percentile(
            g_region_histogram[x],
            region_count[x],
            1,
            4);
    }
}

static void update_adaptive_background(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    int i;
    int dark_pixel_count = 0;
    int percentile_target =
        (BEACON_IMAGE_W * BEACON_IMAGE_H + 3) / 4;
    const unsigned char *pixels = &image[0][0];

    if (g_adaptive_active == 0)
    {
        for (i = 0; i < BEACON_IMAGE_W * BEACON_IMAGE_H; i++)
        {
            if (pixels[i] <= ADAPTIVE_EXIT_BACKGROUND)
            {
                dark_pixel_count++;
            }
        }
        g_adaptive_exit_count = 0;
        if (dark_pixel_count < percentile_target)
        {
            if (g_adaptive_enter_count < ADAPTIVE_ENTER_FRAMES)
            {
                g_adaptive_enter_count++;
            }
            if (g_adaptive_enter_count >= ADAPTIVE_ENTER_FRAMES)
            {
                g_adaptive_active = 1;
                g_adaptive_exit_count = 0;
                calculate_adaptive_backgrounds(image);
            }
        }
        else
        {
            g_adaptive_enter_count = 0;
        }
        return;
    }

    calculate_adaptive_backgrounds(image);
    g_adaptive_enter_count = 0;
    if (g_global_background <= ADAPTIVE_EXIT_BACKGROUND)
    {
        if (g_adaptive_exit_count < ADAPTIVE_EXIT_FRAMES)
        {
            g_adaptive_exit_count++;
        }
        if (g_adaptive_exit_count >= ADAPTIVE_EXIT_FRAMES)
        {
            g_adaptive_active = 0;
            g_adaptive_exit_count = 0;
        }
    }
    else
    {
        g_adaptive_exit_count = 0;
    }
}

static int threshold_offset_for_region(adaptive_region_t region)
{
    return (int)g_global_threshold_offset +
           (int)g_region_threshold_offset[region];
}

static unsigned char threshold_offsets_are_zero(void)
{
    int region;
    if (g_global_threshold_offset != 0)
    {
        return 0;
    }
    for (region = 0; region < ADAPTIVE_REGION_COUNT; region++)
    {
        if (g_region_threshold_offset[region] != 0)
        {
            return 0;
        }
    }
    return 1;
}

static unsigned char effective_threshold(
    int base_threshold,
    adaptive_region_t region,
    int max_threshold)
{
    return (unsigned char)clamp_int(
        base_threshold + threshold_offset_for_region(region),
        ADAPTIVE_THRESHOLD_MIN,
        max_threshold);
}

static int adaptive_contrast_requirement(
    adaptive_region_t region,
    int base_contrast,
    int max_extra)
{
    int extra = g_region_background[region];
    if (extra > max_extra)
    {
        extra = max_extra;
    }
    return base_contrast + extra;
}

typedef enum
{
    ADAPTIVE_THRESHOLD_UNIFORM = 0,
    ADAPTIVE_THRESHOLD_CAR_LAMP,
    ADAPTIVE_THRESHOLD_BEACON
} adaptive_threshold_kind_t;

static int threshold_base_for_pixel(
    adaptive_threshold_kind_t kind,
    int uniform_threshold,
    adaptive_region_t region,
    int y)
{
    if (kind == ADAPTIVE_THRESHOLD_CAR_LAMP)
    {
        return (y < CAR_LAMP_UPPER_Y) ?
                   CAR_LAMP_UPPER_THRESHOLD :
                   CAR_LAMP_BINARY_THRESHOLD;
    }
    if (kind == ADAPTIVE_THRESHOLD_BEACON)
    {
        return (region == ADAPTIVE_REGION_CENTER) ?
                   BEACON_BINARY_THRESHOLD :
                   BEACON_EDGE_THRESHOLD;
    }
    return uniform_threshold;
}

static void threshold_adaptive_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    adaptive_threshold_kind_t kind,
    int uniform_threshold)
{
    int x;
    int y;
    unsigned char upper_threshold[ADAPTIVE_REGION_COUNT];
    unsigned char lower_threshold[ADAPTIVE_REGION_COUNT];

    if (threshold_offsets_are_zero() != 0)
    {
        if (kind == ADAPTIVE_THRESHOLD_UNIFORM)
        {
            int i;
            const unsigned char *src = &image[0][0];
            unsigned char *dst = &g_binary[0][0];
            for (i = 0; i < BEACON_IMAGE_W * BEACON_IMAGE_H; i++)
            {
                dst[i] = (src[i] >= uniform_threshold) ? 255 : 0;
            }
        }
        else if (kind == ADAPTIVE_THRESHOLD_CAR_LAMP)
        {
            for (y = 0; y < BEACON_IMAGE_H; y++)
            {
                unsigned char threshold = CAR_LAMP_BINARY_THRESHOLD;
                if (y < CAR_LAMP_UPPER_Y)
                {
                    threshold = CAR_LAMP_UPPER_THRESHOLD;
                }
                for (x = 0; x < BEACON_IMAGE_W; x++)
                {
                    g_binary[y][x] =
                        (image[y][x] >= threshold) ? 255 : 0;
                }
            }
        }
        else
        {
            for (y = 0; y < BEACON_IMAGE_H; y++)
            {
                for (x = 0; x < BEACON_IMAGE_W; x++)
                {
                    unsigned char threshold = BEACON_BINARY_THRESHOLD;
                    if (y < BEACON_TOP_THRESHOLD_Y ||
                        x < BEACON_EDGE_LEFT_X ||
                        x >= BEACON_EDGE_RIGHT_X)
                    {
                        threshold = BEACON_EDGE_THRESHOLD;
                    }
                    g_binary[y][x] =
                        (image[y][x] >= threshold) ? 255 : 0;
                }
            }
        }
        memset(g_visit, 0, sizeof(g_visit));
        return;
    }

    for (x = 0; x < ADAPTIVE_REGION_COUNT; x++)
    {
        adaptive_region_t region = (adaptive_region_t)x;
        int max_threshold =
            (kind == ADAPTIVE_THRESHOLD_CAR_LAMP) ?
                ADAPTIVE_CAR_LAMP_MAX :
                ADAPTIVE_BEACON_MAX;
        upper_threshold[x] = effective_threshold(
            threshold_base_for_pixel(kind, uniform_threshold, region, 0),
            region,
            max_threshold);
        lower_threshold[x] = effective_threshold(
            threshold_base_for_pixel(
                kind,
                uniform_threshold,
                region,
                CAR_LAMP_UPPER_Y),
            region,
            max_threshold);
    }

    for (y = 0; y < BEACON_TOP_THRESHOLD_Y; y++)
    {
        unsigned char threshold = upper_threshold[ADAPTIVE_REGION_TOP];
        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            g_binary[y][x] =
                (image[y][x] >= threshold) ? 255 : 0;
        }
    }
    for (y = BEACON_TOP_THRESHOLD_Y; y < BEACON_IMAGE_H; y++)
    {
        const unsigned char *thresholds =
            (kind == ADAPTIVE_THRESHOLD_CAR_LAMP &&
             y >= CAR_LAMP_UPPER_Y) ?
                lower_threshold : upper_threshold;
        for (x = 0; x < BEACON_EDGE_LEFT_X; x++)
        {
            g_binary[y][x] =
                (image[y][x] >= thresholds[ADAPTIVE_REGION_LEFT]) ?
                    255 : 0;
        }
        for (x = BEACON_EDGE_LEFT_X; x < BEACON_EDGE_RIGHT_X; x++)
        {
            g_binary[y][x] =
                (image[y][x] >= thresholds[ADAPTIVE_REGION_CENTER]) ?
                    255 : 0;
        }
        for (x = BEACON_EDGE_RIGHT_X; x < BEACON_IMAGE_W; x++)
        {
            g_binary[y][x] =
                (image[y][x] >= thresholds[ADAPTIVE_REGION_RIGHT]) ?
                    255 : 0;
        }
    }
    memset(g_visit, 0, sizeof(g_visit));
}

static int local_average_at(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int center_x,
    int center_y)
{
    int x;
    int y;
    int count = 0;
    int sum = 0;
    int min_x = center_x - ADAPTIVE_LOCAL_RADIUS;
    int max_x = center_x + ADAPTIVE_LOCAL_RADIUS;
    int min_y = center_y - ADAPTIVE_LOCAL_RADIUS;
    int max_y = center_y + ADAPTIVE_LOCAL_RADIUS;

    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if (max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;
    for (y = min_y; y <= max_y; y++)
    {
        for (x = min_x; x <= max_x; x++)
        {
            sum += image[y][x];
            count++;
        }
    }
    return (count > 0) ? sum / count : 0;
}

static unsigned char is_component_seed(unsigned char value)
{
    return (value != 0) ? 1 : 0;
}

void beacon_image_init(void)
{
    memset(g_binary, 0, sizeof(g_binary));
    memset(g_visit, 0, sizeof(g_visit));
    memset(g_region_background, 0, sizeof(g_region_background));
    memset(g_region_threshold_offset, 0,
           sizeof(g_region_threshold_offset));
    g_adaptive_active = 0;
    g_adaptive_enter_count = 0;
    g_adaptive_exit_count = 0;
    g_global_background = 0;
    g_global_threshold_offset = 0;
}

void beacon_image_reset_temporal(void)
{
    memset(&g_b0_track, 0, sizeof(g_b0_track));
    memset(&g_car_track, 0, sizeof(g_car_track));
    g_adaptive_active = 0;
    g_adaptive_enter_count = 0;
    g_adaptive_exit_count = 0;
    g_global_background = 0;
    memset(g_region_background, 0, sizeof(g_region_background));
}

void beacon_image_set_threshold_offsets(
    signed char global_offset,
    signed char top_offset,
    signed char left_offset,
    signed char right_offset,
    signed char center_offset)
{
    g_global_threshold_offset = clamp_offset(global_offset);
    g_region_threshold_offset[ADAPTIVE_REGION_TOP] =
        clamp_offset(top_offset);
    g_region_threshold_offset[ADAPTIVE_REGION_LEFT] =
        clamp_offset(left_offset);
    g_region_threshold_offset[ADAPTIVE_REGION_RIGHT] =
        clamp_offset(right_offset);
    g_region_threshold_offset[ADAPTIVE_REGION_CENTER] =
        clamp_offset(center_offset);
}

static void clear_result(beacon_result_t *result)
{
    if (result != 0)
    {
        memset(result, 0, sizeof(*result));
    }
}

static void threshold_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char threshold)
{
    threshold_adaptive_image(
        image,
        ADAPTIVE_THRESHOLD_UNIFORM,
        threshold);
}

static void bridge_upper_car_lamp_gaps(void)
{
    int y;
    int limit_y = CAR_LAMP_UPPER_Y;

    if (limit_y > BEACON_IMAGE_H)
    {
        limit_y = BEACON_IMAGE_H;
    }

    for (y = 0; y < limit_y; y++)
    {
        int x = 0;
        while (x < BEACON_IMAGE_W)
        {
            int start;
            int end;

            if (g_binary[y][x] != 0)
            {
                x++;
                continue;
            }

            start = x;
            while (x < BEACON_IMAGE_W && g_binary[y][x] == 0)
            {
                x++;
            }
            end = x;

            if (start > 0 && end < BEACON_IMAGE_W &&
                end - start <= CAR_LAMP_BRIDGE_MAX_GAP)
            {
                int fill_x;
                for (fill_x = start; fill_x < end; fill_x++)
                {
                    g_binary[y][fill_x] = 255;
                }
            }
        }
    }
}

static void threshold_car_lamp_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    threshold_adaptive_image(
        image,
        ADAPTIVE_THRESHOLD_CAR_LAMP,
        0);
    bridge_upper_car_lamp_gaps();
    memset(g_visit, 0, sizeof(g_visit));
}

static void threshold_beacon_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    threshold_adaptive_image(
        image,
        ADAPTIVE_THRESHOLD_BEACON,
        0);
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

    g_track_reinforced = 0;
    if (g_b0_track.confirmed == 0 ||
        g_b0_track.misses >= BEACON_MAX_MISSES)
    {
        return;
    }
    cx = (int)((float)BEACON_IMAGE_W * 0.5f -
               g_b0_track.x - g_b0_track.vx + 0.5f);
    cy = (int)((float)BEACON_IMAGE_H * 0.5f +
               g_b0_track.y + g_b0_track.vy + 0.5f);
    min_x = cx - (int)B0_MATCH_DISTANCE;
    max_x = cx + (int)B0_MATCH_DISTANCE;
    min_y = cy - (int)B0_MATCH_DISTANCE;
    max_y = cy + (int)B0_MATCH_DISTANCE;
    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if (max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;

    for (y = min_y; y <= max_y; y++)
    {
        for (x = min_x; x <= max_x; x++)
        {
            if (g_binary[y][x] != 0 && ++count >= BEACON_EDGE_MIN_AREA)
            {
                return;
            }
        }
    }
    for (y = min_y; y <= max_y; y++)
    {
        for (x = min_x; x <= max_x; x++)
        {
            adaptive_region_t region = adaptive_region_for_pixel(x, y);
            int track_threshold = effective_threshold(
                BEACON_TRACK_THRESHOLD,
                region,
                ADAPTIVE_BEACON_MAX);
            if (image[y][x] >= track_threshold &&
                (g_adaptive_active == 0 ||
                 image[y][x] - local_average_at(image, x, y) >=
                     adaptive_contrast_requirement(
                         region,
                         ADAPTIVE_TRACK_CONTRAST,
                         6)))
            {
                g_binary[y][x] = 255;
            }
        }
    }
    g_track_reinforced = 1;
}

static component_t grow_component(unsigned char start_x, unsigned char start_y)
{
    static const signed char dx[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
    static const signed char dy[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };
    unsigned short head = 0;
    unsigned short tail = 0;
    int sum_x = 0;
    int sum_y = 0;
    float sum_xx = 0.0f;
    float sum_yy = 0.0f;
    float sum_xy = 0.0f;
    component_t comp;

    memset(&comp, 0, sizeof(comp));
    comp.min_x = start_x;
    comp.max_x = start_x;
    comp.min_y = start_y;
    comp.max_y = start_y;
    g_queue_x[tail] = start_x;
    g_queue_y[tail] = start_y;
    tail++;
    g_visit[start_y][start_x] = 1;

    while (head < tail)
    {
        unsigned char i;
        unsigned char x = g_queue_x[head];
        unsigned char y = g_queue_y[head];
        head++;

        comp.area++;
        sum_x += x;
        sum_y += y;
        sum_xx += (float)x * (float)x;
        sum_yy += (float)y * (float)y;
        sum_xy += (float)x * (float)y;
        if ((int)x < comp.min_x) comp.min_x = x;
        if ((int)x > comp.max_x) comp.max_x = x;
        if ((int)y < comp.min_y) comp.min_y = y;
        if ((int)y > comp.max_y) comp.max_y = y;

        for (i = 0; i < 8; i++)
        {
            int nx = (int)x + dx[i];
            int ny = (int)y + dy[i];
            if (nx < 0 || nx >= BEACON_IMAGE_W ||
                ny < 0 || ny >= BEACON_IMAGE_H)
            {
                continue;
            }
            if (g_binary[ny][nx] == 0 || g_visit[ny][nx] != 0 ||
                tail >= IMAGE_QUEUE_SIZE)
            {
                continue;
            }
            g_visit[ny][nx] = 1;
            g_queue_x[tail] = (unsigned char)nx;
            g_queue_y[tail] = (unsigned char)ny;
            tail++;
        }
    }

    if (comp.area > 0)
    {
        float inv_area = 1.0f / (float)comp.area;
        float var_x;
        float var_y;
        float cov_xy;
        float trace;
        float det;
        float disc;
        float eig_major;
        float eig_minor;

        comp.cx = (float)sum_x * inv_area;
        comp.cy = (float)sum_y * inv_area;
        var_x = sum_xx * inv_area - comp.cx * comp.cx;
        var_y = sum_yy * inv_area - comp.cy * comp.cy;
        cov_xy = sum_xy * inv_area - comp.cx * comp.cy;
        trace = var_x + var_y;
        det = var_x * var_y - cov_xy * cov_xy;
        disc = trace * trace * 0.25f - det;
        if (disc < 0.0f) disc = 0.0f;
        eig_major = trace * 0.5f + sqrtf(disc);
        eig_minor = trace * 0.5f - sqrtf(disc);
        if (eig_minor < 0.0f) eig_minor = 0.0f;

        comp.major = 4.0f * sqrtf(eig_major + 0.0001f);
        comp.minor = 4.0f * sqrtf(eig_minor + 0.0001f);
        if (comp.minor < 1.0f) comp.minor = 1.0f;
        comp.elongation = comp.major / comp.minor;
        comp.angle = 0.5f * atan2f(2.0f * cov_xy, var_x - var_y) * 180.0f / PI_F;
        comp.valid = 1;
    }
    return comp;
}

static unsigned char is_lamp_candidate(const component_t *comp)
{
    return (comp != 0 &&
            comp->valid != 0 &&
            comp->max_y < COMPONENT_BOTTOM_REJECT_Y &&
            comp->area >= CAR_LAMP_MIN_AREA &&
            comp->area <= CAR_LAMP_MAX_AREA &&
            comp->elongation >= CAR_LAMP_MIN_ELONGATION &&
            comp->major >= CAR_LAMP_MIN_LENGTH) ? 1 : 0;
}

static unsigned char passes_adaptive_component_contrast(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp,
    int base_contrast)
{
    int image_x;
    int image_y;
    int contrast;
    adaptive_region_t region;

    if (g_adaptive_active == 0)
    {
        return 1;
    }
    if (image == 0 || comp == 0 || comp->valid == 0)
    {
        return 0;
    }
    image_x = clamp_int((int)(comp->cx + 0.5f), 0, BEACON_IMAGE_W - 1);
    image_y = clamp_int((int)(comp->cy + 0.5f), 0, BEACON_IMAGE_H - 1);
    region = adaptive_region_for_pixel(image_x, image_y);
    contrast = (int)component_max_gray(image, comp) -
               (int)local_background_average(image, comp);
    return (contrast >= adaptive_contrast_requirement(
                            region, base_contrast, 12)) ? 1 : 0;
}

static unsigned char find_car_lamp(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    component_t *best_lamp)
{
    unsigned char x;
    unsigned char y;
    float best_score = 0.0f;
    unsigned char found = 0;

    memset(best_lamp, 0, sizeof(*best_lamp));
    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            component_t comp;
            float score;

            if (is_component_seed(g_binary[y][x]) == 0 ||
                g_visit[y][x] != 0)
            {
                continue;
            }
            comp = grow_component(x, y);
            if (is_lamp_candidate(&comp) == 0)
            {
                continue;
            }
            if (passes_adaptive_component_contrast(
                    image,
                    &comp,
                    ADAPTIVE_CAR_STRONG_CONTRAST) == 0)
            {
                continue;
            }
            score = (float)comp.area * comp.elongation;
            if (found == 0 || score > best_score)
            {
                *best_lamp = comp;
                best_score = score;
                found = 1;
            }
        }
    }
    return found;
}

static void write_car_lamp(const component_t *lamp, beacon_result_t *result)
{
    if (lamp == 0 || lamp->valid == 0)
    {
        result->car_lamp_count = 0;
        return;
    }
    result->car_lamps[0].cx = (float)BEACON_IMAGE_W * 0.5f - lamp->cx;
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

    if (lamp == 0 || lamp->valid == 0)
    {
        return;
    }

    min_x = lamp->min_x - LAMP_MASK_PAD;
    max_x = lamp->max_x + LAMP_MASK_PAD;
    min_y = lamp->min_y - LAMP_MASK_PAD;
    max_y = lamp->max_y + LAMP_MASK_DOWN_PAD;
    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if (max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;

    for (y = min_y; y <= max_y; y++)
    {
        for (x = min_x; x <= max_x; x++)
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

    if (track == 0 || lamp == 0 || track->confirmed == 0 ||
        track->length <= 0.0f || track->width <= 0.0f)
    {
        return 0;
    }

    memset(lamp, 0, sizeof(*lamp));
    image_cx = (float)BEACON_IMAGE_W * 0.5f - track->x;
    image_cy = track->y + (float)BEACON_IMAGE_H * 0.5f;
    if (predicted != 0)
    {
        image_cx -= track->vx;
        image_cy += track->vy;
    }

    half_len = track->length * 0.5f + (float)CAR_LAMP_TEMPORAL_MASK_PAD;
    half_wid = track->width * 0.5f + (float)CAR_LAMP_TEMPORAL_MASK_PAD;
    radius = sqrtf(half_len * half_len + half_wid * half_wid);

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
    if (lamp->min_x < 0) lamp->min_x = 0;
    if (lamp->min_y < 0) lamp->min_y = 0;
    if (lamp->max_x >= BEACON_IMAGE_W) lamp->max_x = BEACON_IMAGE_W - 1;
    if (lamp->max_y >= BEACON_IMAGE_H) lamp->max_y = BEACON_IMAGE_H - 1;
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

    if (lamp == 0 || lamp->valid == 0)
    {
        return;
    }

    angle = lamp->angle * (PI_F / 180.0f);
    cos_a = cosf(angle);
    sin_a = sinf(angle);
    half_len = lamp->major * 0.5f + (float)CAR_LAMP_TEMPORAL_MASK_PAD;
    half_wid = lamp->minor * 0.5f + (float)CAR_LAMP_TEMPORAL_MASK_PAD;

    for (y = lamp->min_y; y <= lamp->max_y; y++)
    {
        for (x = lamp->min_x; x <= lamp->max_x; x++)
        {
            float dx = (float)x - lamp->cx;
            float dy = (float)y - lamp->cy;
            float major = dx * cos_a + dy * sin_a;
            float minor = -dx * sin_a + dy * cos_a;

            if (fabsf(major) <= half_len && fabsf(minor) <= half_wid)
            {
                g_binary[y][x] = 0;
            }
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

    if (comp == 0 || comp->valid == 0 ||
        lamp == 0 || lamp->valid == 0)
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

    return (fabsf(major) <= lamp->major * 0.5f + (float)CAR_LAMP_TEMPORAL_CORE_PAD &&
            fabsf(minor) <= lamp->minor * 0.5f + (float)CAR_LAMP_TEMPORAL_CORE_PAD) ? 1 : 0;
}

static unsigned char is_near_lamp(const component_t *comp, const component_t *lamp)
{
    if (comp == 0 || lamp == 0 || lamp->valid == 0)
    {
        return 0;
    }
    return (comp->max_x >= lamp->min_x - LAMP_NEAR_BEACON_PAD &&
            comp->min_x <= lamp->max_x + LAMP_NEAR_BEACON_PAD &&
            comp->max_y >= lamp->min_y - LAMP_NEAR_BEACON_PAD &&
            comp->min_y <= lamp->max_y + LAMP_NEAR_BEACON_PAD) ? 1 : 0;
}

static unsigned char is_edge_beacon_area(const component_t *comp)
{
    if (comp == 0 || comp->valid == 0)
    {
        return 0;
    }
    return (comp->min_y < BEACON_EDGE_TOP_Y ||
            comp->max_y >= BEACON_EDGE_BOTTOM_Y ||
            comp->min_x < BEACON_EDGE_LEFT_X ||
            comp->max_x >= BEACON_EDGE_RIGHT_X) ? 1 : 0;
}

static unsigned char is_side_edge_beacon_area(const component_t *comp)
{
    if (comp == 0 || comp->valid == 0)
    {
        return 0;
    }
    return (comp->min_x < BEACON_EDGE_LEFT_X ||
            comp->max_x >= BEACON_EDGE_RIGHT_X) ? 1 : 0;
}

static unsigned char is_incomplete_border_component(const component_t *comp)
{
    return (comp != 0 && comp->valid != 0 &&
            (comp->min_x <= 0 ||
             comp->min_y <= 0 ||
             comp->max_x >= BEACON_IMAGE_W - 1 ||
             comp->max_y >= BEACON_IMAGE_H - 1)) ? 1 : 0;
}

static unsigned char matches_confirmed_beacon_track(const component_t *comp)
{
    float x;
    float y;
    float dx;
    float dy;

    if (comp == 0 || comp->valid == 0 || g_b0_track.confirmed == 0)
    {
        return 0;
    }
    x = (float)BEACON_IMAGE_W * 0.5f - comp->cx;
    y = comp->cy - (float)BEACON_IMAGE_H * 0.5f;
    dx = x - (g_b0_track.x + g_b0_track.vx);
    dy = y - (g_b0_track.y + g_b0_track.vy);
    return (dx * dx + dy * dy <=
            KALMAN_GATE_DISTANCE * KALMAN_GATE_DISTANCE) ? 1 : 0;
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

    if (image == 0 || comp == 0 || comp->valid == 0)
    {
        return 0;
    }

    cx = (int)(comp->cx + 0.5f);
    cy = (int)(comp->cy + 0.5f);
    min_x = cx - BEACON_LOCAL_RING_OUTER;
    max_x = cx + BEACON_LOCAL_RING_OUTER;
    min_y = cy - BEACON_LOCAL_RING_OUTER;
    max_y = cy + BEACON_LOCAL_RING_OUTER;
    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if (max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;

    for (y = min_y; y <= max_y; y++)
    {
        for (x = min_x; x <= max_x; x++)
        {
            int dx = x - cx;
            int dy = y - cy;
            int dist = (dx < 0) ? -dx : dx;
            int abs_dy = (dy < 0) ? -dy : dy;
            if (abs_dy > dist) dist = abs_dy;
            if (dist <= BEACON_LOCAL_RING_INNER ||
                dist > BEACON_LOCAL_RING_OUTER)
            {
                continue;
            }
            sum += image[y][x];
            count++;
        }
    }
    if (count == 0)
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
    unsigned char max_gray = 0;

    if (image == 0 || comp == 0 || comp->valid == 0)
    {
        return 0;
    }

    for (y = comp->min_y; y <= comp->max_y; y++)
    {
        for (x = comp->min_x; x <= comp->max_x; x++)
        {
            if (g_binary[y][x] != 0 && image[y][x] > max_gray)
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

    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if (max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;
    for (y = min_y; y <= max_y; y++)
    {
        for (x = min_x; x <= max_x; x++)
        {
            if (image[y][x] >= BEACON_WEAK_FOOTPRINT_GRAY &&
                ++count > BEACON_WEAK_FOOTPRINT_MAX)
            {
                return 1;
            }
        }
    }
    return 0;
}

static unsigned char is_isolated_small_beacon(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp)
{
    if (comp == 0 || comp->valid == 0 ||
        comp->area < BEACON_ISOLATED_MIN_AREA ||
        comp->area > BEACON_ISOLATED_MAX_AREA)
    {
        return 0;
    }
    if (component_max_gray(image, comp) < BEACON_ISOLATED_GRAY_MIN)
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
    if (comp == 0 || comp->valid == 0 ||
        comp->area < LAMP_NEAR_BEACON_ISOLATED_MIN_AREA)
    {
        return 0;
    }
    if (component_max_gray(image, comp) < LAMP_NEAR_BEACON_GRAY_MIN)
    {
        return 0;
    }
    return (local_background_average(image, comp) <=
            LAMP_NEAR_BEACON_BACKGROUND_MAX) ? 1 : 0;
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

    if (comp == 0 || comp->valid == 0)
    {
        return;
    }
    if (passes_adaptive_component_contrast(
            image,
            comp,
            ADAPTIVE_BEACON_STRONG_CONTRAST) == 0)
    {
        return;
    }
    if (comp->max_y >= COMPONENT_BOTTOM_REJECT_Y)
    {
        return;
    }
    if (comp->cy >= BEACON_LOWER_SMALL_Y &&
        comp->area < BEACON_MIN_COMPONENT_AREA)
    {
        return;
    }
    is_edge = is_edge_beacon_area(comp);
    is_side_edge = is_side_edge_beacon_area(comp);
    if (((comp->min_y <= BEACON_TOP_WEAK_Y &&
          comp->area <= BEACON_TOP_WEAK_AREA_MAX) ||
         (comp->min_y < BEACON_EDGE_TOP_Y &&
          comp->area >= BEACON_TOP_DIFFUSE_AREA_MIN &&
          comp->area <= BEACON_TOP_EDGE_MAX_AREA &&
          matches_confirmed_beacon_track(comp) == 0) ||
         (is_side_edge != 0 &&
          comp->area >= BEACON_SIDE_DIFFUSE_AREA_MIN &&
          comp->area <= BEACON_EDGE_MAX_AREA)) &&
        component_max_gray(image, comp) < BEACON_TOP_WEAK_GRAY_MIN)
    {
        return;
    }
    if (is_edge != 0 &&
        comp->area <= BEACON_ISOLATED_MAX_AREA &&
        component_max_gray(image, comp) < BEACON_ISOLATED_GRAY_MIN &&
        has_large_weak_footprint(image, comp) != 0)
    {
        return;
    }

    min_area = (is_edge != 0 ||
                is_isolated_small_beacon(image, comp) != 0 ||
                (g_track_reinforced != 0 &&
                 matches_confirmed_beacon_track(comp) != 0)) ?
                    BEACON_EDGE_MIN_AREA :
                    BEACON_MIN_COMPONENT_AREA;
    if (comp->min_y < BEACON_EDGE_TOP_Y &&
        comp->area > BEACON_TOP_EDGE_MAX_AREA)
    {
        return;
    }
    if (is_side_edge != 0 && comp->area > BEACON_EDGE_MAX_AREA)
    {
        return;
    }
    if (comp->area < min_area)
    {
        return;
    }
    if (is_incomplete_border_component(comp) != 0 &&
        matches_confirmed_beacon_track(comp) == 0)
    {
        return;
    }
    if (is_component_in_lamp_core(comp, lamp) != 0 ||
        is_component_in_lamp_core(comp, temporal_lamp) != 0)
    {
        return;
    }
    if ((is_near_lamp(comp, lamp) != 0 ||
         is_near_lamp(comp, temporal_lamp) != 0) &&
        comp->area < LAMP_NEAR_BEACON_MIN_AREA &&
        is_isolated_near_lamp_beacon(image, comp) == 0)
    {
        return;
    }

    slot = result->beacon_count;
    if (slot >= BEACON_OUTPUT_MAX_COUNT)
    {
        slot = BEACON_OUTPUT_MAX_COUNT - 1;
        if ((float)comp->area <= result->beacons[slot].radius *
            result->beacons[slot].radius * PI_F)
        {
            return;
        }
    }
    else
    {
        result->beacon_count++;
    }

    for (i = slot - 1; i >= 0; i--)
    {
        if ((float)comp->area <= result->beacons[i].radius *
            result->beacons[i].radius * PI_F)
        {
            break;
        }
        result->beacons[i + 1] = result->beacons[i];
    }

    circle.x = (float)BEACON_IMAGE_W * 0.5f - comp->cx;
    circle.y = comp->cy - (float)BEACON_IMAGE_H * 0.5f;
    circle.radius = sqrtf((float)comp->area / PI_F);
    circle.valid = 1;
    result->beacons[i + 1] = circle;
}

static void sync_legacy_beacons(beacon_result_t *result)
{
    int i;
    int count = result->beacon_count;

    if (count > BEACON_MAX_CIRCLE_COUNT)
    {
        count = BEACON_MAX_CIRCLE_COUNT;
    }
    result->count = (unsigned char)count;
    for (i = 0; i < count; i++)
    {
        result->circles[i] = result->beacons[i];
    }
}

static void find_beacons(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *lamp,
    const component_t *temporal_lamp,
    beacon_result_t *result)
{
    unsigned char x;
    unsigned char y;

    result->beacon_count = 0;
    threshold_beacon_image(image);
    reinforce_tracked_beacon(image);
    erase_lamp_from_binary(lamp);
    erase_temporal_lamp_from_binary(temporal_lamp);

    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            component_t comp;

            if (is_component_seed(g_binary[y][x]) == 0 ||
                g_visit[y][x] != 0)
            {
                continue;
            }
            comp = grow_component(x, y);
            insert_beacon_by_area(image, &comp, lamp, temporal_lamp, result);
        }
    }
    sync_legacy_beacons(result);
}

static float square_distance(float ax, float ay, float bx, float by)
{
    float dx = ax - bx;
    float dy = ay - by;
    return dx * dx + dy * dy;
}

static float beacon_area(const beacon_circle_t *beacon)
{
    if (beacon == 0 || beacon->valid == 0)
    {
        return 0.0f;
    }
    return beacon->radius * beacon->radius * PI_F;
}

static void reset_track(temporal_track_t *track)
{
    memset(track, 0, sizeof(*track));
}

static void start_pending_beacon(temporal_track_t *track, const beacon_circle_t *beacon)
{
    reset_track(track);
    track->active = 1;
    track->hits = 1;
    track->x = beacon->x;
    track->y = beacon->y;
    track->radius = beacon->radius;
    track->area = beacon_area(beacon);
}

static void start_pending_car(temporal_track_t *track, const beacon_rect_t *car)
{
    reset_track(track);
    track->active = 1;
    track->hits = 1;
    track->x = car->cx;
    track->y = car->cy;
    track->width = car->width;
    track->length = car->length;
    track->angle = car->angle;
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
    track->misses = 0;
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
    track->misses = 0;
}

static void output_temporal_beacon(const temporal_track_t *track, beacon_result_t *result)
{
    result->temporal_beacons[0].x = track->x;
    result->temporal_beacons[0].y = track->y;
    result->temporal_beacons[0].radius = track->radius;
    result->temporal_beacons[0].valid = 1;
    result->temporal_beacon_count = 1;
}

static void output_temporal_car(const temporal_track_t *track, beacon_result_t *result)
{
    result->temporal_car_lamps[0].cx = track->x;
    result->temporal_car_lamps[0].cy = track->y;
    result->temporal_car_lamps[0].width = track->width;
    result->temporal_car_lamps[0].length = track->length;
    result->temporal_car_lamps[0].angle = track->angle;
    result->temporal_car_lamps[0].valid = 1;
    result->temporal_car_lamp_count = 1;
}

static unsigned char temporal_car_max_misses(const temporal_track_t *track)
{
    component_t lamp;

    if (component_from_temporal_car(track, &lamp, 1) == 0)
    {
        return CAR_LAMP_EDGE_MAX_MISSES;
    }
    if (lamp.min_x <= CAR_LAMP_TEMPORAL_EDGE_MARGIN ||
        lamp.min_y <= CAR_LAMP_TEMPORAL_EDGE_MARGIN ||
        lamp.max_x >= BEACON_IMAGE_W - 1 - CAR_LAMP_TEMPORAL_EDGE_MARGIN ||
        lamp.max_y >= BEACON_IMAGE_H - 1 - CAR_LAMP_TEMPORAL_EDGE_MARGIN)
    {
        return CAR_LAMP_EDGE_MAX_MISSES;
    }
    return CAR_LAMP_CENTER_MAX_MISSES;
}

static unsigned char can_use_temporal_car_mask(const temporal_track_t *track)
{
    return (track != 0 &&
            track->confirmed != 0 &&
            track->misses < temporal_car_max_misses(track)) ? 1 : 0;
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
    int x;
    int y;
    unsigned char found = 0;

    if (image == 0 || best_lamp == 0 ||
        can_use_temporal_car_mask(&g_car_track) == 0 ||
        component_from_temporal_car(&g_car_track, &temporal_lamp, 1) == 0)
    {
        return 0;
    }

    memset(&best_comp, 0, sizeof(best_comp));
    min_x = temporal_lamp.min_x - CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
    max_x = temporal_lamp.max_x + CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
    min_y = temporal_lamp.min_y - CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
    max_y = temporal_lamp.max_y + CAR_LAMP_TEMPORAL_TAKEOVER_PAD;
    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if (max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;

    threshold_image(image, BEACON_BINARY_THRESHOLD);
    for (y = min_y; y <= max_y; y++)
    {
        for (x = min_x; x <= max_x; x++)
        {
            component_t comp;

            if (is_component_seed(g_binary[y][x]) == 0 ||
                g_visit[y][x] != 0)
            {
                continue;
            }

            comp = grow_component((unsigned char)x, (unsigned char)y);
            if (comp.area >= CAR_LAMP_TEMPORAL_MIN_BRIGHT_AREA &&
                is_component_in_lamp_core(&comp, &temporal_lamp) != 0 &&
                (found == 0 || comp.area > best_comp.area))
            {
                best_comp = comp;
                found = 1;
            }
        }
    }

    if (found != 0)
    {
        *best_lamp = best_comp;
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

    if (count > BEACON_MAX_BEACON_COUNT)
    {
        count = BEACON_MAX_BEACON_COUNT;
    }
    for (i = 0; i < count; i++)
    {
        float d2;
        const beacon_circle_t *beacon = &result->beacons[i];
        if (beacon->valid == 0)
        {
            continue;
        }
        d2 = square_distance(beacon->x, beacon->y, x, y);
        if (d2 <= best_d2)
        {
            best_d2 = d2;
            best = i;
        }
    }
    return best;
}

static unsigned char is_unreliable_acquisition_region(
    const beacon_circle_t *beacon)
{
    int image_x;
    int image_y;
    adaptive_region_t region;

    if (beacon == 0 || beacon->valid == 0)
    {
        return 0;
    }
    if (g_adaptive_active == 0 && g_adaptive_enter_count == 0)
    {
        return 0;
    }
    image_x = (int)((float)BEACON_IMAGE_W * 0.5f - beacon->x + 0.5f);
    image_y = (int)((float)BEACON_IMAGE_H * 0.5f + beacon->y + 0.5f);
    image_x = clamp_int(image_x, 0, BEACON_IMAGE_W - 1);
    image_y = clamp_int(image_y, 0, BEACON_IMAGE_H - 1);
    region = adaptive_region_for_pixel(image_x, image_y);
    if (region == ADAPTIVE_REGION_CENTER)
    {
        return 0;
    }
    if (g_adaptive_active == 0)
    {
        return 1;
    }
    return (g_region_background[region] >= ADAPTIVE_ENTER_BACKGROUND) ?
               1 : 0;
}

static int first_reliable_beacon_index(
    const beacon_result_t *result,
    int count)
{
    int i;
    for (i = 0; i < count; i++)
    {
        if (result->beacons[i].valid != 0 &&
            is_unreliable_acquisition_region(&result->beacons[i]) == 0)
        {
            return i;
        }
    }
    return -1;
}

static void update_temporal_beacon(beacon_result_t *result)
{
    int count = result->beacon_count;
    int selected = -1;
    beacon_circle_t *measurement;
    const float gate = B0_MATCH_DISTANCE;

    result->temporal_beacon_count = 0;
    if (count > BEACON_MAX_BEACON_COUNT)
    {
        count = BEACON_MAX_BEACON_COUNT;
    }

    if (count <= 0)
    {
        if (g_b0_track.confirmed != 0 &&
            g_b0_track.misses < BEACON_MAX_MISSES)
        {
            g_b0_track.x += g_b0_track.vx;
            g_b0_track.y += g_b0_track.vy;
            g_b0_track.misses++;
            output_temporal_beacon(&g_b0_track, result);
            return;
        }
        reset_track(&g_b0_track);
        return;
    }

    if (g_b0_track.confirmed != 0)
    {
        float predict_x = g_b0_track.x + g_b0_track.vx;
        float predict_y = g_b0_track.y + g_b0_track.vy;
        selected = nearest_beacon_index(result, predict_x, predict_y, gate);
        if (selected > 0)
        {
            float b0_area = beacon_area(&result->beacons[0]);
            float selected_area = beacon_area(&result->beacons[selected]);
            float switch_ratio = B0_SWITCH_AREA_RATIO;
            if (b0_area < B0_SMALL_SWITCH_AREA &&
                selected_area < B0_SMALL_SWITCH_AREA)
            {
                switch_ratio = B0_SMALL_SWITCH_RATIO;
            }
            if (b0_area > selected_area * switch_ratio)
            {
                selected = 0;
            }
        }
    }
    else
    {
        selected = first_reliable_beacon_index(result, count);
    }

    if (selected < 0)
    {
        if (g_b0_track.confirmed != 0 &&
            g_b0_track.misses < BEACON_MAX_MISSES)
        {
            g_b0_track.x += g_b0_track.vx;
            g_b0_track.y += g_b0_track.vy;
            g_b0_track.misses++;
            output_temporal_beacon(&g_b0_track, result);
            return;
        }
        reset_track(&g_b0_track);
        return;
    }

    measurement = &result->beacons[selected];
    if (g_b0_track.active == 0)
    {
        start_pending_beacon(&g_b0_track, measurement);
        return;
    }

    if (g_b0_track.confirmed == 0)
    {
        if (square_distance(g_b0_track.x, g_b0_track.y,
                            measurement->x, measurement->y) >
            KALMAN_NEW_TARGET_DISTANCE * KALMAN_NEW_TARGET_DISTANCE)
        {
            start_pending_beacon(&g_b0_track, measurement);
            return;
        }
        update_beacon_track(&g_b0_track, measurement);
        g_b0_track.hits++;
        if (g_b0_track.hits >= B0_INIT_CONFIRM_FRAMES)
        {
            g_b0_track.confirmed = 1;
            output_temporal_beacon(&g_b0_track, result);
        }
        return;
    }

    if (square_distance(g_b0_track.x + g_b0_track.vx,
                        g_b0_track.y + g_b0_track.vy,
                        measurement->x,
                        measurement->y) >
        KALMAN_NEW_TARGET_DISTANCE * KALMAN_NEW_TARGET_DISTANCE)
    {
        start_pending_beacon(&g_b0_track, measurement);
        return;
    }

    update_beacon_track(&g_b0_track, measurement);
    output_temporal_beacon(&g_b0_track, result);
}

static void update_temporal_car(beacon_result_t *result)
{
    beacon_rect_t *measurement = 0;

    result->temporal_car_lamp_count = 0;
    if (result->car_lamp_count > 0 && result->car_lamps[0].valid != 0)
    {
        measurement = &result->car_lamps[0];
    }

    if (measurement == 0)
    {
        if (g_car_track.confirmed != 0 &&
            g_car_track.misses < temporal_car_max_misses(&g_car_track))
        {
            g_car_track.x += g_car_track.vx;
            g_car_track.y += g_car_track.vy;
            g_car_track.misses++;
            output_temporal_car(&g_car_track, result);
            return;
        }
        reset_track(&g_car_track);
        return;
    }

    if (g_car_track.active == 0)
    {
        start_pending_car(&g_car_track, measurement);
        return;
    }

    if (g_car_track.confirmed != 0 &&
        square_distance(g_car_track.x + g_car_track.vx,
                        g_car_track.y + g_car_track.vy,
                        measurement->cx, measurement->cy) >
        KALMAN_GATE_DISTANCE * KALMAN_GATE_DISTANCE)
    {
        start_pending_car(&g_car_track, measurement);
        return;
    }

    if (g_car_track.confirmed == 0 &&
        square_distance(g_car_track.x, g_car_track.y,
                        measurement->cx, measurement->cy) >
        KALMAN_NEW_TARGET_DISTANCE * KALMAN_NEW_TARGET_DISTANCE)
    {
        start_pending_car(&g_car_track, measurement);
        return;
    }

    update_car_track(&g_car_track, measurement);
    g_car_track.hits++;
    if (g_car_track.confirmed == 0 &&
        g_car_track.hits >= B0_INIT_CONFIRM_FRAMES)
    {
        g_car_track.confirmed = 1;
    }
    if (g_car_track.confirmed != 0)
    {
        output_temporal_car(&g_car_track, result);
    }
}

static void update_temporal_result(beacon_result_t *result)
{
    update_temporal_beacon(result);
    update_temporal_car(result);
}

void beacon_image_process(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result)
{
    component_t lamp;
    component_t temporal_lamp;
    unsigned char has_lamp;

    if (result == 0)
    {
        return;
    }

    clear_result(result);
    if (image == 0)
    {
        return;
    }

    update_adaptive_background(image);
    memset(&temporal_lamp, 0, sizeof(temporal_lamp));
    threshold_car_lamp_image(image);
    has_lamp = find_car_lamp(image, &lamp);
    if (has_lamp == 0)
    {
        has_lamp = find_temporal_car_lamp(image, &lamp);
    }
    if (has_lamp == 0)
    {
        memset(&lamp, 0, sizeof(lamp));
    }
    if (can_use_temporal_car_mask(&g_car_track) != 0)
    {
        (void)component_from_temporal_car(&g_car_track, &temporal_lamp, 1);
    }
    write_car_lamp(&lamp, result);
    find_beacons(image, &lamp, &temporal_lamp, result);
    update_temporal_result(result);
}

unsigned char beacon_image_debug_threshold(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    (void)image;
    return CAR_LAMP_BINARY_THRESHOLD;
}

void beacon_image_debug_binary(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char binary[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    if (binary == 0)
    {
        return;
    }
    if (image == 0)
    {
        memset(binary, 0, BEACON_IMAGE_W * BEACON_IMAGE_H);
        return;
    }
    threshold_car_lamp_image(image);
    memcpy(binary, g_binary, sizeof(g_binary));
}
