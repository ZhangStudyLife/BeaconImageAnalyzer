#include "beacon_image.h"
#include "beacon_image_config.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

typedef uint8_t uint8;

#define BEACON_OUTPUT_MAX_COUNT 3
#define CAR_LAMP_OUTPUT_MAX_COUNT 1

static float beacon_area(const beacon_circle_t *beacon);

#define CAR_LAMP_BINARY_THRESHOLD 200
#define CAR_LAMP_UPPER_THRESHOLD  150
#define CAR_LAMP_UPPER_Y          64
#define CAR_LAMP_BRIDGE_MAX_GAP   4
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
#define COMPONENT_TOP_REJECT_Y    0
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
#define BEACON_TINY_LINE_MAX_AREA  3
#define BEACON_TINY_LINE_MAX_MINOR 1.05f
#define BEACON_LINEAR_REFLECTION_MIN_AREA 12
#define BEACON_LINEAR_REFLECTION_MIN_MAJOR 13.0f
#define BEACON_LINEAR_REFLECTION_MAX_MINOR 3.8f
#define BEACON_LINEAR_REFLECTION_MIN_ELONGATION 3.5f
#define BEACON_TOP_LEFT_GLARE_MAX_X 35
#define BEACON_TOP_LEFT_GLARE_MAX_Y 32
#define BEACON_TOP_LEFT_GLARE_MIN_AREA 20
#define BEACON_TOP_LEFT_GLARE_MIN_ASPECT 2
#define LAMP_MASK_PAD             2
#define LAMP_MASK_DOWN_PAD        6
#define LAMP_NEAR_BEACON_PAD      8
#define LAMP_NEAR_BEACON_MIN_AREA 21
#define LAMP_NEAR_BEACON_ISOLATED_MIN_AREA 3
#define LAMP_NEAR_BEACON_BACKGROUND_MAX 20
#define LAMP_NEAR_BEACON_GRAY_MIN 150
#define BEACON_MIN_COMPONENT_AREA 6
#define CAR_LAMP_MIN_AREA 40
#define CAR_LAMP_MAX_AREA         100
#define CAR_LAMP_FRONT_MAX_AREA   180
#define CAR_LAMP_FRONT_LARGE_MIN_ELONGATION 3.0f
#define CAR_LAMP_FRONT_LARGE_MIN_LENGTH 20.0f
#define CAR_LAMP_MIN_ELONGATION   1.6f
#define CAR_LAMP_MIN_LENGTH       12.0f
#define CAR_LAMP_FRONT_MIN_LENGTH 10.0f
#define CAR_LAMP_MIN_CENTER_Y     (BEACON_IMAGE_H / 3)
#define CAR_LAMP_Y_PRIORITY_MARGIN 3.0f
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
#define IMAGE_QUEUE_SIZE          (BEACON_IMAGE_W * BEACON_IMAGE_H)
#define PI_F                      3.1415926f
#define BEACON_AREA_GATE_X_MIN    (-110.0f) /* 信标面积查表的最小中心横坐标 */
#define BEACON_AREA_GATE_Y_MIN    (-75.0f)  /* 信标面积查表的最小中心纵坐标 */
#define BEACON_AREA_GATE_X_STEP   (25.0f)   /* 信标面积查表的横向网格宽度，单位像素 */
#define BEACON_AREA_GATE_Y_STEP   (20.0f)   /* 信标面积查表的纵向网格高度，单位像素 */
#define BEACON_AREA_GATE_X_COUNT  (9U)      /* 信标面积查表的横向网格数量 */
#define BEACON_AREA_GATE_Y_COUNT  (7U)      /* 信标面积查表的纵向网格数量 */
#define BEACON_AREA_GATE_FRONT_ID (0U)      /* 前摄2BL3图像板编号 */
#define BEACON_AREA_GATE_BACK_ID  (1U)      /* 后摄2BL3图像板编号 */
#define BEACON_AREA_GATE_UNKNOWN  (0xFFU)   /* 尚未收到图像板编号时的无效值 */

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

static unsigned char g_binary[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_visit[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_queue_x[IMAGE_QUEUE_SIZE];
static unsigned char g_queue_y[IMAGE_QUEUE_SIZE];
static temporal_track_t g_b0_track;
static temporal_track_t g_car_track;
static unsigned char g_track_reinforced;
static unsigned char g_temporal_car_hard_rejected;
static uint8 g_camera_board_id = BEACON_DEFAULT_CAMERA_BOARD_ID; /* 当前图像板编号，用于选择前摄或后摄面积表 */

/* 前摄跨飞行保守面积上限，0表示该坐标格样本不足并保持原排序。 */
static const uint8 g_front_beacon_area_upper[BEACON_AREA_GATE_Y_COUNT][BEACON_AREA_GATE_X_COUNT] =
{
    {  0U,   0U,   0U,   0U,  18U,   0U,   0U,   0U,   0U},
    {  0U,   0U,   0U,  24U,  19U,  20U,   0U,   0U,   0U},
    {  0U,   0U,  27U,  42U,  40U,  31U,  25U,   0U,   0U},
    {  0U,   0U,  56U,  60U,  62U,  55U,  25U,  19U,   0U},
    {  0U,  60U,  80U, 113U, 126U, 116U, 157U,  28U,   0U},
    {  0U,  52U,  91U, 155U, 159U, 143U, 162U,  35U,   0U},
    {  0U,  68U, 105U, 172U, 198U, 160U,  76U,  45U,   0U}
};

/* 前摄信标面积下限，0表示该坐标网格不启用下限过滤。 */
static const uint8 g_front_beacon_area_lower[BEACON_AREA_GATE_Y_COUNT][BEACON_AREA_GATE_X_COUNT] =
{
    { 0U,  0U,  0U,  0U,  3U,  0U,  0U,  0U, 0U},
    { 0U,  0U,  0U,  4U,  3U,  5U,  0U,  0U, 0U},
    { 0U,  7U,  2U,  7U,  9U,  7U,  8U,  7U, 0U},
    { 0U,  0U,  8U, 15U, 16U, 12U, 10U,  3U, 0U},
    { 8U, 15U, 32U, 36U, 41U, 38U, 14U, 10U, 0U},
    {16U, 20U, 34U, 55U, 55U, 48U, 19U, 12U, 0U},
    { 0U, 25U, 39U, 68U, 70U, 63U, 28U, 15U, 0U}
};

/* 后摄跨飞行保守面积上限，0表示该坐标格样本不足并保持原排序。 */
static const uint8 g_back_beacon_area_upper[BEACON_AREA_GATE_Y_COUNT][BEACON_AREA_GATE_X_COUNT] =
{
    {  0U,  37U,  22U,   0U,  18U,  38U,  20U,  18U,   0U},
    { 15U,  20U,  13U,  16U,  15U,  22U,  20U,  16U,   0U},
    {  6U,  14U,  14U,  30U,  29U,  29U,  20U,   4U,   8U},
    {  9U,  18U,  27U,  58U,  54U,  48U,  23U,  14U,   0U},
    { 18U,  27U,  54U,  98U,  96U,  85U,  43U,  22U,   0U},
    { 29U,  37U,  96U, 154U, 148U, 136U,  87U,  30U,  23U},
    { 27U,  52U, 143U, 232U, 190U, 163U,  96U,  32U,   0U}
};

/* 后摄信标面积下限，0表示该坐标网格不启用下限过滤。 */
static const uint8 g_back_beacon_area_lower[BEACON_AREA_GATE_Y_COUNT][BEACON_AREA_GATE_X_COUNT] =
{
    {0U,  4U,  4U,  0U,  3U,  5U,  5U, 4U, 0U},
    {1U,  4U,  1U,  2U,  3U,  7U,  6U, 2U, 0U},
    {1U,  1U,  3U,  4U,  4U,  8U,  5U, 1U, 1U},
    {1U,  4U,  4U, 12U, 13U, 11U,  6U, 1U, 0U},
    {3U,  7U, 14U, 22U, 28U, 23U, 12U, 5U, 0U},
    {4U,  9U, 29U, 42U, 45U, 39U, 23U, 8U, 8U},
    {5U, 12U, 32U, 69U, 60U, 48U, 31U, 5U, 0U}
};

int g_beacon_binary_threshold = BEACON_BINARY_THRESHOLD_DEFAULT;

void beacon_image_init(void)
{
    memset(g_binary, 0, sizeof(g_binary));
    memset(g_visit, 0, sizeof(g_visit));
}

void beacon_image_reset_temporal(void)
{
    memset(&g_b0_track, 0, sizeof(g_b0_track));
    memset(&g_car_track, 0, sizeof(g_car_track));
    g_temporal_car_hard_rejected = 0U;
}

static void clear_result(beacon_result_t *result)
{
    if(result != 0)
    {
        memset(result, 0, sizeof(*result));
    }
}

static void threshold_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char threshold)
{
    int i;
    const unsigned char *src = &image[0][0];
    unsigned char *dst = &g_binary[0][0];

    for(i = 0; i < BEACON_IMAGE_W * BEACON_IMAGE_H; i++)
    {
        dst[i] = (src[i] >= threshold) ? 255 : 0;
    }
    memset(g_visit, 0, sizeof(g_visit));
}

static void bridge_upper_car_lamp_gaps(void)
{
    int y;
    int limit_y = CAR_LAMP_UPPER_Y;

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

static void threshold_car_lamp_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    int x;
    int y;

    for(y = 0; y < BEACON_IMAGE_H; y++)
    {
        unsigned char threshold = CAR_LAMP_BINARY_THRESHOLD;

        if(y < CAR_LAMP_UPPER_Y)
        {
            threshold = CAR_LAMP_UPPER_THRESHOLD;
        }

        for(x = 0; x < BEACON_IMAGE_W; x++)
        {
            g_binary[y][x] = (image[y][x] >= threshold) ? 255 : 0;
        }
    }
    bridge_upper_car_lamp_gaps();
    memset(g_visit, 0, sizeof(g_visit));
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
    memset(g_visit, 0, sizeof(g_visit));
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

    while(head < tail)
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

        if((int)x < comp.min_x) comp.min_x = x;
        if((int)x > comp.max_x) comp.max_x = x;
        if((int)y < comp.min_y) comp.min_y = y;
        if((int)y > comp.max_y) comp.max_y = y;

        for(i = 0; i < 8; i++)
        {
            int nx = (int)x + dx[i];
            int ny = (int)y + dy[i];

            if((nx < 0) || (nx >= BEACON_IMAGE_W) ||
               (ny < 0) || (ny >= BEACON_IMAGE_H))
            {
                continue;
            }
            if((g_binary[ny][nx] == 0) || (g_visit[ny][nx] != 0) ||
               (tail >= IMAGE_QUEUE_SIZE))
            {
                continue;
            }

            g_visit[ny][nx] = 1;
            g_queue_x[tail] = (unsigned char)nx;
            g_queue_y[tail] = (unsigned char)ny;
            tail++;
        }
    }

    if(comp.area > 0)
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
        if(disc < 0.0f) disc = 0.0f;
        eig_major = trace * 0.5f + sqrtf(disc);
        eig_minor = trace * 0.5f - sqrtf(disc);
        if(eig_minor < 0.0f) eig_minor = 0.0f;

        comp.major = 4.0f * sqrtf(eig_major + 0.0001f);
        comp.minor = 4.0f * sqrtf(eig_minor + 0.0001f);
        if(comp.minor < 1.0f) comp.minor = 1.0f;
        comp.elongation = comp.major / comp.minor;
        comp.angle = 0.5f * atan2f(2.0f * cov_xy, var_x - var_y) *
                     180.0f / PI_F;
        comp.valid = 1;
    }

    return comp;
}

static unsigned char is_lamp_candidate(const component_t *comp)
{
    float min_length = (g_camera_board_id == BEACON_AREA_GATE_FRONT_ID) ?
                           CAR_LAMP_FRONT_MIN_LENGTH : CAR_LAMP_MIN_LENGTH;
    unsigned char area_allowed;

    if(comp == 0)
    {
        return 0U;
    }
    area_allowed = ((comp->area <= CAR_LAMP_MAX_AREA) ||
                    ((g_camera_board_id == BEACON_AREA_GATE_FRONT_ID) &&
                     (comp->area <= CAR_LAMP_FRONT_MAX_AREA) &&
                     (comp->elongation >= CAR_LAMP_FRONT_LARGE_MIN_ELONGATION) &&
                     (comp->major >= CAR_LAMP_FRONT_LARGE_MIN_LENGTH))) ? 1U : 0U;

    return ((comp->valid != 0) &&
            (comp->min_y > COMPONENT_TOP_REJECT_Y) &&
            (comp->max_y < COMPONENT_BOTTOM_REJECT_Y) &&
            (comp->cy >= (float)CAR_LAMP_MIN_CENTER_Y) &&
            (comp->area >= CAR_LAMP_MIN_AREA) &&
            (area_allowed != 0U) &&
            (comp->elongation >= CAR_LAMP_MIN_ELONGATION) &&
            (comp->major >= min_length)) ? 1 : 0;
}

static unsigned char find_car_lamp(component_t *best_lamp)
{
    unsigned char x;
    unsigned char y;
    float best_score = 0.0f;
    unsigned char found = 0;

    memset(best_lamp, 0, sizeof(*best_lamp));

    for(y = 0; y < BEACON_IMAGE_H; y++)
    {
        for(x = 0; x < BEACON_IMAGE_W; x++)
        {
            component_t comp;
            float score;

            if((g_binary[y][x] == 0) || (g_visit[y][x] != 0))
            {
                continue;
            }

            comp = grow_component(x, y);
            if(is_lamp_candidate(&comp) == 0)
            {
                continue;
            }

            score = (float)comp.area * comp.elongation;
            if((found == 0) ||
               (comp.cy > best_lamp->cy + CAR_LAMP_Y_PRIORITY_MARGIN) ||
               ((fabsf(comp.cy - best_lamp->cy) <=
                 CAR_LAMP_Y_PRIORITY_MARGIN) &&
                (score > best_score)))
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

    if((lamp == 0) || (lamp->valid == 0))
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
        for(x = lamp->min_x; x <= lamp->max_x; x++)
        {
            float dx = (float)x - lamp->cx;
            float dy = (float)y - lamp->cy;
            float major = dx * cos_a + dy * sin_a;
            float minor = -dx * sin_a + dy * cos_a;

            if((fabsf(major) <= half_len) && (fabsf(minor) <= half_wid))
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

static unsigned char is_obvious_linear_reflection(const component_t *comp)
{
    int width;
    int height;

    if((comp == 0) || (comp->valid == 0U))
    {
        return 0U;
    }
    if(g_camera_board_id != BEACON_AREA_GATE_FRONT_ID)
    {
        return 0U;
    }
    width = comp->max_x - comp->min_x + 1;
    height = comp->max_y - comp->min_y + 1;
    if((comp->max_x < BEACON_TOP_LEFT_GLARE_MAX_X) &&
       (comp->max_y < BEACON_TOP_LEFT_GLARE_MAX_Y) &&
       (comp->area >= BEACON_TOP_LEFT_GLARE_MIN_AREA) &&
       (height >= width * BEACON_TOP_LEFT_GLARE_MIN_ASPECT))
    {
        return 1U;
    }
    if((is_edge_beacon_area(comp) != 0U) &&
       (comp->area <= BEACON_TINY_LINE_MAX_AREA) &&
       (comp->minor <= BEACON_TINY_LINE_MAX_MINOR) &&
       (matches_confirmed_beacon_track(comp) == 0U))
    {
        return 1U;
    }
    return ((comp->area >= BEACON_LINEAR_REFLECTION_MIN_AREA) &&
            (comp->major >= BEACON_LINEAR_REFLECTION_MIN_MAJOR) &&
            (comp->minor <= BEACON_LINEAR_REFLECTION_MAX_MINOR) &&
            (comp->elongation >= BEACON_LINEAR_REFLECTION_MIN_ELONGATION)) ? 1U : 0U;
}

/**
 * @brief 根据前摄或后摄坐标查表，拒绝面积超过标定上限的连通域。
 * @return 1表示允许；板号无效、坐标越界或网格值为0时保持原规则。
 */
static unsigned char is_beacon_area_allowed(float x, float y, float area)
{
    int x_index;
    int y_index;
    uint8 lower;
    uint8 upper;

    if((g_camera_board_id != BEACON_AREA_GATE_FRONT_ID) &&
       (g_camera_board_id != BEACON_AREA_GATE_BACK_ID))
    {
        return 1U;
    }
    if((x < BEACON_AREA_GATE_X_MIN) ||
       (x >= BEACON_AREA_GATE_X_MIN +
             BEACON_AREA_GATE_X_STEP * (float)BEACON_AREA_GATE_X_COUNT) ||
       (y < BEACON_AREA_GATE_Y_MIN) ||
       (y >= BEACON_AREA_GATE_Y_MIN +
             BEACON_AREA_GATE_Y_STEP * (float)BEACON_AREA_GATE_Y_COUNT))
    {
        return 1U;
    }
    x_index = (int)((x - BEACON_AREA_GATE_X_MIN) / BEACON_AREA_GATE_X_STEP);
    y_index = (int)((y - BEACON_AREA_GATE_Y_MIN) / BEACON_AREA_GATE_Y_STEP);
    if((x_index < 0) || (x_index >= (int)BEACON_AREA_GATE_X_COUNT) ||
       (y_index < 0) || (y_index >= (int)BEACON_AREA_GATE_Y_COUNT))
    {
        return 1U;
    }

    lower = (g_camera_board_id == BEACON_AREA_GATE_FRONT_ID) ?
                g_front_beacon_area_lower[y_index][x_index] :
                g_back_beacon_area_lower[y_index][x_index];
    upper = (g_camera_board_id == BEACON_AREA_GATE_FRONT_ID) ?
                g_front_beacon_area_upper[y_index][x_index] :
                g_back_beacon_area_upper[y_index][x_index];
    if((lower != 0U) && (area < (float)lower))
    {
        return 0U;
    }
    if((upper != 0U) && (area > (float)upper))
    {
        return 0U;
    }
    return 1U;
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
    min_x = cx - BEACON_LOCAL_RING_OUTER;
    max_x = cx + BEACON_LOCAL_RING_OUTER;
    min_y = cy - BEACON_LOCAL_RING_OUTER;
    max_y = cy + BEACON_LOCAL_RING_OUTER;
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
    unsigned char max_gray = 0;

    if((image == 0) || (comp == 0) || (comp->valid == 0))
    {
        return 0;
    }

    for(y = comp->min_y; y <= comp->max_y; y++)
    {
        for(x = comp->min_x; x <= comp->max_x; x++)
        {
            if((g_binary[y][x] != 0) && (image[y][x] > max_gray))
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
    if(is_obvious_linear_reflection(comp) != 0U)
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
    if(((((comp->min_y <= BEACON_TOP_WEAK_Y) &&
           (comp->area <= BEACON_TOP_WEAK_AREA_MAX)) ||
          ((comp->min_y < BEACON_EDGE_TOP_Y) &&
           (comp->area >= BEACON_TOP_DIFFUSE_AREA_MIN) &&
           (comp->area <= BEACON_TOP_EDGE_MAX_AREA) &&
           (matches_confirmed_beacon_track(comp) == 0U)) ||
          ((is_side_edge != 0U) &&
           (comp->area >= BEACON_SIDE_DIFFUSE_AREA_MIN) &&
           (comp->area <= BEACON_EDGE_MAX_AREA))) &&
         (component_max_gray(image, comp) < BEACON_TOP_WEAK_GRAY_MIN)))
    {
        return;
    }
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

    circle.x = comp->cx - (float)BEACON_IMAGE_W * 0.5f;
    circle.y = comp->cy - (float)BEACON_IMAGE_H * 0.5f;
    circle.radius = sqrtf((float)comp->area / PI_F);
    circle.valid = 1;
    if(is_beacon_area_allowed(circle.x, circle.y, (float)comp->area) == 0U)
    {
        return;
    }

    slot = result->beacon_count;
    if(slot >= BEACON_OUTPUT_MAX_COUNT)
    {
        slot = BEACON_OUTPUT_MAX_COUNT - 1;
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
    result->beacons[i + 1] = circle;
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

    for(y = 0; y < BEACON_IMAGE_H; y++)
    {
        for(x = 0; x < BEACON_IMAGE_W; x++)
        {
            component_t comp;

            if((g_binary[y][x] == 0) || (g_visit[y][x] != 0))
            {
                continue;
            }

            comp = grow_component(x, y);
            insert_beacon_by_area(image, &comp, lamp, temporal_lamp, result);
        }
    }
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

    return beacon->radius * beacon->radius * PI_F;
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
    track->misses = 0U;
}

static void write_temporal_beacon(
    const temporal_track_t *track,
    beacon_result_t *result,
    int matched_index)
{
    (void)matched_index;
    result->temporal_beacons[0].x = track->x;
    result->temporal_beacons[0].y = track->y;
    result->temporal_beacons[0].radius = track->radius;
    result->temporal_beacons[0].valid = 1U;
    result->temporal_beacon_count = 1U;
}

static void write_temporal_car(const temporal_track_t *track, beacon_result_t *result)
{
    result->temporal_car_lamps[0].cx = track->x;
    result->temporal_car_lamps[0].cy = track->y;
    result->temporal_car_lamps[0].width = track->width;
    result->temporal_car_lamps[0].length = track->length;
    result->temporal_car_lamps[0].angle = track->angle;
    result->temporal_car_lamps[0].valid = 1U;
    result->temporal_car_lamp_count = 1U;
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

    threshold_image(image, (unsigned char)g_beacon_binary_threshold);
    for(y = min_y; y <= max_y; y++)
    {
        for(x = min_x; x <= max_x; x++)
        {
            component_t comp;

            if((g_binary[y][x] == 0) || (g_visit[y][x] != 0))
            {
                continue;
            }

            comp = grow_component((unsigned char)x, (unsigned char)y);
            if(is_component_in_lamp_core(&comp, &temporal_lamp) == 0)
            {
                continue;
            }
            if(comp.area > ((g_camera_board_id == BEACON_AREA_GATE_FRONT_ID) ?
                            CAR_LAMP_FRONT_MAX_AREA : CAR_LAMP_MAX_AREA))
            {
                g_temporal_car_hard_rejected = 1U;
                continue;
            }
            if(comp.cy < (float)CAR_LAMP_MIN_CENTER_Y)
            {
                g_temporal_car_hard_rejected = 1U;
                continue;
            }
            if((comp.area >= CAR_LAMP_TEMPORAL_MIN_BRIGHT_AREA) &&
               ((found == 0) || (comp.area > best_comp.area)))
            {
                best_comp = comp;
                found = 1;
            }
        }
    }

    if(found != 0)
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

    if(count > BEACON_OUTPUT_MAX_COUNT)
    {
        count = BEACON_OUTPUT_MAX_COUNT;
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

    result->temporal_beacon_count = 0U;

    if(count > BEACON_OUTPUT_MAX_COUNT)
    {
        count = BEACON_OUTPUT_MAX_COUNT;
    }

    if(count <= 0)
    {
        if((g_b0_track.confirmed != 0U) &&
           (g_b0_track.misses < BEACON_MAX_MISSES))
        {
            g_b0_track.x += g_b0_track.vx;
            g_b0_track.y += g_b0_track.vy;
            g_b0_track.misses++;
            write_temporal_beacon(&g_b0_track, result, -1);
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
        if((g_b0_track.confirmed != 0U) &&
           (g_b0_track.misses < BEACON_MAX_MISSES))
        {
            g_b0_track.x += g_b0_track.vx;
            g_b0_track.y += g_b0_track.vy;
            g_b0_track.misses++;
            write_temporal_beacon(&g_b0_track, result, -1);
            return;
        }
        reset_track(&g_b0_track);
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

    result->temporal_car_lamp_count = 0U;

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
            g_car_track.x += g_car_track.vx;
            g_car_track.y += g_car_track.vy;
            g_car_track.misses++;
            write_temporal_car(&g_car_track, result);
            return;
        }
        reset_track(&g_car_track);
        return;
    }

    if(g_car_track.active == 0U)
    {
        start_pending_car(&g_car_track, measurement);
        return;
    }

    if((g_car_track.confirmed != 0U) &&
       (square_distance(g_car_track.x + g_car_track.vx,
                        g_car_track.y + g_car_track.vy,
                        measurement->cx, measurement->cy) >
        KALMAN_GATE_DISTANCE * KALMAN_GATE_DISTANCE))
    {
        start_pending_car(&g_car_track, measurement);
        return;
    }

    if((g_car_track.confirmed == 0U) &&
       (square_distance(g_car_track.x, g_car_track.y,
                        measurement->cx, measurement->cy) >
        KALMAN_NEW_TARGET_DISTANCE * KALMAN_NEW_TARGET_DISTANCE))
    {
        start_pending_car(&g_car_track, measurement);
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
        write_temporal_car(&g_car_track, result);
    }
}

static void update_temporal_result(beacon_result_t *result)
{
    update_temporal_beacon(result);
    update_temporal_car(result);
}

static void convert_result_to_analyzer_coordinates(beacon_result_t *result)
{
    int i;

    for(i = 0; i < BEACON_MAX_CIRCLE_COUNT; i++)
    {
        result->circles[i].x = -result->circles[i].x;
    }
    for(i = 0; i < BEACON_MAX_BEACON_COUNT; i++)
    {
        result->beacons[i].x = -result->beacons[i].x;
        result->temporal_beacons[i].x = -result->temporal_beacons[i].x;
    }
    for(i = 0; i < BEACON_MAX_CAR_LAMP_COUNT; i++)
    {
        result->car_lamps[i].cx = -result->car_lamps[i].cx;
        result->temporal_car_lamps[i].cx = -result->temporal_car_lamps[i].cx;
    }
}

void beacon_image_process(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result)
{
    component_t lamp;
    component_t temporal_lamp;
    unsigned char has_lamp;

    g_temporal_car_hard_rejected = 0U;

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

    memset(&temporal_lamp, 0, sizeof(temporal_lamp));
    threshold_car_lamp_image(image);
    has_lamp = find_car_lamp(&lamp);
    if(has_lamp == 0)
    {
        threshold_image(image, CAR_LAMP_BINARY_THRESHOLD);
        has_lamp = find_car_lamp(&lamp);
    }
    if(has_lamp == 0)
    {
        has_lamp = find_temporal_car_lamp(image, &lamp);
    }
    if((has_lamp == 0) && (g_temporal_car_hard_rejected != 0U))
    {
        reset_track(&g_car_track);
    }
    if(has_lamp == 0)
    {
        memset(&lamp, 0, sizeof(lamp));
    }
    if((has_lamp == 0) &&
       (can_use_temporal_car_mask(&g_car_track) != 0))
    {
        (void)component_from_temporal_car(&g_car_track, &temporal_lamp, 1U);
    }
    write_car_lamp(&lamp, result);
    find_beacons(image, &lamp, &temporal_lamp, result);
    update_temporal_result(result);
    convert_result_to_analyzer_coordinates(result);
}

void beacon_image_set_camera_board_id(unsigned char board_id)
{
    g_camera_board_id = (board_id <= BEACON_AREA_GATE_BACK_ID) ?
                        board_id : BEACON_AREA_GATE_UNKNOWN;
    beacon_image_reset_temporal();
}

unsigned char beacon_image_get_camera_board_id(void)
{
    return g_camera_board_id;
}

unsigned char beacon_image_debug_threshold(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    (void)image;
    return (unsigned char)g_beacon_binary_threshold;
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
