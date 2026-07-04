#include "beacon_image.h"
#include "beacon_image_config.h"

#include <math.h>
#include <string.h>

#define CAR_LAMP_BINARY_THRESHOLD       200
#define BEACON_BINARY_THRESHOLD         120
#define LAMP_MASK_PAD                   2
#define LAMP_NEAR_BEACON_PAD            8
#define LAMP_NEAR_BEACON_MIN_AREA       21
#define BEACON_SIDE_EDGE_MARGIN         25
#define BEACON_SIDE_EDGE_MIN_AREA       3
#define BEACON_SIDE_EDGE_THRESHOLD      100
#define CAR_LAMP_MIN_AREA               24
#define CAR_LAMP_MAX_AREA               1200
#define CAR_LAMP_MIN_ELONGATION         1.6f
#define CAR_LAMP_MIN_LENGTH             8.0f
#define CAR_LAMP_TRACK_START_AREA       45
#define CAR_LAMP_TRACK_START_ELONGATION 2.0f
#define CAR_LAMP_TRACK_START_SCORE      120.0f
#define CAR_LAMP_LARGE_AREA             90
#define CAR_LAMP_LARGE_ELONGATION       1.2f
#define CAR_LAMP_STRONG_AREA            45
#define CAR_LAMP_SMALL_MIN_SPAN         12
#define CAR_LAMP_SMALL_BACKGROUND_MAX   60
#define CAR_LAMP_EDGE_BACKGROUND_MAX    80
#define CAR_LAMP_SIDE_SUN_BACKGROUND_MAX 70
#define CAR_LAMP_SIDE_SUN_MARGIN        16
#define CAR_LAMP_SIDE_SUN_Y             36
#define CAR_LAMP_EDGE_MARGIN            2
#define CAR_LAMP_LOCAL_RING_PAD         3

#define IMAGE_QUEUE_SIZE                (BEACON_IMAGE_W * BEACON_IMAGE_H)
#define PI_F                            3.1415926f

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

static unsigned char g_binary[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_visit_stamp[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_current_stamp = 0;
static unsigned char g_queue_x[IMAGE_QUEUE_SIZE];
static unsigned char g_queue_y[IMAGE_QUEUE_SIZE];
static const unsigned char (*g_current_image)[BEACON_IMAGE_W] = 0;
static unsigned char g_has_lamp_track = 0;

void beacon_image_init(void)
{
    memset(g_binary, 0, sizeof(g_binary));
    memset(g_visit_stamp, 0, sizeof(g_visit_stamp));
    g_current_image = 0;
    g_has_lamp_track = 0;
    g_current_stamp = 0;
}

static void clear_result(beacon_result_t *result)
{
    if (result != 0)
    {
        memset(result, 0, sizeof(*result));
    }
}

static void begin_visit_pass(void)
{
    g_current_stamp++;
    if (g_current_stamp == 0)
    {
        memset(g_visit_stamp, 0, sizeof(g_visit_stamp));
        g_current_stamp = 1;
    }
}

static unsigned char is_visited(unsigned char x, unsigned char y)
{
    return (g_visit_stamp[y][x] == g_current_stamp) ? 1 : 0;
}

static void mark_visited(unsigned char x, unsigned char y)
{
    g_visit_stamp[y][x] = g_current_stamp;
}

static void threshold_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char threshold)
{
    const unsigned char *src = &image[0][0];
    unsigned char *dst = &g_binary[0][0];
    int i;

    for (i = 0; i < BEACON_IMAGE_W * BEACON_IMAGE_H; i++)
    {
        dst[i] = (src[i] >= threshold) ? 255 : 0;
    }
}

static void threshold_beacon_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    unsigned char x;
    unsigned char y;

    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            unsigned char threshold =
                (x < BEACON_SIDE_EDGE_MARGIN ||
                 x >= BEACON_IMAGE_W - BEACON_SIDE_EDGE_MARGIN)
                    ? BEACON_SIDE_EDGE_THRESHOLD
                    : BEACON_BINARY_THRESHOLD;
            g_binary[y][x] = (image[y][x] >= threshold) ? 255 : 0;
        }
    }
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
    mark_visited(start_x, start_y);

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
            if (g_binary[ny][nx] == 0 || is_visited((unsigned char)nx, (unsigned char)ny))
            {
                continue;
            }
            if (tail >= IMAGE_QUEUE_SIZE)
            {
                continue;
            }

            mark_visited((unsigned char)nx, (unsigned char)ny);
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
        if (disc < 0.0f)
        {
            disc = 0.0f;
        }

        eig_major = trace * 0.5f + sqrtf(disc);
        eig_minor = trace * 0.5f - sqrtf(disc);
        if (eig_minor < 0.0f)
        {
            eig_minor = 0.0f;
        }

        comp.major = 4.0f * sqrtf(eig_major + 0.0001f);
        comp.minor = 4.0f * sqrtf(eig_minor + 0.0001f);
        if (comp.minor < 1.0f)
        {
            comp.minor = 1.0f;
        }
        comp.elongation = comp.major / comp.minor;
        comp.angle = 0.5f * atan2f(2.0f * cov_xy, var_x - var_y) * 180.0f / PI_F;
        comp.valid = 1;
    }

    return comp;
}

static int local_background_average(const component_t *comp, int pad)
{
    int min_x = comp->min_x - pad;
    int max_x = comp->max_x + pad;
    int min_y = comp->min_y - pad;
    int max_y = comp->max_y + pad;
    int x;
    int y;
    int sum = 0;
    int count = 0;

    if (g_current_image == 0)
    {
        return 0;
    }
    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if (max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;

    for (y = min_y; y <= max_y; y++)
    {
        for (x = min_x; x <= max_x; x++)
        {
            if (x >= comp->min_x && x <= comp->max_x &&
                y >= comp->min_y && y <= comp->max_y)
            {
                continue;
            }
            sum += g_current_image[y][x];
            count++;
        }
    }

    if (count == 0)
    {
        return 0;
    }
    return sum / count;
}

static unsigned char is_lamp_candidate(const component_t *comp)
{
    unsigned char touches_top_or_bottom;
    unsigned char touches_left_or_right;
    int bbox_w = comp->max_x - comp->min_x + 1;
    int bbox_h = comp->max_y - comp->min_y + 1;
    int bbox_span = bbox_w > bbox_h ? bbox_w : bbox_h;

    if (comp->area > CAR_LAMP_MAX_AREA)
    {
        return 0;
    }
    if (g_has_lamp_track == 0 &&
        (comp->area < CAR_LAMP_TRACK_START_AREA ||
         comp->elongation < CAR_LAMP_TRACK_START_ELONGATION ||
         (float)comp->area * comp->elongation < CAR_LAMP_TRACK_START_SCORE))
    {
        return 0;
    }

    touches_top_or_bottom =
        (comp->min_y <= CAR_LAMP_EDGE_MARGIN ||
         comp->max_y >= BEACON_IMAGE_H - 1 - CAR_LAMP_EDGE_MARGIN) ? 1 : 0;
    touches_left_or_right =
        (comp->min_x <= CAR_LAMP_EDGE_MARGIN ||
         comp->max_x >= BEACON_IMAGE_W - 1 - CAR_LAMP_EDGE_MARGIN) ? 1 : 0;

    if (touches_top_or_bottom != 0 && comp->area < CAR_LAMP_MIN_AREA)
    {
        return 0;
    }
    if (comp->area < CAR_LAMP_MIN_AREA)
    {
        return 0;
    }
    if (comp->elongation < CAR_LAMP_MIN_ELONGATION &&
        (comp->area < CAR_LAMP_LARGE_AREA ||
         comp->elongation < CAR_LAMP_LARGE_ELONGATION))
    {
        return 0;
    }
    if (comp->major < CAR_LAMP_MIN_LENGTH)
    {
        return 0;
    }
    if (touches_top_or_bottom != 0 &&
        comp->area < CAR_LAMP_LARGE_AREA &&
        local_background_average(comp, CAR_LAMP_LOCAL_RING_PAD) >
            CAR_LAMP_EDGE_BACKGROUND_MAX)
    {
        return 0;
    }
    if (touches_left_or_right != 0 &&
        comp->cy < (float)BEACON_IMAGE_H * 0.5f &&
        local_background_average(comp, CAR_LAMP_LOCAL_RING_PAD) >
            CAR_LAMP_EDGE_BACKGROUND_MAX)
    {
        return 0;
    }
    if (touches_left_or_right != 0 &&
        comp->cy < CAR_LAMP_SIDE_SUN_Y &&
        local_background_average(comp, CAR_LAMP_LOCAL_RING_PAD) >
            CAR_LAMP_SIDE_SUN_BACKGROUND_MAX)
    {
        return 0;
    }
    if ((comp->min_x <= CAR_LAMP_SIDE_SUN_MARGIN ||
         comp->max_x >= BEACON_IMAGE_W - 1 - CAR_LAMP_SIDE_SUN_MARGIN) &&
        comp->cy < CAR_LAMP_SIDE_SUN_Y &&
        local_background_average(comp, CAR_LAMP_LOCAL_RING_PAD) >
            CAR_LAMP_SIDE_SUN_BACKGROUND_MAX)
    {
        return 0;
    }
    if (comp->area < CAR_LAMP_STRONG_AREA &&
        (bbox_span < CAR_LAMP_SMALL_MIN_SPAN ||
         local_background_average(comp, CAR_LAMP_LOCAL_RING_PAD) >
             CAR_LAMP_SMALL_BACKGROUND_MAX))
    {
        return 0;
    }
    return 1;
}

static float lamp_score(const component_t *comp)
{
    return (float)comp->area * comp->elongation;
}

static unsigned char is_edge_lamp_candidate(const component_t *comp)
{
    if (comp == 0 || comp->valid == 0)
    {
        return 0;
    }
    if (comp->min_y > 2 && comp->max_y < BEACON_IMAGE_H - 3)
    {
        return 0;
    }
    return (comp->elongation >= 2.6f &&
            comp->major >= 16.0f &&
            comp->minor <= 6.5f) ? 1 : 0;
}

static unsigned char find_car_lamp(component_t *best_lamp)
{
    unsigned char x;
    unsigned char y;
    float best_score = 0.0f;
    unsigned char found = 0;

    memset(best_lamp, 0, sizeof(*best_lamp));
    begin_visit_pass();

    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            component_t comp;
            float score;

            if (g_binary[y][x] == 0 || is_visited(x, y))
            {
                continue;
            }

            comp = grow_component(x, y);
            if (!is_lamp_candidate(&comp))
            {
                continue;
            }

            score = lamp_score(&comp);
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

static unsigned char find_edge_car_lamp(component_t *best_lamp)
{
    unsigned char x;
    unsigned char y;
    float best_score = 0.0f;
    unsigned char found = 0;

    memset(best_lamp, 0, sizeof(*best_lamp));
    begin_visit_pass();

    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            component_t comp;
            float score;

            if (g_binary[y][x] == 0 || is_visited(x, y))
            {
                continue;
            }

            comp = grow_component(x, y);
            if (!is_edge_lamp_candidate(&comp))
            {
                continue;
            }

            score = lamp_score(&comp);
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
    beacon_rect_t *rect;

    if (lamp->valid == 0)
    {
        result->car_lamp_count = 0;
        return;
    }

    rect = &result->car_lamps[0];
    rect->cx = (float)BEACON_IMAGE_W * 0.5f - lamp->cx;
    rect->cy = lamp->cy - (float)BEACON_IMAGE_H * 0.5f;
    rect->length = lamp->major;
    rect->width = lamp->minor;
    rect->angle = lamp->angle;
    rect->valid = 1;
    result->car_lamp_count = 1;
}

static void erase_lamp_rect_from_binary(const component_t *lamp)
{
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    int x;
    int y;

    if (lamp == 0 || lamp->valid == 0)
    {
        return;
    }

    min_x = lamp->min_x - LAMP_MASK_PAD;
    max_x = lamp->max_x + LAMP_MASK_PAD;
    min_y = lamp->min_y - LAMP_MASK_PAD;
    max_y = lamp->max_y + LAMP_MASK_PAD;

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

static unsigned char is_side_edge_beacon(const component_t *comp)
{
    if (comp == 0 || comp->valid == 0)
    {
        return 0;
    }
    return (comp->min_x < BEACON_SIDE_EDGE_MARGIN ||
            comp->max_x >= BEACON_IMAGE_W - BEACON_SIDE_EDGE_MARGIN) ? 1 : 0;
}

static void insert_beacon_by_area(
    const component_t *comp,
    const component_t *lamp,
    beacon_result_t *result)
{
    int i;
    int slot;
    int min_area;
    beacon_circle_t circle;

    if (comp == 0 || comp->valid == 0)
    {
        return;
    }
    min_area = is_side_edge_beacon(comp) != 0
        ? BEACON_SIDE_EDGE_MIN_AREA
        : BEACON_MIN_COMPONENT_AREA;
    if (comp->area < min_area)
    {
        return;
    }
    if (is_near_lamp(comp, lamp) != 0 &&
        comp->area < LAMP_NEAR_BEACON_MIN_AREA)
    {
        return;
    }

    slot = result->beacon_count;
    if (slot >= BEACON_MAX_BEACON_COUNT)
    {
        slot = BEACON_MAX_BEACON_COUNT - 1;
        if ((float)comp->area <= result->beacons[slot].radius * result->beacons[slot].radius * PI_F)
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
        if ((float)comp->area <= result->beacons[i].radius * result->beacons[i].radius * PI_F)
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
    beacon_result_t *result)
{
    unsigned char x;
    unsigned char y;

    result->beacon_count = 0;
    threshold_beacon_image(image);
    erase_lamp_rect_from_binary(lamp);
    begin_visit_pass();

    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            component_t comp;

            if (g_binary[y][x] == 0 || is_visited(x, y))
            {
                continue;
            }

            comp = grow_component(x, y);
            insert_beacon_by_area(&comp, lamp, result);
        }
    }

    sync_legacy_beacons(result);
}

void beacon_image_process(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result)
{
    component_t lamp;
    unsigned char has_lamp;
    int i;

    if (result == 0)
    {
        return;
    }

    clear_result(result);
    if (image == 0)
    {
        return;
    }

    g_current_image = image;
    threshold_image(image, CAR_LAMP_BINARY_THRESHOLD);
    has_lamp = find_car_lamp(&lamp);
    if (has_lamp == 0)
    {
        has_lamp = find_edge_car_lamp(&lamp);
    }

    if (has_lamp == 0)
    {
        memset(&lamp, 0, sizeof(lamp));
        g_has_lamp_track = 0;
    }
    else
    {
        g_has_lamp_track = 1;
    }

    write_car_lamp(&lamp, result);
    find_beacons(image, &lamp, result);

    for (i = result->car_lamp_count; i < BEACON_MAX_CAR_LAMP_COUNT; i++)
    {
        result->car_lamps[i].valid = 0;
    }
}

void beacon_image_debug_binary(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char binary[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    int x;
    int y;

    if (image == 0 || binary == 0)
    {
        return;
    }

    threshold_image(image, CAR_LAMP_BINARY_THRESHOLD);
    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            binary[y][x] = g_binary[y][x];
        }
    }
}
