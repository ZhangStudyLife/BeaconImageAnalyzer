#include "beacon_image.h"
#include "beacon_image_config.h"

#include <math.h>
#include <string.h>

#define PI_F 3.1415926f
#define MAX_INTERNAL_BEACONS BEACON_MAX_BEACON_COUNT

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
    float radius;
    unsigned char valid;
    unsigned char used;
} component_t;

typedef struct
{
    beacon_circle_t circle;
    int area;
    float image_x;
    float image_y;
    unsigned char valid;
    unsigned char used;
} beacon_candidate_t;

static unsigned char g_binary[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_mask[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_visit_stamp[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_current_stamp = 0;
static unsigned char g_queue_x[BEACON_QUEUE_SIZE];
static unsigned char g_queue_y[BEACON_QUEUE_SIZE];
static beacon_candidate_t g_beacon_candidates[MAX_INTERNAL_BEACONS];
static unsigned char g_beacon_candidate_count = 0;
static component_t g_lamp_mask_components[BEACON_MAX_CAR_LAMP_COUNT + 3];
static unsigned char g_lamp_mask_component_count = 0;
static beacon_circle_t g_last_beacons[BEACON_MAX_BEACON_COUNT];
static unsigned char g_has_last_beacons = 0;

static float squaref_local(float value)
{
    return value * value;
}

static void clear_result(beacon_result_t *result)
{
    memset(result, 0, sizeof(*result));
}

void beacon_image_init(void)
{
    memset(g_binary, 0, sizeof(g_binary));
    memset(g_mask, 0, sizeof(g_mask));
    memset(g_visit_stamp, 0, sizeof(g_visit_stamp));
    memset(g_beacon_candidates, 0, sizeof(g_beacon_candidates));
    memset(g_lamp_mask_components, 0, sizeof(g_lamp_mask_components));
    memset(g_last_beacons, 0, sizeof(g_last_beacons));
    g_beacon_candidate_count = 0;
    g_lamp_mask_component_count = 0;
    g_has_last_beacons = 0;
    g_current_stamp = 0;
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
    unsigned char threshold,
    unsigned char use_mask)
{
    const unsigned char *src = &image[0][0];
    unsigned char *dst = &g_binary[0][0];
    const unsigned char *mask = &g_mask[0][0];
    int i;

    for (i = 0; i < BEACON_IMAGE_W * BEACON_IMAGE_H; i++)
    {
        dst[i] = (src[i] >= threshold && (use_mask == 0 || mask[i] == 0)) ? 255 : 0;
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

            if (nx < 0 || nx >= BEACON_IMAGE_W || ny < 0 || ny >= BEACON_IMAGE_H)
            {
                continue;
            }
            if (g_binary[ny][nx] == 0)
            {
                continue;
            }
            if (is_visited((unsigned char)nx, (unsigned char)ny))
            {
                continue;
            }
            if (tail >= BEACON_QUEUE_SIZE)
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
        comp.radius = sqrtf((float)comp.area / PI_F);
        comp.valid = 1;
    }

    return comp;
}

static unsigned char is_lamp_candidate(const component_t *comp)
{
    unsigned char touches_edge;
    int bbox_w = comp->max_x - comp->min_x + 1;
    int bbox_h = comp->max_y - comp->min_y + 1;
    int bbox_span = bbox_w > bbox_h ? bbox_w : bbox_h;

    if (comp->area > BEACON_MAX_LAMP_AREA)
    {
        return 0;
    }

    touches_edge =
        (comp->min_x <= BEACON_EDGE_LAMP_MARGIN ||
         comp->max_x >= BEACON_IMAGE_W - 1 - BEACON_EDGE_LAMP_MARGIN ||
         comp->min_y <= BEACON_EDGE_LAMP_MARGIN ||
         comp->max_y >= BEACON_IMAGE_H - 1 - BEACON_EDGE_LAMP_MARGIN) ? 1 : 0;

    if (touches_edge != 0 &&
        comp->area >= BEACON_EDGE_LAMP_MIN_AREA &&
        bbox_span >= BEACON_EDGE_LAMP_MIN_SPAN)
    {
        return 1;
    }

    if (comp->area < BEACON_MIN_LAMP_AREA)
    {
        return 0;
    }
    if (comp->elongation < BEACON_MIN_LAMP_ELONGATION)
    {
        return 0;
    }
    if (comp->major < BEACON_MIN_LAMP_LENGTH)
    {
        return 0;
    }
    return 1;
}

static float lamp_score(const component_t *comp)
{
    return (float)comp->area * comp->elongation;
}

static unsigned char find_car_lamp(component_t *best_lamp)
{
    unsigned char x;
    unsigned char y;
    float best_score = 0.0f;
    unsigned char found = 0;

    memset(best_lamp, 0, sizeof(*best_lamp));
    memset(g_lamp_mask_components, 0, sizeof(g_lamp_mask_components));
    g_lamp_mask_component_count = 0;
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

            if (g_lamp_mask_component_count <
                (unsigned char)(BEACON_MAX_CAR_LAMP_COUNT + 3))
            {
                g_lamp_mask_components[g_lamp_mask_component_count] = comp;
                g_lamp_mask_component_count++;
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

static void build_lamp_mask(const component_t *lamp)
{
    int seed_min_x = lamp->min_x - BEACON_LAMP_MASK_PAD;
    int seed_max_x = lamp->max_x + BEACON_LAMP_MASK_PAD;
    int seed_min_y = lamp->min_y - BEACON_LAMP_MASK_PAD;
    int seed_max_y = lamp->max_y + BEACON_LAMP_MASK_PAD;
    int x;
    int y;

    if (lamp->valid == 0)
    {
        return;
    }

    if (seed_min_x < 0) seed_min_x = 0;
    if (seed_min_y < 0) seed_min_y = 0;
    if (seed_max_x >= BEACON_IMAGE_W) seed_max_x = BEACON_IMAGE_W - 1;
    if (seed_max_y >= BEACON_IMAGE_H) seed_max_y = BEACON_IMAGE_H - 1;

    for (y = seed_min_y; y <= seed_max_y; y++)
    {
        for (x = seed_min_x; x <= seed_max_x; x++)
        {
            g_mask[y][x] = 1;
        }
    }
}

static void build_all_lamp_masks(const component_t *lamp)
{
    unsigned char i;

    memset(g_mask, 0, sizeof(g_mask));
    if (lamp->valid == 0)
    {
        return;
    }

    for (i = 0; i < g_lamp_mask_component_count; i++)
    {
        build_lamp_mask(&g_lamp_mask_components[i]);
    }
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

static unsigned char is_beacon_candidate(const component_t *comp)
{
    unsigned char touches_edge =
        (comp->min_x <= BEACON_EDGE_BEACON_MARGIN ||
         comp->max_x >= BEACON_IMAGE_W - 1 - BEACON_EDGE_BEACON_MARGIN ||
         comp->min_y <= BEACON_EDGE_BEACON_MARGIN ||
         comp->max_y >= BEACON_IMAGE_H - 1 - BEACON_EDGE_BEACON_MARGIN) ? 1 : 0;

    if (comp->area < BEACON_MIN_COMPONENT_AREA || comp->area > BEACON_MAX_BEACON_AREA)
    {
        return 0;
    }
    if (touches_edge != 0 && comp->area < BEACON_EDGE_BEACON_MIN_AREA)
    {
        return 0;
    }
    if (comp->elongation > BEACON_MAX_BEACON_ELONGATION && comp->area > 20)
    {
        return 0;
    }
    return 1;
}

static void insert_beacon_candidate(const component_t *comp)
{
    int i;
    int slot;
    float comp_x;
    float comp_y;

    if (!is_beacon_candidate(comp))
    {
        return;
    }

    comp_x = (float)BEACON_IMAGE_W * 0.5f - comp->cx;
    comp_y = comp->cy - (float)BEACON_IMAGE_H * 0.5f;
    for (i = 0; i < g_beacon_candidate_count; i++)
    {
        float dx = g_beacon_candidates[i].circle.x - comp_x;
        float dy = g_beacon_candidates[i].circle.y - comp_y;
        if (dx * dx + dy * dy <= BEACON_DUPLICATE_DISTANCE * BEACON_DUPLICATE_DISTANCE)
        {
            if (comp->area > g_beacon_candidates[i].area)
            {
                g_beacon_candidates[i].circle.x = comp_x;
                g_beacon_candidates[i].circle.y = comp_y;
                g_beacon_candidates[i].circle.radius = comp->radius;
                g_beacon_candidates[i].circle.valid = 1;
                g_beacon_candidates[i].area = comp->area;
                g_beacon_candidates[i].image_x = comp->cx;
                g_beacon_candidates[i].image_y = comp->cy;
                g_beacon_candidates[i].valid = 1;
            }
            return;
        }
    }

    slot = g_beacon_candidate_count;
    if (slot >= MAX_INTERNAL_BEACONS)
    {
        slot = MAX_INTERNAL_BEACONS - 1;
        if (comp->area <= g_beacon_candidates[slot].area)
        {
            return;
        }
    }
    else
    {
        g_beacon_candidate_count++;
    }

    for (i = slot - 1; i >= 0; i--)
    {
        if (comp->area <= g_beacon_candidates[i].area)
        {
            break;
        }
        g_beacon_candidates[i + 1] = g_beacon_candidates[i];
    }

    g_beacon_candidates[i + 1].circle.x = comp_x;
    g_beacon_candidates[i + 1].circle.y = comp_y;
    g_beacon_candidates[i + 1].circle.radius = comp->radius;
    g_beacon_candidates[i + 1].circle.valid = 1;
    g_beacon_candidates[i + 1].area = comp->area;
    g_beacon_candidates[i + 1].image_x = comp->cx;
    g_beacon_candidates[i + 1].image_y = comp->cy;
    g_beacon_candidates[i + 1].valid = 1;
    g_beacon_candidates[i + 1].used = 0;
}

static void find_beacon_candidates_pass(void)
{
    unsigned char x;
    unsigned char y;

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
            insert_beacon_candidate(&comp);
        }
    }
}

static void find_beacon_candidates(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    memset(g_beacon_candidates, 0, sizeof(g_beacon_candidates));
    g_beacon_candidate_count = 0;

    threshold_image(image, BEACON_BINARY_THRESHOLD, 1);
    find_beacon_candidates_pass();

    threshold_image(image, BEACON_BEACON_LOW_THRESHOLD, 1);
    find_beacon_candidates_pass();
}

static void copy_beacons_to_legacy(beacon_result_t *result)
{
    int i;

    result->count = result->beacon_count;
    for (i = 0; i < BEACON_MAX_CIRCLE_COUNT; i++)
    {
        if (i < BEACON_MAX_BEACON_COUNT)
        {
            result->circles[i] = result->beacons[i];
        }
        else
        {
            result->circles[i].valid = 0;
        }
    }
}

static void write_beacons(beacon_result_t *result)
{
    int i;
    int out_index;
    int highest_slot = 0;

    for (i = 0; i < BEACON_MAX_BEACON_COUNT; i++)
    {
        result->beacons[i].valid = 0;
    }

    if (g_has_last_beacons != 0)
    {
        for (out_index = 0; out_index < BEACON_MAX_BEACON_COUNT; out_index++)
        {
            int best = -1;
            float best_distance = 0.0f;

            if (g_last_beacons[out_index].valid == 0)
            {
                continue;
            }

            for (i = 0; i < g_beacon_candidate_count; i++)
            {
                float dx;
                float dy;
                float dist2;

                if (g_beacon_candidates[i].valid == 0 ||
                    g_beacon_candidates[i].used != 0)
                {
                    continue;
                }

                dx = g_beacon_candidates[i].circle.x - g_last_beacons[out_index].x;
                dy = g_beacon_candidates[i].circle.y - g_last_beacons[out_index].y;
                dist2 = dx * dx + dy * dy;
                if (dist2 > squaref_local(BEACON_TRACK_MATCH_DISTANCE))
                {
                    continue;
                }
                if (best < 0 || dist2 < best_distance)
                {
                    best = i;
                    best_distance = dist2;
                }
            }

            if (best >= 0)
            {
                result->beacons[out_index] = g_beacon_candidates[best].circle;
                g_beacon_candidates[best].used = 1;
            }
            if (g_last_beacons[out_index].valid != 0 &&
                highest_slot < out_index + 1)
            {
                highest_slot = out_index + 1;
            }
        }
    }

    out_index = g_has_last_beacons != 0 ? BEACON_MAX_BEACON_COUNT : 0;
    for (i = 0; i < g_beacon_candidate_count; i++)
    {
        if (g_beacon_candidates[i].valid == 0 ||
            g_beacon_candidates[i].used != 0)
        {
            continue;
        }

        while (out_index < BEACON_MAX_BEACON_COUNT &&
               result->beacons[out_index].valid != 0)
        {
            out_index++;
        }
        if (out_index >= BEACON_MAX_BEACON_COUNT)
        {
            break;
        }

        result->beacons[out_index] = g_beacon_candidates[i].circle;
        highest_slot = out_index + 1;
        out_index++;
    }

    for (i = 0; i < BEACON_MAX_BEACON_COUNT; i++)
    {
        if (result->beacons[i].valid != 0)
        {
            g_last_beacons[i] = result->beacons[i];
        }
    }
    if (highest_slot > 0 || g_has_last_beacons != 0)
    {
        g_has_last_beacons = 1;
    }

    result->beacon_count = (unsigned char)highest_slot;
    copy_beacons_to_legacy(result);
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

    threshold_image(image, BEACON_CAR_LAMP_THRESHOLD, 0);
    has_lamp = find_car_lamp(&lamp);
    if (has_lamp == 0)
    {
        memset(&lamp, 0, sizeof(lamp));
    }

    build_all_lamp_masks(&lamp);
    write_car_lamp(&lamp, result);

    if (has_lamp == 0)
    {
        return;
    }

    find_beacon_candidates(image);
    write_beacons(result);

    for (i = result->car_lamp_count; i < BEACON_MAX_CAR_LAMP_COUNT; i++)
    {
        result->car_lamps[i].valid = 0;
    }
}

unsigned char beacon_image_debug_threshold(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    (void)image;
    return BEACON_BINARY_THRESHOLD;
}

void beacon_image_debug_binary(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char binary[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    int i;

    if (image == 0 || binary == 0)
    {
        return;
    }

    threshold_image(image, BEACON_BINARY_THRESHOLD, 1);
    for (i = 0; i < BEACON_IMAGE_W * BEACON_IMAGE_H; i++)
    {
        ((unsigned char *)binary)[i] = ((const unsigned char *)g_binary)[i];
    }
}
