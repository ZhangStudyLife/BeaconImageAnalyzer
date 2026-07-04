#include "beacon_image.h"
#include "beacon_image_config.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define CAR_LAMP_BINARY_THRESHOLD 200
#define CAR_LAMP_UPPER_THRESHOLD  150
#define CAR_LAMP_UPPER_Y          64
#define CAR_LAMP_BRIDGE_MAX_GAP   5
#define CAR_LAMP_MIN_AREA         24
#define CAR_LAMP_MAX_AREA         1200
#define CAR_LAMP_MIN_ELONGATION   1.6f
#define CAR_LAMP_MIN_LENGTH       10.0f

#define BEACON_LOCAL_MEAN_RADIUS  4
#define BEACON_MIN_GRAY           55
#define BEACON_MIN_CONTRAST       35
#define BEACON_MIN_AREA           11
#define BEACON_EDGE_MIN_AREA      3
#define BEACON_EDGE_MAX_AREA      60
#define BEACON_EDGE_TOP_Y         36
#define BEACON_EDGE_LEFT_X        16
#define BEACON_EDGE_RIGHT_X       172
#define BEACON_LOCAL_RING_INNER   3
#define BEACON_LOCAL_RING_OUTER   8
#define BEACON_HALO_AREA_MAX      40
#define BEACON_HALO_BACKGROUND_MAX 12

#define COMPONENT_BOTTOM_REJECT_Y (BEACON_IMAGE_H - 2)
#define LAMP_MASK_PAD             4
#define LAMP_MASK_DOWN_PAD        10
#define LAMP_NEAR_BEACON_PAD      8
#define LAMP_NEAR_BEACON_MIN_AREA 21
#define IMAGE_QUEUE_SIZE          (BEACON_IMAGE_W * BEACON_IMAGE_H)
#define PI_F                      3.1415926f

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
static unsigned char g_visit[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_queue_x[IMAGE_QUEUE_SIZE];
static unsigned char g_queue_y[IMAGE_QUEUE_SIZE];
static uint16_t g_col_sum[BEACON_IMAGE_W];

void beacon_image_init(void)
{
    memset(g_binary, 0, sizeof(g_binary));
    memset(g_visit, 0, sizeof(g_visit));
    memset(g_col_sum, 0, sizeof(g_col_sum));
}

static void clear_result(beacon_result_t *result)
{
    if (result != 0)
    {
        memset(result, 0, sizeof(*result));
    }
}

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
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
    int x;
    int y;

    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        unsigned char threshold = CAR_LAMP_BINARY_THRESHOLD;
        if (y < CAR_LAMP_UPPER_Y)
        {
            threshold = CAR_LAMP_UPPER_THRESHOLD;
        }

        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            g_binary[y][x] = (image[y][x] >= threshold) ? 255 : 0;
        }
    }
    bridge_upper_car_lamp_gaps();
    memset(g_visit, 0, sizeof(g_visit));
}

static void update_local_column_sums(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int min_y,
    int max_y)
{
    int x;
    int y;

    for (x = 0; x < BEACON_IMAGE_W; x++)
    {
        uint16_t sum = 0;
        for (y = min_y; y <= max_y; y++)
        {
            sum = (uint16_t)(sum + image[y][x]);
        }
        g_col_sum[x] = sum;
    }
}

static void threshold_beacon_image(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    int y;

    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        int x;
        int min_y = clamp_int(y - BEACON_LOCAL_MEAN_RADIUS, 0,
                              BEACON_IMAGE_H - 1);
        int max_y = clamp_int(y + BEACON_LOCAL_MEAN_RADIUS, 0,
                              BEACON_IMAGE_H - 1);
        int height = max_y - min_y + 1;
        uint16_t window_sum = 0;

        update_local_column_sums(image, min_y, max_y);

        for (x = 0; x <= BEACON_LOCAL_MEAN_RADIUS && x < BEACON_IMAGE_W; x++)
        {
            window_sum = (uint16_t)(window_sum + g_col_sum[x]);
        }

        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            int min_x = x - BEACON_LOCAL_MEAN_RADIUS;
            int max_x = x + BEACON_LOCAL_MEAN_RADIUS;
            int width;
            int local_mean;
            int value = image[y][x];

            if (min_x < 0) min_x = 0;
            if (max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
            width = max_x - min_x + 1;
            local_mean = window_sum / (width * height);

            g_binary[y][x] = (value >= BEACON_MIN_GRAY &&
                              value - local_mean >= BEACON_MIN_CONTRAST) ?
                             255 : 0;

            if (x - BEACON_LOCAL_MEAN_RADIUS >= 0)
            {
                window_sum = (uint16_t)(
                    window_sum - g_col_sum[x - BEACON_LOCAL_MEAN_RADIUS]);
            }
            if (x + BEACON_LOCAL_MEAN_RADIUS + 1 < BEACON_IMAGE_W)
            {
                window_sum = (uint16_t)(
                    window_sum + g_col_sum[x + BEACON_LOCAL_MEAN_RADIUS + 1]);
            }
        }
    }
    memset(g_visit, 0, sizeof(g_visit));
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
        comp.angle = 0.5f * atan2f(2.0f * cov_xy, var_x - var_y) *
                     180.0f / PI_F;
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

static unsigned char find_car_lamp(component_t *best_lamp)
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

            if (g_binary[y][x] == 0 || g_visit[y][x] != 0)
            {
                continue;
            }
            comp = grow_component(x, y);
            if (is_lamp_candidate(&comp) == 0)
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

    min_x = clamp_int(lamp->min_x - LAMP_MASK_PAD, 0, BEACON_IMAGE_W - 1);
    max_x = clamp_int(lamp->max_x + LAMP_MASK_PAD, 0, BEACON_IMAGE_W - 1);
    min_y = clamp_int(lamp->min_y - LAMP_MASK_PAD, 0, BEACON_IMAGE_H - 1);
    max_y = clamp_int(lamp->max_y + LAMP_MASK_DOWN_PAD, 0,
                      BEACON_IMAGE_H - 1);

    for (y = min_y; y <= max_y; y++)
    {
        for (x = min_x; x <= max_x; x++)
        {
            g_binary[y][x] = 0;
        }
    }
}

static unsigned char is_near_lamp(
    const component_t *comp,
    const component_t *lamp)
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
    min_x = clamp_int(cx - BEACON_LOCAL_RING_OUTER, 0, BEACON_IMAGE_W - 1);
    max_x = clamp_int(cx + BEACON_LOCAL_RING_OUTER, 0, BEACON_IMAGE_W - 1);
    min_y = clamp_int(cy - BEACON_LOCAL_RING_OUTER, 0, BEACON_IMAGE_H - 1);
    max_y = clamp_int(cy + BEACON_LOCAL_RING_OUTER, 0, BEACON_IMAGE_H - 1);

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

static unsigned char is_halo_noise_beacon(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp)
{
    if (comp == 0 || comp->valid == 0 ||
        comp->area > BEACON_HALO_AREA_MAX)
    {
        return 0;
    }
    return (local_background_average(image, comp) >
            BEACON_HALO_BACKGROUND_MAX) ? 1 : 0;
}

static void insert_beacon_by_area(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp,
    const component_t *lamp,
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
    if (comp->max_y >= COMPONENT_BOTTOM_REJECT_Y)
    {
        return;
    }

    is_edge = is_edge_beacon_area(comp);
    is_side_edge = is_side_edge_beacon_area(comp);
    min_area = (is_edge != 0) ? BEACON_EDGE_MIN_AREA :
                                BEACON_MIN_AREA;
    if (is_side_edge != 0 && comp->area > BEACON_EDGE_MAX_AREA)
    {
        return;
    }
    if (comp->area < min_area)
    {
        return;
    }
    if (is_halo_noise_beacon(image, comp) != 0)
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
    beacon_result_t *result)
{
    unsigned char x;
    unsigned char y;

    result->beacon_count = 0;
    threshold_beacon_image(image);
    erase_lamp_from_binary(lamp);

    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            component_t comp;

            if (g_binary[y][x] == 0 || g_visit[y][x] != 0)
            {
                continue;
            }
            comp = grow_component(x, y);
            insert_beacon_by_area(image, &comp, lamp, result);
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

    if (result == 0)
    {
        return;
    }

    clear_result(result);
    if (image == 0)
    {
        return;
    }

    threshold_car_lamp_image(image);
    has_lamp = find_car_lamp(&lamp);
    if (has_lamp == 0)
    {
        memset(&lamp, 0, sizeof(lamp));
    }
    write_car_lamp(&lamp, result);
    find_beacons(image, &lamp, result);
}

unsigned char beacon_image_debug_threshold(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    (void)image;
    return BEACON_MIN_CONTRAST;
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
    threshold_beacon_image(image);
    memcpy(binary, g_binary, sizeof(g_binary));
}
