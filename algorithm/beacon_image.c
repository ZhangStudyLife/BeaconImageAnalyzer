#include "beacon_image.h"

#include "beacon_image_config.h"

#include <math.h>
#include <string.h>

typedef struct
{
    int area;
    int sum_x;
    int sum_y;
    int min_x;
    int min_y;
    int max_x;
    int max_y;
} beacon_component_t;

static unsigned char g_visited[BEACON_IMAGE_H][BEACON_IMAGE_W];
static int g_queue_x[BEACON_IMAGE_W * BEACON_IMAGE_H];
static int g_queue_y[BEACON_IMAGE_W * BEACON_IMAGE_H];
static beacon_component_t g_components[BEACON_IMAGE_W * BEACON_IMAGE_H / BEACON_MIN_COMPONENT_AREA];

void beacon_image_init(void)
{
    memset(g_visited, 0, sizeof(g_visited));
}

void beacon_image_reset_temporal(void)
{
}

static void clear_result(beacon_result_t *result)
{
    int i;

    if (result == 0)
    {
        return;
    }

    memset(result, 0, sizeof(*result));
    for (i = 0; i < BEACON_MAX_CIRCLE_COUNT; ++i)
    {
        result->circles[i].valid = 0;
    }
    for (i = 0; i < BEACON_MAX_BEACON_COUNT; ++i)
    {
        result->beacons[i].valid = 0;
    }
    for (i = 0; i < BEACON_MAX_CAR_LAMP_COUNT; ++i)
    {
        result->car_lamps[i].valid = 0;
    }
}

static unsigned char compute_threshold(const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    int x;
    int y;
    int max_value = 0;
    unsigned long sum = 0;
    const int pixel_count = BEACON_IMAGE_W * BEACON_IMAGE_H;
    int mean_value;
    int threshold;

    for (y = 0; y < BEACON_IMAGE_H; ++y)
    {
        for (x = 0; x < BEACON_IMAGE_W; ++x)
        {
            const int value = image[y][x];
            sum += (unsigned long)value;
            if (value > max_value)
            {
                max_value = value;
            }
        }
    }

    if (max_value < BEACON_THRESHOLD_MIN_VALUE)
    {
        return 255;
    }

    mean_value = (int)(sum / (unsigned long)pixel_count);
    threshold = mean_value + (max_value - mean_value) * 45 / 100;
    if (threshold < BEACON_THRESHOLD_MIN_VALUE)
    {
        threshold = BEACON_THRESHOLD_MIN_VALUE;
    }
    if (threshold > 245)
    {
        threshold = 245;
    }

    return (unsigned char)threshold;
}

unsigned char beacon_image_debug_threshold(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    if (image == 0)
    {
        return 255;
    }
    return compute_threshold(image);
}

void beacon_image_debug_binary(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char binary[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    int x;
    int y;
    const unsigned char threshold = beacon_image_debug_threshold(image);

    if (image == 0 || binary == 0)
    {
        return;
    }

    for (y = 0; y < BEACON_IMAGE_H; ++y)
    {
        for (x = 0; x < BEACON_IMAGE_W; ++x)
        {
            binary[y][x] = (threshold != 255 && image[y][x] >= threshold) ? 255 : 0;
        }
    }
}

static beacon_component_t flood_component(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    int start_x,
    int start_y,
    unsigned char threshold)
{
    int head = 0;
    int tail = 0;
    beacon_component_t component;

    component.area = 0;
    component.sum_x = 0;
    component.sum_y = 0;
    component.min_x = start_x;
    component.min_y = start_y;
    component.max_x = start_x;
    component.max_y = start_y;

    g_queue_x[tail] = start_x;
    g_queue_y[tail] = start_y;
    ++tail;
    g_visited[start_y][start_x] = 1;

    while (head < tail)
    {
        static const int dx[4] = { 1, -1, 0, 0 };
        static const int dy[4] = { 0, 0, 1, -1 };
        int i;
        const int x = g_queue_x[head];
        const int y = g_queue_y[head];
        ++head;

        ++component.area;
        component.sum_x += x;
        component.sum_y += y;
        if (x < component.min_x)
        {
            component.min_x = x;
        }
        if (y < component.min_y)
        {
            component.min_y = y;
        }
        if (x > component.max_x)
        {
            component.max_x = x;
        }
        if (y > component.max_y)
        {
            component.max_y = y;
        }

        for (i = 0; i < 4; ++i)
        {
            const int nx = x + dx[i];
            const int ny = y + dy[i];
            if (nx < 0 || nx >= BEACON_IMAGE_W || ny < 0 || ny >= BEACON_IMAGE_H)
            {
                continue;
            }
            if (g_visited[ny][nx] != 0 || image[ny][nx] < threshold)
            {
                continue;
            }
            g_visited[ny][nx] = 1;
            g_queue_x[tail] = nx;
            g_queue_y[tail] = ny;
            ++tail;
        }
    }

    return component;
}

static void insert_component_sorted(beacon_component_t *components, int *count, beacon_component_t component)
{
    int pos = *count;
    int i;

    if (pos >= BEACON_MAX_CIRCLE_COUNT && component.area <= components[BEACON_MAX_CIRCLE_COUNT - 1].area)
    {
        return;
    }

    if (pos < BEACON_MAX_CIRCLE_COUNT)
    {
        ++(*count);
    }
    else
    {
        pos = BEACON_MAX_CIRCLE_COUNT - 1;
    }

    for (i = pos - 1; i >= 0 && components[i].area < component.area; --i)
    {
        components[i + 1] = components[i];
    }
    components[i + 1] = component;
}

void beacon_image_process(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result)
{
    unsigned char threshold;
    int x;
    int y;
    int component_count = 0;
    int i;

    clear_result(result);
    if (image == 0 || result == 0)
    {
        return;
    }

    memset(g_visited, 0, sizeof(g_visited));
    memset(g_components, 0, sizeof(g_components));
    threshold = compute_threshold(image);
    if (threshold == 255)
    {
        return;
    }

    for (y = 0; y < BEACON_IMAGE_H; ++y)
    {
        for (x = 0; x < BEACON_IMAGE_W; ++x)
        {
            beacon_component_t component;

            if (g_visited[y][x] != 0 || image[y][x] < threshold)
            {
                continue;
            }

            component = flood_component(image, x, y, threshold);
            if (component.area < BEACON_MIN_COMPONENT_AREA ||
                component.area > BEACON_MAX_COMPONENT_AREA)
            {
                continue;
            }

            insert_component_sorted(g_components, &component_count, component);
        }
    }

    result->count = (unsigned char)component_count;
    result->beacon_count = (unsigned char)component_count;
    result->car_lamp_count = 0;
    for (i = 0; i < component_count; ++i)
    {
        const beacon_component_t *component = &g_components[i];
        const float center_x = (float)component->sum_x / (float)component->area;
        const float center_y = (float)component->sum_y / (float)component->area;
        const float radius = sqrtf((float)component->area / 3.1415926f);

        result->circles[i].x = (float)BEACON_IMAGE_W * 0.5f - center_x;
        result->circles[i].y = center_y - (float)BEACON_IMAGE_H * 0.5f;
        result->circles[i].radius = radius;
        result->circles[i].valid = 1;
        result->beacons[i] = result->circles[i];
    }
}
