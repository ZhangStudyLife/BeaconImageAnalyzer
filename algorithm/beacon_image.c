#include "beacon_image.h"
#include "beacon_image_config.h"

#include <math.h>
#include <string.h>

/* 内部圆结构，与车端 image_circle 一致 */
typedef struct
{
    float x;
    float y;
    float radius;
    unsigned char valid;
} internal_circle_t;

/* 连通域统计 */
typedef struct
{
    int area;
    int sum_x;
    int sum_y;
} component_t;

/* 静态缓冲区 */
static unsigned char g_binary[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_visit_stamp[BEACON_IMAGE_H][BEACON_IMAGE_W];
static unsigned char g_current_stamp = 0;
static unsigned char g_queue_x[BEACON_QUEUE_SIZE];
static unsigned char g_queue_y[BEACON_QUEUE_SIZE];
static internal_circle_t g_circles[BEACON_MAX_CIRCLE_COUNT];
static unsigned short g_circle_area[BEACON_MAX_CIRCLE_COUNT];

/* 全局结果 */
static beacon_result_t g_result;

void beacon_image_init(void)
{
    memset(g_binary, 0, sizeof(g_binary));
    memset(g_visit_stamp, 0, sizeof(g_visit_stamp));
    memset(g_circles, 0, sizeof(g_circles));
    memset(g_circle_area, 0, sizeof(g_circle_area));
    memset(&g_result, 0, sizeof(g_result));
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

/* 固定阈值二值化，与车端一致 */
static void threshold_fixed(const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    const unsigned char *src = &image[0][0];
    unsigned char *dst = &g_binary[0][0];
    int i;

    for (i = 0; i < BEACON_IMAGE_W * BEACON_IMAGE_H; i++)
    {
        dst[i] = (src[i] >= BEACON_BINARY_THRESHOLD) ? 255 : 0;
    }
}

/* 8 连通域 flood fill */
static component_t grow_component(unsigned char start_x, unsigned char start_y)
{
    static const signed char dx[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
    static const signed char dy[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };
    unsigned short head = 0;
    unsigned short tail = 0;
    component_t comp;

    comp.area = 0;
    comp.sum_x = 0;
    comp.sum_y = 0;

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
        comp.sum_x += x;
        comp.sum_y += y;

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

    return comp;
}

/* 按面积排序插入 */
static void insert_sorted(const component_t *comp)
{
    int i;
    int slot;

    if (comp->area < BEACON_MIN_COMPONENT_AREA)
    {
        return;
    }

    slot = g_result.count;
    if (slot >= BEACON_MAX_CIRCLE_COUNT)
    {
        slot = BEACON_MAX_CIRCLE_COUNT - 1;
        if (comp->area <= g_circle_area[slot])
        {
            return;
        }
    }
    else
    {
        g_result.count++;
    }

    for (i = slot - 1; i >= 0; i--)
    {
        if (comp->area <= g_circle_area[i])
        {
            break;
        }
        g_circles[i + 1] = g_circles[i];
        g_circle_area[i + 1] = g_circle_area[i];
    }

    g_circle_area[i + 1] = (unsigned short)comp->area;
    g_circles[i + 1].x = (float)comp->sum_x / (float)comp->area;
    g_circles[i + 1].y = (float)comp->sum_y / (float)comp->area;
    g_circles[i + 1].radius = sqrtf((float)comp->area / 3.1415926f);
    g_circles[i + 1].valid = 1;
}

/* 查找连通域 */
static void find_components(void)
{
    unsigned char x;
    unsigned char y;

    memset(g_circles, 0, sizeof(g_circles));
    memset(g_circle_area, 0, sizeof(g_circle_area));
    g_result.count = 0;
    begin_visit_pass();

    for (y = 0; y < BEACON_IMAGE_H; y++)
    {
        for (x = 0; x < BEACON_IMAGE_W; x++)
        {
            component_t comp;

            if (g_binary[y][x] == 0)
            {
                continue;
            }
            if (is_visited(x, y))
            {
                continue;
            }

            comp = grow_component(x, y);
            insert_sorted(&comp);
        }
    }
}

void beacon_image_process(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result)
{
    int i;

    if (result == 0)
    {
        return;
    }

    memset(result, 0, sizeof(*result));

    if (image == 0)
    {
        return;
    }

    threshold_fixed(image);
    find_components();

    /* 坐标转换：图像坐标 -> 算法坐标 */
    result->beacon_count = g_result.count;
    result->car_lamp_count = 0;

    for (i = 0; i < g_result.count; i++)
    {
        result->circles[i].x = (float)BEACON_IMAGE_W * 0.5f - g_circles[i].x;
        result->circles[i].y = g_circles[i].y - (float)BEACON_IMAGE_H * 0.5f;
        result->circles[i].radius = g_circles[i].radius;
        result->circles[i].valid = g_circles[i].valid;

        /* 默认全部作为信标 */
        result->beacons[i] = result->circles[i];
    }

    for (i = 0; i < BEACON_MAX_BEACON_COUNT; i++)
    {
        if (i >= g_result.count)
        {
            result->beacons[i].valid = 0;
        }
    }

    for (i = 0; i < BEACON_MAX_CAR_LAMP_COUNT; i++)
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

    for (i = 0; i < BEACON_IMAGE_W * BEACON_IMAGE_H; i++)
    {
        ((unsigned char *)binary)[i] = (((const unsigned char *)image)[i] >= BEACON_BINARY_THRESHOLD) ? 255 : 0;
    }
}
