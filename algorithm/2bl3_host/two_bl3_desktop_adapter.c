#include <math.h>
#include <stdint.h>
#include <string.h>

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"
#include "Image/image.h"

#define DESKTOP_IMAGE_W 188
#define DESKTOP_IMAGE_H 120
#define DESKTOP_MAX_CIRCLES 8
#define DESKTOP_MAX_BEACONS 8
#define DESKTOP_MAX_CAR_LAMPS 4
#define DESKTOP_MAX_CANDIDATES 16
#define DESKTOP_PI 3.1415926f
#define DESKTOP_BEACON_PREDICT_FRAMES 2U
#define DESKTOP_BEACON_RETAIN_FRAMES 60U
#define DESKTOP_STRONG_THRESHOLD 160U
#define DESKTOP_STRONG_PEAK_MIN 200U
#define DESKTOP_STRONG_AREA_MIN 8
#define DESKTOP_STRONG_AREA_MAX 80
#define DESKTOP_STRONG_CONTRAST_MIN 150
#define DESKTOP_WEAK_PEAK_MIN 70U
#define DESKTOP_WEAK_CONTRAST_MIN 24
#define DESKTOP_WEAK_RING_MAX 45
#define DESKTOP_WEAK_AREA_MAX 20
#define DESKTOP_WEAK_MATCH_DISTANCE 12.0f
#define DESKTOP_TRACK_AREA_RATIO_MAX 4.0f
#define DESKTOP_REACQUIRE_AREA_RATIO_MAX 8.0f
#define DESKTOP_START_MATCH_DISTANCE 18.0f
#define DESKTOP_SATURATED_AREA_MIN 8.0f
#define DESKTOP_SATURATED_AREA_MAX 20.0f
#define DESKTOP_SATURATED_PEAK_MIN 230U
#define DESKTOP_SATURATED_BACKGROUND_MIN 70
#define DESKTOP_DARK_BACKGROUND_MAX 9
#define DESKTOP_DARK_CONTRAST_MIN 120
#define DESKTOP_FIRMWARE_AREA_MIN 4.5f
#define DESKTOP_DARK_LARGE_AREA_MIN 9.0f
#define DESKTOP_DARK_LARGE_CONTRAST_MIN 70
#define DESKTOP_DARK_INTERIOR_CONTRAST_MIN 40
#define DESKTOP_RELIABLE_LEFT_X 20.0f
#define DESKTOP_RELIABLE_RIGHT_X 168.0f
#define DESKTOP_RELIABLE_TOP_Y 14.0f
#define DESKTOP_DARK_SCENE_MEAN_MAX 10U
#define DESKTOP_CANDIDATE_RAW 0x01U
#define DESKTOP_CANDIDATE_FIRMWARE 0x02U
#define DESKTOP_CANDIDATE_WEAK 0x04U

typedef struct
{
    float x;
    float y;
    float radius;
    unsigned char valid;
} desktop_circle_t;

typedef struct
{
    float cx;
    float cy;
    float width;
    float length;
    float angle;
    unsigned char valid;
} desktop_rect_t;

typedef struct
{
    desktop_circle_t circles[DESKTOP_MAX_CIRCLES];
    unsigned char count;
    desktop_circle_t beacons[DESKTOP_MAX_BEACONS];
    unsigned char beacon_count;
    desktop_rect_t car_lamps[DESKTOP_MAX_CAR_LAMPS];
    unsigned char car_lamp_count;
    desktop_circle_t temporal_beacons[DESKTOP_MAX_BEACONS];
    unsigned char temporal_beacon_count;
    desktop_rect_t temporal_car_lamps[DESKTOP_MAX_CAR_LAMPS];
    unsigned char temporal_car_lamp_count;
} desktop_result_t;

typedef struct
{
    float x;
    float y;
    float vx;
    float vy;
    float radius;
    unsigned char valid;
    unsigned char misses;
} desktop_beacon_track_t;

typedef struct
{
    desktop_circle_t circle;
    int area;
    unsigned char sources;
    unsigned char verified;
    unsigned char border;
} desktop_beacon_candidate_t;

typedef struct
{
    float x;
    float y;
    unsigned char valid;
} desktop_beacon_start_t;

uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
volatile uint8 mt9v03x_finish_flag;
uint16 g_mt9v03x_exp_time;
static desktop_beacon_track_t g_desktop_beacon_track;
static desktop_beacon_start_t g_desktop_beacon_start;
static unsigned char g_desktop_strong_visit[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W];
static uint16_t g_desktop_strong_queue[DESKTOP_IMAGE_H * DESKTOP_IMAGE_W];

uint8 mt9v03x_init(void)
{
    return 0U;
}

void flash_init(void)
{
}

uint8 flash_check(uint32 sector, uint32 page)
{
    (void)sector;
    (void)page;
    return 0U;
}

void flash_read_page(uint32 sector, uint32 page, uint32 *buffer, uint32 length)
{
    (void)sector;
    (void)page;
    memset(buffer, 0, length * sizeof(*buffer));
}

void flash_erase_page(uint32 sector, uint32 page)
{
    (void)sector;
    (void)page;
}

void flash_write_page(uint32 sector, uint32 page, const uint32 *buffer, uint32 length)
{
    (void)sector;
    (void)page;
    (void)buffer;
    (void)length;
}

void beacon_image_init(void)
{
    memset(&g_desktop_beacon_track, 0, sizeof(g_desktop_beacon_track));
    memset(&g_desktop_beacon_start, 0, sizeof(g_desktop_beacon_start));
    image_init();
}

void beacon_image_reset_temporal(void)
{
    memset(&g_desktop_beacon_track, 0, sizeof(g_desktop_beacon_track));
    memset(&g_desktop_beacon_start, 0, sizeof(g_desktop_beacon_start));
    image_algorithm_params_changed();
}

static unsigned char strong_component_near_car(float x, float y)
{
    uint8 index;

    for(index = 0U; index < IMAGE_MAX_CAR_LAMP_COUNT; index++)
    {
        const car_lamp_data *car = &g_image_data.car_lamp_data[index];
        float dx;
        float dy;
        float radius;
        if(image_data_car_lamp_valid(car) == 0U)
        {
            continue;
        }
        dx = x - ((float)DESKTOP_IMAGE_W * 0.5f + car->cx);
        dy = y - ((float)DESKTOP_IMAGE_H * 0.5f + car->cy);
        radius = ((car->length > car->width) ? car->length : car->width) *
                 0.6f + 6.0f;
        if(dx * dx + dy * dy <= radius * radius)
        {
            return 1U;
        }
    }
    return 0U;
}

static unsigned char track_matches_image_point(float x,
                                                float y,
                                                float maximum_distance)
{
    float predict_x;
    float predict_y;
    float dx;
    float dy;

    if(g_desktop_beacon_track.valid == 0U)
    {
        return 0U;
    }
    predict_x = (float)DESKTOP_IMAGE_W * 0.5f -
                (g_desktop_beacon_track.x + g_desktop_beacon_track.vx);
    predict_y = (float)DESKTOP_IMAGE_H * 0.5f +
                g_desktop_beacon_track.y + g_desktop_beacon_track.vy;
    dx = x - predict_x;
    dy = y - predict_y;
    return (dx * dx + dy * dy <=
            maximum_distance * maximum_distance) ? 1U : 0U;
}

static void add_desktop_candidate(
    desktop_beacon_candidate_t candidates[DESKTOP_MAX_CANDIDATES],
    uint8 *count,
    const desktop_beacon_candidate_t *candidate)
{
    uint8 index;

    if((count == NULL) || (candidate == NULL) ||
       (*count >= DESKTOP_MAX_CANDIDATES))
    {
        return;
    }
    for(index = 0U; index < *count; index++)
    {
        float dx = candidates[index].circle.x - candidate->circle.x;
        float dy = candidates[index].circle.y - candidate->circle.y;
        if(dx * dx + dy * dy < 25.0f)
        {
            candidates[index].sources |= candidate->sources;
            candidates[index].verified |= candidate->verified;
            return;
        }
    }
    candidates[*count] = *candidate;
    (*count)++;
}

static float track_area_ratio(int area)
{
    float previous_area;
    float ratio;

    if((area <= 0) ||
       (g_desktop_beacon_track.valid == 0U) ||
       (g_desktop_beacon_track.radius <= 0.0f))
    {
        return 1.0f;
    }
    previous_area = DESKTOP_PI * g_desktop_beacon_track.radius *
                    g_desktop_beacon_track.radius;
    ratio = (float)area / previous_area;
    if(ratio < 1.0f)
    {
        ratio = 1.0f / ratio;
    }
    return ratio;
}

static unsigned char candidate_area_matches_track(
    const desktop_beacon_candidate_t *candidate)
{
    float maximum_ratio;

    if(candidate == NULL)
    {
        return 0U;
    }
    maximum_ratio =
        (g_desktop_beacon_track.misses >= DESKTOP_BEACON_PREDICT_FRAMES) ?
        DESKTOP_REACQUIRE_AREA_RATIO_MAX : DESKTOP_TRACK_AREA_RATIO_MAX;
    return (track_area_ratio(candidate->area) <= maximum_ratio) ? 1U : 0U;
}

static float candidate_track_score(int area, float distance2)
{
    float area_delta = track_area_ratio(area) - 1.0f;
    return distance2 + 64.0f * area_delta * area_delta;
}

static unsigned char point_source_means(
    const unsigned char image[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W],
    float center_x,
    float center_y,
    int *core_mean,
    int *ring_mean)
{
    int x;
    int y;
    int core_sum = 0;
    int core_count = 0;
    int ring_sum = 0;
    int ring_count = 0;

    for(y = (int)center_y - 11; y <= (int)center_y + 11; y++)
    {
        for(x = (int)center_x - 11; x <= (int)center_x + 11; x++)
        {
            float dx;
            float dy;
            float distance2;
            if((x < 0) || (x >= DESKTOP_IMAGE_W) ||
               (y < 0) || (y >= DESKTOP_IMAGE_H))
            {
                continue;
            }
            dx = (float)x - center_x;
            dy = (float)y - center_y;
            distance2 = dx * dx + dy * dy;
            if(distance2 < 4.0f)
            {
                core_sum += image[y][x];
                core_count++;
            }
            else if((distance2 >= 49.0f) && (distance2 < 121.0f))
            {
                ring_sum += image[y][x];
                ring_count++;
            }
        }
    }

    if((core_count == 0) || (ring_count == 0) ||
       (core_mean == NULL) || (ring_mean == NULL))
    {
        return 0U;
    }
    *core_mean = core_sum / core_count;
    *ring_mean = ring_sum / ring_count;
    return 1U;
}

static unsigned char strong_point_source(
    const unsigned char image[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W],
    float center_x,
    float center_y)
{
    int core_mean;
    int ring_mean;

    if(point_source_means(image, center_x, center_y,
                          &core_mean, &ring_mean) == 0U)
    {
        return 0U;
    }
    return (core_mean - ring_mean >=
            DESKTOP_STRONG_CONTRAST_MIN) ? 1U : 0U;
}

static unsigned char firmware_point_source(
    const unsigned char image[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W],
    const beacon_data *source)
{
    float center_x;
    float center_y;
    int x;
    int y;
    int core_mean;
    int ring_mean;
    unsigned char peak = 0U;

    if((source == NULL) || (source->area < DESKTOP_FIRMWARE_AREA_MIN))
    {
        return 0U;
    }
    center_x = (float)DESKTOP_IMAGE_W * 0.5f + source->x;
    center_y = (float)DESKTOP_IMAGE_H * 0.5f + source->y;
    if(point_source_means(image, center_x, center_y,
                          &core_mean, &ring_mean) == 0U)
    {
        return 0U;
    }
    if((ring_mean <= DESKTOP_DARK_BACKGROUND_MAX) &&
       ((core_mean - ring_mean >= DESKTOP_DARK_CONTRAST_MIN) ||
        ((source->area >= DESKTOP_DARK_LARGE_AREA_MIN) &&
         (core_mean - ring_mean >= DESKTOP_DARK_LARGE_CONTRAST_MIN)) ||
        ((center_x >= DESKTOP_RELIABLE_LEFT_X) &&
         (center_x <= DESKTOP_RELIABLE_RIGHT_X) &&
         (center_y >= DESKTOP_RELIABLE_TOP_Y) &&
         (core_mean - ring_mean >=
          DESKTOP_DARK_INTERIOR_CONTRAST_MIN))))
    {
        return 1U;
    }
    if((source->area < DESKTOP_SATURATED_AREA_MIN) ||
       (source->area > DESKTOP_SATURATED_AREA_MAX))
    {
        return 0U;
    }
    for(y = (int)center_y - 3; y <= (int)center_y + 3; y++)
    {
        for(x = (int)center_x - 3; x <= (int)center_x + 3; x++)
        {
            if((x >= 0) && (x < DESKTOP_IMAGE_W) &&
               (y >= 0) && (y < DESKTOP_IMAGE_H) &&
               (image[y][x] > peak))
            {
                peak = image[y][x];
            }
        }
    }
    if(peak < DESKTOP_SATURATED_PEAK_MIN)
    {
        return 0U;
    }
    return (ring_mean >= DESKTOP_SATURATED_BACKGROUND_MIN) ? 1U : 0U;
}

static void find_strong_beacons(
    const unsigned char image[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W],
    desktop_beacon_candidate_t candidates[DESKTOP_MAX_CANDIDATES],
    uint8 *count)
{
    static const signed char dx[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
    static const signed char dy[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };
    int x;
    int y;

    memset(g_desktop_strong_visit, 0, sizeof(g_desktop_strong_visit));
    for(y = 0; y < DESKTOP_IMAGE_H; y++)
    {
        for(x = 0; x < DESKTOP_IMAGE_W; x++)
        {
            unsigned int head = 0U;
            unsigned int tail = 0U;
            int area = 0;
            unsigned int weighted_x = 0U;
            unsigned int weighted_y = 0U;
            unsigned int weight_sum = 0U;
            int min_x = x;
            int max_x = x;
            int min_y = y;
            int max_y = y;
            unsigned char peak = 0U;
            int width;
            int height;
            float center_x;
            float center_y;
            desktop_beacon_candidate_t candidate;

            if((image[y][x] < DESKTOP_STRONG_THRESHOLD) ||
               (g_desktop_strong_visit[y][x] != 0U))
            {
                continue;
            }
            g_desktop_strong_visit[y][x] = 1U;
            g_desktop_strong_queue[tail++] =
                (uint16_t)(y * DESKTOP_IMAGE_W + x);
            while(head < tail)
            {
                uint8 neighbor;
                uint16_t position = g_desktop_strong_queue[head++];
                int current_x = position % DESKTOP_IMAGE_W;
                int current_y = position / DESKTOP_IMAGE_W;
                unsigned char pixel = image[current_y][current_x];

                area++;
                weighted_x += (unsigned int)current_x * pixel;
                weighted_y += (unsigned int)current_y * pixel;
                weight_sum += pixel;
                if(current_x < min_x) min_x = current_x;
                if(current_x > max_x) max_x = current_x;
                if(current_y < min_y) min_y = current_y;
                if(current_y > max_y) max_y = current_y;
                if(pixel > peak) peak = pixel;
                for(neighbor = 0U; neighbor < 8U; neighbor++)
                {
                    int next_x = current_x + dx[neighbor];
                    int next_y = current_y + dy[neighbor];
                    if((next_x < 0) || (next_x >= DESKTOP_IMAGE_W) ||
                       (next_y < 0) || (next_y >= DESKTOP_IMAGE_H) ||
                       (g_desktop_strong_visit[next_y][next_x] != 0U) ||
                       (image[next_y][next_x] < DESKTOP_STRONG_THRESHOLD))
                    {
                        continue;
                    }
                    g_desktop_strong_visit[next_y][next_x] = 1U;
                    g_desktop_strong_queue[tail++] =
                        (uint16_t)(next_y * DESKTOP_IMAGE_W + next_x);
                }
            }

            width = max_x - min_x + 1;
            height = max_y - min_y + 1;
            if((area < DESKTOP_STRONG_AREA_MIN) ||
               (area > DESKTOP_STRONG_AREA_MAX) ||
               (width < 2) || (height < 2) ||
               (width * 2 > height * 3) ||
               (height * 2 > width * 3) ||
               (area * 100 < width * height * 35) ||
               (peak < DESKTOP_STRONG_PEAK_MIN))
            {
                continue;
            }
            center_x = (float)weighted_x / (float)weight_sum;
            center_y = (float)weighted_y / (float)weight_sum;
            if(strong_point_source(image, center_x, center_y) == 0U)
            {
                continue;
            }
            if((strong_component_near_car(center_x, center_y) != 0U) &&
               (track_matches_image_point(center_x, center_y,
                                          DESKTOP_WEAK_MATCH_DISTANCE) == 0U))
            {
                continue;
            }
            memset(&candidate, 0, sizeof(candidate));
            candidate.circle.x = (float)DESKTOP_IMAGE_W * 0.5f - center_x;
            candidate.circle.y = center_y - (float)DESKTOP_IMAGE_H * 0.5f;
            candidate.circle.radius = sqrtf((float)area / DESKTOP_PI);
            candidate.circle.valid = 1U;
            candidate.area = area;
            candidate.sources = DESKTOP_CANDIDATE_RAW;
            candidate.verified = 1U;
            candidate.border = ((min_x <= 2) || (min_y <= 2) ||
                                (max_x >= DESKTOP_IMAGE_W - 3) ||
                                (max_y >= DESKTOP_IMAGE_H - 3)) ? 1U : 0U;
            add_desktop_candidate(candidates, count, &candidate);
        }
    }
}

static int nearest_candidate(
    const desktop_beacon_candidate_t candidates[DESKTOP_MAX_CANDIDATES],
    uint8 count,
    uint8 required_sources,
    uint8 verified_only,
    float x,
    float y,
    float maximum_distance)
{
    int selected = -1;
    uint8 index;
    float maximum_distance2 = maximum_distance * maximum_distance;
    float best_score = 1000000.0f;

    for(index = 0U; index < count; index++)
    {
        if(((candidates[index].sources & required_sources) != required_sources) ||
           ((verified_only != 0U) && (candidates[index].verified == 0U)) ||
           (candidates[index].border != 0U) ||
           (candidate_area_matches_track(&candidates[index]) == 0U))
        {
            continue;
        }
        float dx = candidates[index].circle.x - x;
        float dy = candidates[index].circle.y - y;
        float distance2 = dx * dx + dy * dy;
        float score;
        if(distance2 > maximum_distance2)
        {
            continue;
        }
        score = candidate_track_score(candidates[index].area, distance2);
        if(score <= best_score)
        {
            best_score = score;
            selected = index;
        }
    }
    return selected;
}

static int nearest_firmware_candidate(
    const desktop_beacon_candidate_t candidates[DESKTOP_MAX_CANDIDATES],
    uint8 count,
    float x,
    float y,
    float maximum_distance)
{
    int selected = -1;
    uint8 index;
    float maximum_distance2 = maximum_distance * maximum_distance;
    float best_score = 1000000.0f;

    for(index = 0U; index < count; index++)
    {
        float dx;
        float dy;
        float distance2;
        float score;
        if(((candidates[index].sources &
             DESKTOP_CANDIDATE_FIRMWARE) == 0U) ||
           (candidates[index].border != 0U) ||
           (candidate_area_matches_track(&candidates[index]) == 0U) ||
           ((candidates[index].verified == 0U) &&
            (candidates[index].area <= 8) &&
            ((((float)DESKTOP_IMAGE_W * 0.5f -
               candidates[index].circle.x) < DESKTOP_RELIABLE_LEFT_X) ||
             (((float)DESKTOP_IMAGE_W * 0.5f -
               candidates[index].circle.x) > DESKTOP_RELIABLE_RIGHT_X) ||
             (((float)DESKTOP_IMAGE_H * 0.5f +
               candidates[index].circle.y) < DESKTOP_RELIABLE_TOP_Y))))
        {
            continue;
        }
        dx = candidates[index].circle.x - x;
        dy = candidates[index].circle.y - y;
        distance2 = dx * dx + dy * dy;
        if(distance2 > maximum_distance2)
        {
            continue;
        }
        score = candidate_track_score(candidates[index].area, distance2);
        if(score <= best_score)
        {
            best_score = score;
            selected = index;
        }
    }
    return selected;
}

static void add_track_supported_candidate(
    const unsigned char image[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W],
    desktop_beacon_candidate_t candidates[DESKTOP_MAX_CANDIDATES],
    uint8 *count)
{
    desktop_beacon_candidate_t candidate;
    float predict_x;
    float predict_y;
    float gate;
    float best_score;
    int best_x = -1;
    int best_y = -1;
    int best_area = 0;
    int x;
    int y;

    if(g_desktop_beacon_track.valid == 0U)
    {
        return;
    }
    predict_x = (float)DESKTOP_IMAGE_W * 0.5f -
                (g_desktop_beacon_track.x + g_desktop_beacon_track.vx);
    predict_y = (float)DESKTOP_IMAGE_H * 0.5f +
                g_desktop_beacon_track.y + g_desktop_beacon_track.vy;
    gate = DESKTOP_WEAK_MATCH_DISTANCE;
    best_score = 1000000.0f;

    for(y = (int)(predict_y - gate); y <= (int)(predict_y + gate); y++)
    {
        for(x = (int)(predict_x - gate); x <= (int)(predict_x + gate); x++)
        {
            unsigned char peak;
            int core_mean;
            int ring_mean;
            int bright_area = 0;
            int dx;
            int dy;
            float distance_x;
            float distance_y;
            float distance2;
            float score;

            if((x <= 0) || (x >= DESKTOP_IMAGE_W - 1) ||
               (y <= 0) || (y >= DESKTOP_IMAGE_H - 1))
            {
                continue;
            }
            peak = image[y][x];
            if(peak < DESKTOP_WEAK_PEAK_MIN)
            {
                continue;
            }
            for(dy = -1; dy <= 1; dy++)
            {
                for(dx = -1; dx <= 1; dx++)
                {
                    if(image[y + dy][x + dx] > peak)
                    {
                        peak = 0U;
                    }
                }
            }
            if((peak == 0U) ||
               (point_source_means(image, (float)x, (float)y,
                                   &core_mean, &ring_mean) == 0U) ||
               (ring_mean > DESKTOP_WEAK_RING_MAX) ||
               (core_mean - ring_mean < DESKTOP_WEAK_CONTRAST_MIN))
            {
                continue;
            }
            for(dy = -3; dy <= 3; dy++)
            {
                for(dx = -3; dx <= 3; dx++)
                {
                    int sample_x = x + dx;
                    int sample_y = y + dy;
                    if((sample_x >= 0) && (sample_x < DESKTOP_IMAGE_W) &&
                       (sample_y >= 0) && (sample_y < DESKTOP_IMAGE_H) &&
                       (image[sample_y][sample_x] >= ring_mean + 24))
                    {
                        bright_area++;
                    }
                }
            }
            if(bright_area > DESKTOP_WEAK_AREA_MAX)
            {
                continue;
            }
            distance_x = (float)x - predict_x;
            distance_y = (float)y - predict_y;
            distance2 = distance_x * distance_x + distance_y * distance_y;
            if((distance2 > gate * gate) ||
               (track_area_ratio(bright_area) > DESKTOP_TRACK_AREA_RATIO_MAX))
            {
                continue;
            }
            score = candidate_track_score(bright_area, distance2);
            if(score <= best_score)
            {
                best_score = score;
                best_x = x;
                best_y = y;
                best_area = bright_area;
            }
        }
    }
    if(best_x < 0)
    {
        return;
    }

    memset(&candidate, 0, sizeof(candidate));
    candidate.circle.x = (float)DESKTOP_IMAGE_W * 0.5f - (float)best_x;
    candidate.circle.y = (float)best_y - (float)DESKTOP_IMAGE_H * 0.5f;
    candidate.circle.radius = sqrtf((float)best_area / DESKTOP_PI);
    candidate.circle.valid = 1U;
    candidate.area = best_area;
    candidate.sources = DESKTOP_CANDIDATE_WEAK;
    candidate.verified = 1U;
    add_desktop_candidate(candidates, count, &candidate);
}

static int select_current_beacon(
    const desktop_beacon_candidate_t candidates[DESKTOP_MAX_CANDIDATES],
    uint8 count,
    uint8 dark_scene)
{
    int selected;
    uint8 index;

    if(g_desktop_beacon_track.valid == 0U)
    {
        int largest_area = -1;
        selected = -1;
        if(dark_scene != 0U)
        {
            for(index = 0U; index < count; index++)
            {
                if(((candidates[index].sources & DESKTOP_CANDIDATE_RAW) != 0U) &&
                   (candidates[index].border == 0U) &&
                   (candidate_area_matches_track(&candidates[index]) != 0U) &&
                   (candidates[index].area > largest_area))
                {
                    largest_area = candidates[index].area;
                    selected = index;
                }
            }
            if(selected >= 0)
            {
                memset(&g_desktop_beacon_start, 0,
                       sizeof(g_desktop_beacon_start));
                return selected;
            }
        }
        if(g_desktop_beacon_start.valid != 0U)
        {
            selected = nearest_candidate(
                candidates, count, DESKTOP_CANDIDATE_FIRMWARE,
                1U,
                g_desktop_beacon_start.x, g_desktop_beacon_start.y,
                DESKTOP_START_MATCH_DISTANCE);
            if(selected >= 0)
            {
                g_desktop_beacon_track.x = g_desktop_beacon_start.x;
                g_desktop_beacon_track.y = g_desktop_beacon_start.y;
                g_desktop_beacon_track.vx =
                    candidates[selected].circle.x - g_desktop_beacon_start.x;
                g_desktop_beacon_track.vy =
                    candidates[selected].circle.y - g_desktop_beacon_start.y;
                g_desktop_beacon_track.radius =
                    candidates[selected].circle.radius;
                g_desktop_beacon_track.valid = 1U;
                g_desktop_beacon_track.misses = 0U;
                memset(&g_desktop_beacon_start, 0,
                       sizeof(g_desktop_beacon_start));
                return selected;
            }
        }
        for(index = 0U; index < count; index++)
        {
            if(((candidates[index].sources &
                 DESKTOP_CANDIDATE_FIRMWARE) == 0U) ||
               (candidates[index].verified == 0U) ||
               (candidates[index].border != 0U) ||
               (candidates[index].area <= largest_area))
            {
                continue;
            }
            largest_area = candidates[index].area;
            selected = index;
        }
        if(selected >= 0)
        {
            g_desktop_beacon_start.x = candidates[selected].circle.x;
            g_desktop_beacon_start.y = candidates[selected].circle.y;
            g_desktop_beacon_start.valid = 1U;
        }
        else
        {
            memset(&g_desktop_beacon_start, 0,
                   sizeof(g_desktop_beacon_start));
        }
        return -1;
    }
    else
    {
        float predict_x = g_desktop_beacon_track.x + g_desktop_beacon_track.vx;
        float predict_y = g_desktop_beacon_track.y + g_desktop_beacon_track.vy;
        float speed = sqrtf(g_desktop_beacon_track.vx *
                            g_desktop_beacon_track.vx +
                            g_desktop_beacon_track.vy *
                            g_desktop_beacon_track.vy);
        float gate = speed * 1.8f + 10.0f;
        float raw_gate;

        if(gate < b0_match_distance)
        {
            gate = b0_match_distance;
        }
        raw_gate = (gate < 36.0f) ? 36.0f : gate;
        memset(&g_desktop_beacon_start, 0, sizeof(g_desktop_beacon_start));
        selected = nearest_candidate(
            candidates, count,
            DESKTOP_CANDIDATE_RAW | DESKTOP_CANDIDATE_FIRMWARE,
            0U,
            predict_x, predict_y, raw_gate);
        if(selected >= 0)
        {
            return selected;
        }
        selected = nearest_candidate(
            candidates, count, DESKTOP_CANDIDATE_RAW,
            0U,
            predict_x, predict_y, raw_gate);
        if(selected >= 0)
        {
            return selected;
        }
        selected = nearest_firmware_candidate(
            candidates, count, predict_x, predict_y, gate);
        if(selected >= 0)
        {
            return selected;
        }
        selected = nearest_candidate(
            candidates, count, DESKTOP_CANDIDATE_WEAK,
            0U, predict_x, predict_y, DESKTOP_WEAK_MATCH_DISTANCE);
        if(selected >= 0)
        {
            return selected;
        }
        if(g_desktop_beacon_track.misses < DESKTOP_BEACON_PREDICT_FRAMES)
        {
            return -1;
        }
        if(dark_scene != 0U)
        {
            float best_score = 1000000.0f;
            for(index = 0U; index < count; index++)
            {
                float dx;
                float dy;
                float score;
                if(((candidates[index].sources & DESKTOP_CANDIDATE_RAW) != 0U) &&
                   (candidates[index].border == 0U) &&
                   (candidate_area_matches_track(&candidates[index]) != 0U))
                {
                    dx = candidates[index].circle.x - predict_x;
                    dy = candidates[index].circle.y - predict_y;
                    score = candidate_track_score(
                        candidates[index].area, dx * dx + dy * dy);
                    if(score < best_score)
                    {
                        best_score = score;
                        selected = index;
                    }
                }
            }
            if(selected >= 0)
            {
                return selected;
            }
            selected = nearest_firmware_candidate(
                candidates, count, predict_x, predict_y, 300.0f);
            if(selected >= 0)
            {
                return selected;
            }
        }
        selected = -1;
        {
            float best_score = 1000000.0f;
            for(index = 0U; index < count; index++)
            {
                float dx;
                float dy;
                float distance2;
                float score;
                if(((candidates[index].sources &
                     DESKTOP_CANDIDATE_FIRMWARE) == 0U) ||
                   (candidates[index].verified == 0U) ||
                   (candidates[index].border != 0U) ||
                   (candidate_area_matches_track(&candidates[index]) == 0U))
                {
                    continue;
                }
                dx = candidates[index].circle.x - predict_x;
                dy = candidates[index].circle.y - predict_y;
                distance2 = dx * dx + dy * dy;
                score = candidate_track_score(
                    candidates[index].area, distance2);
                if(score < best_score)
                {
                    best_score = score;
                    selected = index;
                }
            }
        }
        return selected;
    }
}

static void update_desktop_beacon_track(
    const desktop_circle_t *measurement,
    desktop_result_t *result)
{
    desktop_circle_t *output;

    if(measurement != NULL)
    {
        if(g_desktop_beacon_track.valid != 0U)
        {
            float alpha = filter_vel_alpha;
            float dx = measurement->x - g_desktop_beacon_track.x;
            float dy = measurement->y - g_desktop_beacon_track.y;
            g_desktop_beacon_track.vx =
                (1.0f - alpha) * g_desktop_beacon_track.vx + alpha * dx;
            g_desktop_beacon_track.vy =
                (1.0f - alpha) * g_desktop_beacon_track.vy + alpha * dy;
        }
        else
        {
            g_desktop_beacon_track.valid = 1U;
            g_desktop_beacon_track.vx = 0.0f;
            g_desktop_beacon_track.vy = 0.0f;
        }
        g_desktop_beacon_track.x = measurement->x;
        g_desktop_beacon_track.y = measurement->y;
        g_desktop_beacon_track.radius = measurement->radius;
        g_desktop_beacon_track.misses = 0U;
    }
    else if(g_desktop_beacon_track.valid != 0U)
    {
        g_desktop_beacon_track.x += g_desktop_beacon_track.vx;
        g_desktop_beacon_track.y += g_desktop_beacon_track.vy;
        g_desktop_beacon_track.vx *= 0.9f;
        g_desktop_beacon_track.vy *= 0.9f;
        g_desktop_beacon_track.misses++;
        if(g_desktop_beacon_track.misses > DESKTOP_BEACON_RETAIN_FRAMES)
        {
            memset(&g_desktop_beacon_track, 0, sizeof(g_desktop_beacon_track));
            return;
        }
        if(g_desktop_beacon_track.misses > DESKTOP_BEACON_PREDICT_FRAMES)
        {
            return;
        }
    }
    else
    {
        memset(&g_desktop_beacon_track, 0, sizeof(g_desktop_beacon_track));
        return;
    }

    output = &result->temporal_beacons[0];
    output->x = g_desktop_beacon_track.x;
    output->y = g_desktop_beacon_track.y;
    output->radius = g_desktop_beacon_track.radius;
    output->valid = 1U;
    result->temporal_beacon_count = 1U;
}

static unsigned char find_bottom_edge_car_lamp(
    const unsigned char image[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W],
    desktop_rect_t *lamp)
{
    static const signed char dx[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
    static const signed char dy[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };
    float best_score = 0.0f;
    int threshold = car_lamp_binary_threshold;
    int x;
    int y;
    unsigned char found = 0U;

    if((image == NULL) || (lamp == NULL))
    {
        return 0U;
    }
    if(threshold < 0) threshold = 0;
    if(threshold > 255) threshold = 255;
    memset(g_desktop_strong_visit, 0, sizeof(g_desktop_strong_visit));

    for(y = 0; y < DESKTOP_IMAGE_H; y++)
    {
        for(x = 0; x < DESKTOP_IMAGE_W; x++)
        {
            unsigned int head = 0U;
            unsigned int tail = 0U;
            int area = 0;
            int max_y = y;
            float sum_x = 0.0f;
            float sum_y = 0.0f;
            float sum_xx = 0.0f;
            float sum_yy = 0.0f;
            float sum_xy = 0.0f;
            float center_x;
            float center_y;
            float var_x;
            float var_y;
            float cov_xy;
            float trace;
            float determinant;
            float discriminant;
            float major_eigenvalue;
            float minor_eigenvalue;
            float major;
            float minor;
            float elongation;
            float score;

            if((image[y][x] < threshold) ||
               (g_desktop_strong_visit[y][x] != 0U))
            {
                continue;
            }
            g_desktop_strong_visit[y][x] = 1U;
            g_desktop_strong_queue[tail++] =
                (uint16_t)(y * DESKTOP_IMAGE_W + x);
            while(head < tail)
            {
                uint8 neighbor;
                uint16_t position = g_desktop_strong_queue[head++];
                int current_x = position % DESKTOP_IMAGE_W;
                int current_y = position / DESKTOP_IMAGE_W;

                area++;
                sum_x += (float)current_x;
                sum_y += (float)current_y;
                sum_xx += (float)(current_x * current_x);
                sum_yy += (float)(current_y * current_y);
                sum_xy += (float)(current_x * current_y);
                if(current_y > max_y) max_y = current_y;
                for(neighbor = 0U; neighbor < 8U; neighbor++)
                {
                    int next_x = current_x + dx[neighbor];
                    int next_y = current_y + dy[neighbor];
                    if((next_x < 0) || (next_x >= DESKTOP_IMAGE_W) ||
                       (next_y < 0) || (next_y >= DESKTOP_IMAGE_H) ||
                       (g_desktop_strong_visit[next_y][next_x] != 0U) ||
                       (image[next_y][next_x] < threshold))
                    {
                        continue;
                    }
                    g_desktop_strong_visit[next_y][next_x] = 1U;
                    g_desktop_strong_queue[tail++] =
                        (uint16_t)(next_y * DESKTOP_IMAGE_W + next_x);
                }
            }
            if((max_y != DESKTOP_IMAGE_H - 1) ||
               (area < car_lamp_min_area) ||
               (area > car_lamp_max_area))
            {
                continue;
            }

            center_x = sum_x / (float)area;
            center_y = sum_y / (float)area;
            var_x = sum_xx / (float)area - center_x * center_x;
            var_y = sum_yy / (float)area - center_y * center_y;
            cov_xy = sum_xy / (float)area - center_x * center_y;
            trace = var_x + var_y;
            determinant = var_x * var_y - cov_xy * cov_xy;
            discriminant = trace * trace * 0.25f - determinant;
            if(discriminant < 0.0f) discriminant = 0.0f;
            major_eigenvalue = trace * 0.5f + sqrtf(discriminant);
            minor_eigenvalue = trace * 0.5f - sqrtf(discriminant);
            if(minor_eigenvalue < 0.0f) minor_eigenvalue = 0.0f;
            major = 4.0f * sqrtf(major_eigenvalue + 0.0001f);
            minor = 4.0f * sqrtf(minor_eigenvalue + 0.0001f);
            if(minor < 1.0f) minor = 1.0f;
            elongation = major / minor;

            if((center_y < (float)DESKTOP_IMAGE_H / 3.0f) ||
               (elongation < car_lamp_min_elongation) ||
               (major < car_lamp_min_length) ||
               ((minor < car_lamp_min_width) &&
                ((minor < car_lamp_narrow_min_width) ||
                 (elongation < car_lamp_narrow_min_elongation))))
            {
                continue;
            }
            score = (float)area * elongation;
            if((found != 0U) && (score <= best_score))
            {
                continue;
            }

            lamp->cx = (float)DESKTOP_IMAGE_W * 0.5f - center_x;
            lamp->cy = center_y - (float)DESKTOP_IMAGE_H * 0.5f;
            lamp->width = minor;
            lamp->length = major;
            lamp->angle = 0.5f * atan2f(2.0f * cov_xy, var_x - var_y) *
                          180.0f / DESKTOP_PI;
            lamp->valid = 1U;
            best_score = score;
            found = 1U;
        }
    }
    return found;
}

void beacon_image_process(
    const unsigned char image[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W],
    desktop_result_t *result)
{
    desktop_beacon_candidate_t candidates[DESKTOP_MAX_CANDIDATES];
    uint8 candidate_count = 0U;
    uint8 index;
    uint32 scene_sum = 0U;

    if((image == NULL) || (result == NULL))
    {
        return;
    }
    memset(result, 0, sizeof(*result));
    memcpy(mt9v03x_image, image, sizeof(mt9v03x_image));
    mt9v03x_finish_flag = 1U;
    image_update();

#if IMAGE_ALGORITHM_BUILD_ID >= 0x20260905UL
    for(index = 0U; index < IMAGE_MAX_BEACON_COUNT; index++)
    {
        const beacon_data *source = &g_image_data.beacon_data[index];
        desktop_circle_t *target;

        if(image_data_beacon_valid(source) == 0U)
        {
            continue;
        }
        target = &result->beacons[result->beacon_count++];
        target->x = -source->x;
        target->y = source->y;
        target->radius = sqrtf(source->area / DESKTOP_PI);
        target->valid = 1U;
        result->circles[result->count++] = *target;
    }
    for(index = 0U; index < IMAGE_MAX_CAR_LAMP_COUNT; index++)
    {
        const car_lamp_data *source = &g_image_data.car_lamp_data[index];
        desktop_rect_t *target;

        if(image_data_car_lamp_valid(source) == 0U)
        {
            continue;
        }
        target = &result->car_lamps[result->car_lamp_count++];
        target->cx = -source->cx;
        target->cy = source->cy;
        target->width = source->width;
        target->length = source->length;
        target->angle = source->angle;
        target->valid = 1U;
    }
    return;
#endif

    memset(candidates, 0, sizeof(candidates));
    find_strong_beacons(image, candidates, &candidate_count);
    add_track_supported_candidate(image, candidates, &candidate_count);
    for(index = 0U; index < DESKTOP_IMAGE_H; index++)
    {
        uint16 x;
        for(x = 0U; x < DESKTOP_IMAGE_W; x++)
        {
            scene_sum += image[index][x];
        }
    }

    for(index = 0U; index < IMAGE_MAX_BEACON_COUNT; index++)
    {
        const beacon_data *source = &g_image_data.beacon_data[index];
        desktop_beacon_candidate_t candidate;
        if(image_data_beacon_valid(source) == 0U)
        {
            continue;
        }
        memset(&candidate, 0, sizeof(candidate));
        candidate.circle.x = -source->x;
        candidate.circle.y = source->y;
        candidate.circle.radius = sqrtf(source->area / DESKTOP_PI);
        candidate.circle.valid = 1U;
        candidate.area = (int)(source->area + 0.5f);
        candidate.sources = DESKTOP_CANDIDATE_FIRMWARE;
        candidate.verified = firmware_point_source(image, source);
        candidate.border = ((((float)DESKTOP_IMAGE_W * 0.5f + source->x) <= 2.0f) ||
                            (((float)DESKTOP_IMAGE_H * 0.5f + source->y) <= 2.0f) ||
                            (((float)DESKTOP_IMAGE_W * 0.5f + source->x) >=
                             (float)DESKTOP_IMAGE_W - 3.0f) ||
                            (((float)DESKTOP_IMAGE_H * 0.5f + source->y) >=
                             (float)DESKTOP_IMAGE_H - 3.0f)) ? 1U : 0U;
        add_desktop_candidate(candidates, &candidate_count, &candidate);
    }

    {
        int selected = select_current_beacon(
            candidates, candidate_count,
            (scene_sum / (DESKTOP_IMAGE_W * DESKTOP_IMAGE_H) <
             DESKTOP_DARK_SCENE_MEAN_MAX) ? 1U : 0U);
        if(selected >= 0)
        {
            result->beacons[0] = candidates[selected].circle;
            result->circles[0] = candidates[selected].circle;
            result->beacon_count = 1U;
            result->count = 1U;
            update_desktop_beacon_track(&result->beacons[0], result);
        }
        else
        {
            update_desktop_beacon_track(NULL, result);
        }
    }

    for(index = 0U; index < IMAGE_MAX_CAR_LAMP_COUNT; index++)
    {
        const car_lamp_data *source = &g_image_data.car_lamp_data[index];
        desktop_rect_t *target;
        if(image_data_car_lamp_valid(source) == 0U)
        {
            continue;
        }
        target = &result->car_lamps[result->car_lamp_count++];
        target->cx = -source->cx;
        target->cy = source->cy;
        target->width = source->width;
        target->length = source->length;
        target->angle = source->angle;
        target->valid = 1U;
    }
    if(result->car_lamp_count == 0U)
    {
        desktop_rect_t lamp;
        memset(&lamp, 0, sizeof(lamp));
        if(find_bottom_edge_car_lamp(image, &lamp) != 0U)
        {
            result->car_lamps[0] = lamp;
            result->car_lamp_count = 1U;
        }
    }
}

uint32_t beacon_image_debug_build_id(void)
{
    return IMAGE_ALGORITHM_BUILD_ID;
}

uint16_t beacon_image_debug_parameter_count(void)
{
    return image_param_count();
}

int beacon_image_debug_parameter_info(uint16_t index,
                                      uint16_t *parameter_id,
                                      uint8_t *type,
                                      float *minimum,
                                      float *maximum)
{
    return image_param_info(index, parameter_id, type, minimum, maximum);
}

int beacon_image_debug_parameter_get(uint8_t type,
                                     uint16_t parameter_id,
                                     uint32_t *value_bits)
{
    return image_param_get(type, parameter_id, value_bits);
}

int beacon_image_debug_parameter_set(uint8_t type,
                                     uint16_t parameter_id,
                                     uint32_t value_bits,
                                     uint32_t *actual_bits)
{
    return image_param_set(type, parameter_id, value_bits, actual_bits);
}
