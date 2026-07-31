#include <math.h>
#include <stdint.h>
#include <string.h>

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"
#include "Image/image.h"
#include "Image/image_horizon.h"

#define DESKTOP_IMAGE_W 188
#define DESKTOP_IMAGE_H 120
#define DESKTOP_MAX_CIRCLES 8
#define DESKTOP_MAX_BEACONS 8
#define DESKTOP_MAX_CAR_LAMPS 4
#define DESKTOP_MAX_CANDIDATES IMAGE_MAX_BEACON_COUNT
#define DESKTOP_PI 3.1415926f
#define DESKTOP_BEACON_PREDICT_FRAMES 2U
#define DESKTOP_TRACK_GATE 36.0f
#define DESKTOP_TRACK_STATE_SEARCH 0U
#define DESKTOP_TRACK_STATE_TRACK 1U
#define DESKTOP_TRACK_STATE_COAST 2U
#define DESKTOP_HORIZON_TOLERANCE_PX 2.0f
#define DESKTOP_HORIZON_SMALL_BAND_PX 6.0f
#define DESKTOP_NEAR_TINY_AREA_MAX 8.5f
#define DESKTOP_NEAR_TINY_DEPTH_MIN_PX 20.0f
#define DESKTOP_REFLECTION_BG_MIN 25
#define DESKTOP_REFLECTION_BG_MAX 50
#define DESKTOP_REFLECTION_OUTER_10_MIN 90
#define DESKTOP_REFLECTION_OUTER_20_MIN 45
#define DESKTOP_REFLECTION_OUTER_30_MAX 15

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
    desktop_circle_t candidate_beacons[DESKTOP_MAX_BEACONS];
    unsigned char candidate_beacon_count;
} desktop_result_t;

typedef struct
{
    float x;
    float y;
    float vx;
    float vy;
    float radius;
    unsigned char state;
    unsigned char misses;
} desktop_beacon_track_t;

typedef struct
{
    desktop_circle_t circle;
    float area;
} desktop_beacon_candidate_t;

typedef struct
{
    float roll_deg;
    float pitch_deg;
    float height_mm;
    unsigned char board_id;
    unsigned char attitude_valid;
    unsigned char height_valid;
} desktop_telemetry_t;

uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
volatile uint8 mt9v03x_finish_flag;
uint16 g_mt9v03x_exp_time;

static desktop_beacon_track_t g_desktop_beacon_track;
static desktop_telemetry_t g_desktop_telemetry;

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

void flash_write_page(uint32 sector, uint32 page,
                      const uint32 *buffer, uint32 length)
{
    (void)sector;
    (void)page;
    (void)buffer;
    (void)length;
}

void beacon_image_init(void)
{
    memset(&g_desktop_beacon_track, 0, sizeof(g_desktop_beacon_track));
    memset(&g_desktop_telemetry, 0, sizeof(g_desktop_telemetry));
    image_horizon_init();
    image_init();
}

void beacon_image_reset_temporal(void)
{
    memset(&g_desktop_beacon_track, 0, sizeof(g_desktop_beacon_track));
    memset(&g_desktop_telemetry, 0, sizeof(g_desktop_telemetry));
    image_algorithm_params_changed();
}

void beacon_image_set_telemetry(unsigned char board_id,
                                float roll_deg,
                                float pitch_deg,
                                float height_mm,
                                unsigned char attitude_valid,
                                unsigned char height_valid)
{
    g_desktop_telemetry.board_id = board_id;
    g_desktop_telemetry.roll_deg = roll_deg;
    g_desktop_telemetry.pitch_deg = pitch_deg;
    g_desktop_telemetry.height_mm = height_mm;
    g_desktop_telemetry.attitude_valid = attitude_valid;
    g_desktop_telemetry.height_valid = height_valid;
}

static unsigned char candidate_embedded_in_structure(
    const unsigned char image[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W],
    float center_x,
    float center_y,
    float area)
{
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
    unsigned char ring_histogram[256];
    int dx;
    int dy;

    if(area > 8.5f)
    {
        return 0U;
    }
    cx = (int)(center_x + 0.5f);
    cy = (int)(center_y + 0.5f);
    memset(ring_histogram, 0, sizeof(ring_histogram));
    for(dy = -12; dy <= 12; dy++)
    {
        for(dx = -12; dx <= 12; dx++)
        {
            int x = cx + dx;
            int y = cy + dy;
            int radius2 = dx * dx + dy * dy;
            int pixel;

            if((radius2 > 144) || (x < 0) || (x >= DESKTOP_IMAGE_W) ||
               (y < 0) || (y >= DESKTOP_IMAGE_H))
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
        int background = 0;
        int rank = ring_count / 2;

        for(background = 0; background < 256; background++)
        {
            cumulative += ring_histogram[background];
            if(cumulative > rank)
            {
                break;
            }
        }
        if((background >= DESKTOP_REFLECTION_BG_MIN) &&
           (background <= DESKTOP_REFLECTION_BG_MAX))
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
                       (x < 0) || (x >= DESKTOP_IMAGE_W) ||
                       (y < 0) || (y >= DESKTOP_IMAGE_H))
                    {
                        continue;
                    }
                    pixel = image[y][x];
                    if(pixel >= background + 10) relative_outer10++;
                    if(pixel >= background + 20) relative_outer20++;
                    if(pixel >= background + 30) relative_outer30++;
                }
            }
            if((relative_outer10 >= DESKTOP_REFLECTION_OUTER_10_MIN) &&
               (relative_outer20 >= DESKTOP_REFLECTION_OUTER_20_MIN) &&
               (relative_outer30 <= DESKTOP_REFLECTION_OUTER_30_MAX))
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

static unsigned char candidate_matches_horizon_geometry(float center_x,
                                                         float center_y,
                                                         float area)
{
    int column = (int)(center_x + 0.5f);
    float depth;

    if((g_image_horizon_valid == 0U) ||
       (column < 0) || (column >= DESKTOP_IMAGE_W) ||
       (g_image_horizon_column_valid[column] == 0U))
    {
        return 1U;
    }

    depth = center_y - g_image_horizon_y[column];
    if(depth < -DESKTOP_HORIZON_TOLERANCE_PX)
    {
        return 0U;
    }
    if((depth <= DESKTOP_HORIZON_SMALL_BAND_PX) &&
       (area > DESKTOP_NEAR_TINY_AREA_MAX))
    {
        return 0U;
    }
    if((area <= DESKTOP_NEAR_TINY_AREA_MAX) &&
       (depth >= DESKTOP_NEAR_TINY_DEPTH_MIN_PX))
    {
        return 0U;
    }
    return 1U;
}

static unsigned char collect_candidates(
    const unsigned char image[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W],
    desktop_beacon_candidate_t candidates[DESKTOP_MAX_CANDIDATES])
{
    unsigned char count = 0U;
    uint8 index;

    memset(candidates, 0,
           sizeof(desktop_beacon_candidate_t) * DESKTOP_MAX_CANDIDATES);
    for(index = 0U; index < IMAGE_MAX_BEACON_COUNT; index++)
    {
        const beacon_data *source = &g_image_data.beacon_data[index];
        desktop_beacon_candidate_t *candidate;
        float image_x;
        float image_y;

        if((image_data_beacon_valid(source) == 0U) ||
           (count >= DESKTOP_MAX_CANDIDATES))
        {
            continue;
        }
        image_x = (float)DESKTOP_IMAGE_W * 0.5f + source->x;
        image_y = (float)DESKTOP_IMAGE_H * 0.5f + source->y;
        if(candidate_embedded_in_structure(
               image, image_x, image_y, source->area) != 0U)
        {
            continue;
        }
        if(candidate_matches_horizon_geometry(
               image_x, image_y, source->area) == 0U)
        {
            continue;
        }
        candidate = &candidates[count];
        candidate->circle.x = -source->x;
        candidate->circle.y = source->y;
        candidate->circle.radius = sqrtf(source->area / DESKTOP_PI);
        candidate->circle.valid = 1U;
        candidate->area = source->area;
        count++;
    }
    return count;
}

static int select_candidate(
    const desktop_beacon_candidate_t candidates[DESKTOP_MAX_CANDIDATES],
    unsigned char count)
{
    int selected = 0;
    unsigned char index;

    if(count == 0U)
    {
        return -1;
    }
    for(index = 1U; index < count; index++)
    {
        if(candidates[index].area > candidates[selected].area)
        {
            selected = index;
        }
    }
    return selected;
}

static int select_secondary_candidate(
    const desktop_beacon_candidate_t candidates[DESKTOP_MAX_CANDIDATES],
    unsigned char count,
    int primary)
{
    int selected = -1;
    unsigned char index;

    for(index = 0U; index < count; index++)
    {
        if(index == (unsigned char)primary)
        {
            continue;
        }
        if((selected < 0) ||
           (candidates[index].area > candidates[selected].area))
        {
            selected = index;
        }
    }
    return selected;
}

static unsigned char secondary_candidate_reliable(
    const unsigned char image[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W],
    const desktop_beacon_candidate_t *candidate)
{
    int center_x;
    int center_y;
    int x;
    int y;
    unsigned char peak = 0U;

    if((image == NULL) || (candidate == NULL))
    {
        return 0U;
    }
    if(candidate->area > 8.0f)
    {
        return 1U;
    }
    center_x = (int)((float)DESKTOP_IMAGE_W * 0.5f -
                     candidate->circle.x + 0.5f);
    center_y = (int)((float)DESKTOP_IMAGE_H * 0.5f +
                     candidate->circle.y + 0.5f);
    for(y = center_y - 3; y <= center_y + 3; y++)
    {
        for(x = center_x - 3; x <= center_x + 3; x++)
        {
            if((x >= 0) && (x < DESKTOP_IMAGE_W) &&
               (y >= 0) && (y < DESKTOP_IMAGE_H) &&
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
    return ((center_y >= DESKTOP_IMAGE_H / 8) &&
            (center_y <= DESKTOP_IMAGE_H * 2 / 3)) ? 1U : 0U;
}

static void accept_measurement(desktop_beacon_track_t *track,
                               const desktop_circle_t *measurement)
{
    if((track == NULL) || (measurement == NULL))
    {
        return;
    }
    if(track->state != DESKTOP_TRACK_STATE_SEARCH)
    {
        float dx = measurement->x - track->x;
        float dy = measurement->y - track->y;
        if(dx * dx + dy * dy > DESKTOP_TRACK_GATE * DESKTOP_TRACK_GATE)
        {
            track->vx = 0.0f;
            track->vy = 0.0f;
        }
        else
        {
            track->vx =
                (1.0f - filter_vel_alpha) * track->vx +
                filter_vel_alpha * dx;
            track->vy =
                (1.0f - filter_vel_alpha) * track->vy +
                filter_vel_alpha * dy;
        }
    }
    else
    {
        track->vx = 0.0f;
        track->vy = 0.0f;
    }
    track->x = measurement->x;
    track->y = measurement->y;
    track->radius = measurement->radius;
    track->state = DESKTOP_TRACK_STATE_TRACK;
    track->misses = 0U;
}

static void write_prediction(desktop_result_t *result)
{
    desktop_circle_t *output;

    if(g_desktop_beacon_track.state == DESKTOP_TRACK_STATE_SEARCH)
    {
        return;
    }
    if(g_desktop_beacon_track.misses >= DESKTOP_BEACON_PREDICT_FRAMES)
    {
        memset(&g_desktop_beacon_track, 0, sizeof(g_desktop_beacon_track));
        return;
    }
    g_desktop_beacon_track.x += g_desktop_beacon_track.vx;
    g_desktop_beacon_track.y += g_desktop_beacon_track.vy;
    g_desktop_beacon_track.vx *= 0.9f;
    g_desktop_beacon_track.vy *= 0.9f;
    g_desktop_beacon_track.misses++;
    g_desktop_beacon_track.state = DESKTOP_TRACK_STATE_COAST;

    output = &result->temporal_beacons[0];
    output->x = g_desktop_beacon_track.x;
    output->y = g_desktop_beacon_track.y;
    output->radius = g_desktop_beacon_track.radius;
    output->valid = 1U;
    result->temporal_beacon_count = 1U;
}

void beacon_image_process(
    const unsigned char image[DESKTOP_IMAGE_H][DESKTOP_IMAGE_W],
    desktop_result_t *result)
{
    desktop_beacon_candidate_t candidates[DESKTOP_MAX_CANDIDATES];
    unsigned char candidate_count;
    int selected;
    int secondary;
    uint8 index;

    if((image == NULL) || (result == NULL))
    {
        return;
    }
    memset(result, 0, sizeof(*result));
    image_horizon_update(g_desktop_telemetry.board_id,
                         g_desktop_telemetry.roll_deg,
                         g_desktop_telemetry.pitch_deg,
                         g_desktop_telemetry.height_mm,
                         g_desktop_telemetry.attitude_valid,
                         g_desktop_telemetry.height_valid);
    memcpy(mt9v03x_image, image, sizeof(mt9v03x_image));
    mt9v03x_finish_flag = 1U;
    image_update();

    candidate_count = collect_candidates(image, candidates);
    result->candidate_beacon_count = candidate_count;
    for(index = 0U; index < candidate_count; index++)
    {
        result->candidate_beacons[index] = candidates[index].circle;
    }
    selected = select_candidate(candidates, candidate_count);
    if(selected >= 0)
    {
        result->beacons[0] = candidates[selected].circle;
        result->circles[0] = candidates[selected].circle;
        result->beacon_count = 1U;
        result->count = 1U;
        accept_measurement(&g_desktop_beacon_track,
                           &result->beacons[0]);

        secondary = select_secondary_candidate(candidates,
                                               candidate_count,
                                               selected);
        if((secondary >= 0) &&
           (secondary_candidate_reliable(
                image, &candidates[secondary]) != 0U))
        {
            result->beacons[1] = candidates[secondary].circle;
            result->circles[1] = candidates[secondary].circle;
            result->beacon_count = 2U;
            result->count = 2U;
        }
    }
    else
    {
        write_prediction(result);
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
}

unsigned char beacon_image_debug_horizon(
    float y[DESKTOP_IMAGE_W],
    unsigned char column_valid[DESKTOP_IMAGE_W])
{
    if((y == NULL) || (column_valid == NULL))
    {
        return 0U;
    }
    memset(column_valid, 0, DESKTOP_IMAGE_W * sizeof(column_valid[0]));
    if(g_image_horizon_valid == 0U)
    {
        return 0U;
    }
    memcpy(y, g_image_horizon_y, DESKTOP_IMAGE_W * sizeof(y[0]));
    memcpy(column_valid,
           g_image_horizon_column_valid,
           DESKTOP_IMAGE_W * sizeof(column_valid[0]));
    return 1U;
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
