#include "beacon_fusion.h"

#include <math.h>
#include <string.h>

#define BEACON_FUSION_PI                         (3.1415926f)
#define BEACON_FUSION_HALF_DIAGONAL_PX           (111.5168f)
#define BEACON_FUSION_MAX_IMAGE_ANGLE_RAD        (1.48353f)
#define BEACON_FUSION_IMAGE_WIDTH_PX             (188.0f)
#define BEACON_FUSION_IMAGE_HEIGHT_PX            (120.0f)
#define BEACON_FUSION_IMAGE_CENTER_X_PX          (94.0f)
#define BEACON_FUSION_IMAGE_CENTER_Y_PX          (60.0f)
#define BEACON_FUSION_ORIGIN_PROJ_X_PX           (94.0f)
#define BEACON_FUSION_ORIGIN_PROJ_Y_PX           (100.0f)
#define BEACON_FUSION_MIN_RADIUS_PX              (1.0f)
#define BEACON_FUSION_MIN_GROUND_RAY_Z           (0.02f)
#define BEACON_FUSION_FALLBACK_RANGE_MM          (900.0f)
#define BEACON_FUSION_MISSING_RANGE_PROXY        (5000.0f)
#define BEACON_FUSION_FILTER_ALPHA               (0.35f)
#define BEACON_FUSION_AUTO_MAX_COUNT             (3U)
#define BEACON_FUSION_AUTO_PROMOTE_TICKS         (1U)
#define BEACON_FUSION_AUTO_GLOBAL_THREE_TICKS    (50U)
#define BEACON_FUSION_AUTO_THREE_DEMOTE_TICKS    (50U)
#define BEACON_FUSION_AUTO_TWO_DEMOTE_TICKS      (80U)
#define BEACON_FUSION_AUTO_BEARING_CLUSTER_DEG   (25.0f)
#define BEACON_FUSION_AUTO_GLOBAL_THREE_OBS      (3U)
#define BEACON_FUSION_AUTO_LOCAL_SEP_DEG         (25.0f)
#define BEACON_FUSION_AUTO_NO_OBS_TICKS          (20U)
#define BEACON_FUSION_TRACK_MATCH_DEG            (35.0f)
#define BEACON_FUSION_RADIUS_RANGE_SCALE         (2200.0f)
#define BEACON_FUSION_MIN_RANGE_PROXY            (100.0f)
#define BEACON_FUSION_MAX_RANGE_PROXY            (5000.0f)

typedef struct
{
    float x;
    float y;
    float z;
} beacon_fusion_vec3_t;

typedef struct
{
    beacon_fusion_vec3_t forward;
    beacon_fusion_vec3_t right;
    beacon_fusion_vec3_t down;
} beacon_fusion_camera_model_t;

typedef struct
{
    uint8 camera_id;
    uint8 slot_id;
    float x;
    float y;
    float radius;
    float bearing_deg;
    float range_proxy;
    float x_body;
    float y_body;
    float control_x;
    float control_y;
    float weight;
} beacon_fusion_observation_t;

typedef struct
{
    beacon_fusion_observation_t item[BEACON_FUSION_CAMERA_TARGETS];
    uint8 count;
} beacon_fusion_camera_bucket_t;

typedef struct
{
    float bearing_deg;
    float weight;
    uint8 camera_mask;
    uint8 count;
} beacon_fusion_bearing_cluster_t;

typedef struct
{
    beacon_fusion_observation_t member[BEACON_FUSION_CAMERA_COUNT];
    uint8 member_count;
    uint8 camera_mask;
    float bearing_deg;
    float weight;
    float radius_max;
} beacon_fusion_observation_cluster_t;

beacon_fusion_result_t g_beacon_fusion_result;

static uint8 s_auto_count_state;
static uint8 s_auto_max_count;
static uint8 s_expected_count;
static uint8 s_seen_two_ticks;
static uint8 s_seen_local_three_ticks;
static uint8 s_seen_global_three_ticks;
static uint8 s_missing_two_ticks;
static uint8 s_missing_three_ticks;
static uint8 s_no_observation_ticks;

static const beacon_fusion_camera_model_t s_camera_model[BEACON_FUSION_CAMERA_COUNT] =
{
    {
        { 0.6229665f, -0.3596699f, -0.6946584f },
        { -0.5000000f, -0.8660254f, 0.0f },
        { -0.6015918f, 0.3473292f, -0.7193398f }
    },
    {
        { 0.0f, 0.7193398f, -0.6946584f },
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, -0.6946584f, -0.7193398f }
    },
    {
        { -0.6229665f, -0.3596699f, -0.6946584f },
        { -0.5000000f, 0.8660254f, 0.0f },
        { 0.6015918f, 0.3473292f, -0.7193398f }
    }
};

static float beacon_fusion_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float beacon_fusion_clampf(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        return min_value;
    }
    if(value > max_value)
    {
        return max_value;
    }
    return value;
}

static float beacon_fusion_wrap_deg(float angle_deg)
{
    while(angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while(angle_deg <= -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float beacon_fusion_rad_to_deg(float angle_rad)
{
    return angle_rad * (180.0f / BEACON_FUSION_PI);
}

static float beacon_fusion_deg_to_rad(float angle_deg)
{
    return angle_deg * (BEACON_FUSION_PI / 180.0f);
}

static float beacon_fusion_angle_abs_diff(float lhs_deg, float rhs_deg);

static uint8 beacon_fusion_float_valid(float value)
{
    return (value == value) &&
           (value > -1000000.0f) &&
           (value < 1000000.0f);
}

static float beacon_fusion_vec3_len(const beacon_fusion_vec3_t *vec)
{
    return sqrtf((vec->x * vec->x) + (vec->y * vec->y) + (vec->z * vec->z));
}

static beacon_fusion_vec3_t beacon_fusion_vec3_scale(const beacon_fusion_vec3_t *vec, float scale)
{
    beacon_fusion_vec3_t out;

    out.x = vec->x * scale;
    out.y = vec->y * scale;
    out.z = vec->z * scale;
    return out;
}

static beacon_fusion_vec3_t beacon_fusion_vec3_add(const beacon_fusion_vec3_t *a,
                                                   const beacon_fusion_vec3_t *b)
{
    beacon_fusion_vec3_t out;

    out.x = a->x + b->x;
    out.y = a->y + b->y;
    out.z = a->z + b->z;
    return out;
}

static beacon_fusion_vec3_t beacon_fusion_vec3_normalize(const beacon_fusion_vec3_t *vec)
{
    beacon_fusion_vec3_t out = *vec;
    float len = beacon_fusion_vec3_len(vec);

    if(len > 0.000001f)
    {
        out.x /= len;
        out.y /= len;
        out.z /= len;
    }
    return out;
}

static beacon_fusion_vec3_t beacon_fusion_image_to_ray(uint8 camera_id, float target_x, float target_y)
{
    const beacon_fusion_camera_model_t *camera = &s_camera_model[camera_id];
    beacon_fusion_vec3_t ray;
    float image_x = target_x - BEACON_FUSION_IMAGE_CENTER_X_PX;
    float image_y = target_y - BEACON_FUSION_IMAGE_CENTER_Y_PX;
    float radius_px = sqrtf((image_x * image_x) + (image_y * image_y));

    ray = camera->forward;
    if(radius_px > 0.001f)
    {
        float theta = (radius_px / BEACON_FUSION_HALF_DIAGONAL_PX) *
                      BEACON_FUSION_MAX_IMAGE_ANGLE_RAD;
        float tx = image_x / radius_px;
        float ty = image_y / radius_px;
        beacon_fusion_vec3_t right_part = beacon_fusion_vec3_scale(&camera->right, tx);
        beacon_fusion_vec3_t down_part = beacon_fusion_vec3_scale(&camera->down, ty);
        beacon_fusion_vec3_t tangent = beacon_fusion_vec3_add(&right_part, &down_part);
        beacon_fusion_vec3_t forward_part;
        beacon_fusion_vec3_t tangent_part;

        theta = beacon_fusion_clampf(theta, 0.0f, BEACON_FUSION_MAX_IMAGE_ANGLE_RAD);
        tangent = beacon_fusion_vec3_normalize(&tangent);
        forward_part = beacon_fusion_vec3_scale(&camera->forward, cosf(theta));
        tangent_part = beacon_fusion_vec3_scale(&tangent, sinf(theta));
        ray = beacon_fusion_vec3_add(&forward_part, &tangent_part);
    }

    return beacon_fusion_vec3_normalize(&ray);
}

static uint8 beacon_fusion_image_to_origin_delta(uint8 camera_id,
                                                 float target_x,
                                                 float target_y,
                                                 float *x_body,
                                                 float *y_body)
{
    beacon_fusion_vec3_t target_ray = beacon_fusion_image_to_ray(camera_id, target_x, target_y);
    beacon_fusion_vec3_t origin_ray = beacon_fusion_image_to_ray(camera_id,
                                                                 BEACON_FUSION_ORIGIN_PROJ_X_PX,
                                                                 BEACON_FUSION_ORIGIN_PROJ_Y_PX);
    float target_scale;
    float origin_scale;

    if((target_ray.z > -BEACON_FUSION_MIN_GROUND_RAY_Z) ||
       (origin_ray.z > -BEACON_FUSION_MIN_GROUND_RAY_Z))
    {
        return 0U;
    }

    target_scale = BEACON_FUSION_FALLBACK_RANGE_MM / -target_ray.z;
    origin_scale = BEACON_FUSION_FALLBACK_RANGE_MM / -origin_ray.z;
    *x_body = (target_ray.x * target_scale) - (origin_ray.x * origin_scale);
    *y_body = (target_ray.y * target_scale) - (origin_ray.y * origin_scale);
    return 1U;
}

static uint8 beacon_fusion_target_valid(const beacon_fusion_camera_target_t *target)
{
    float target_px_x;
    float target_px_y;

    if(target->valid == 0U)
    {
        return 0U;
    }
    if((beacon_fusion_float_valid(target->x) == 0U) ||
       (beacon_fusion_float_valid(target->y) == 0U) ||
       (beacon_fusion_float_valid(target->radius) == 0U))
    {
        return 0U;
    }
    if((beacon_fusion_absf(target->x) < 0.001f) &&
       (beacon_fusion_absf(target->y) < 0.001f) &&
       (beacon_fusion_absf(target->radius) < 0.001f))
    {
        return 0U;
    }
    if(target->radius < BEACON_FUSION_MIN_RADIUS_PX)
    {
        return 0U;
    }
    target_px_x = target->x;
    target_px_y = target->y;
    if((target_px_x < 0.0f) || (target_px_x >= BEACON_FUSION_IMAGE_WIDTH_PX) ||
       (target_px_y < 0.0f) || (target_px_y >= BEACON_FUSION_IMAGE_HEIGHT_PX))
    {
        target_px_x = BEACON_FUSION_ORIGIN_PROJ_X_PX - target->x;
        target_px_y = BEACON_FUSION_ORIGIN_PROJ_Y_PX + target->y;
        if((target_px_x < 0.0f) || (target_px_x >= BEACON_FUSION_IMAGE_WIDTH_PX) ||
           (target_px_y < 0.0f) || (target_px_y >= BEACON_FUSION_IMAGE_HEIGHT_PX))
        {
            return 0U;
        }
    }
    return 1U;
}

static uint8 beacon_fusion_observation_before(const beacon_fusion_observation_t *lhs,
                                              const beacon_fusion_observation_t *rhs)
{
    if(lhs->y > rhs->y)
    {
        return 1U;
    }
    if(lhs->y < rhs->y)
    {
        return 0U;
    }
    if(lhs->radius > rhs->radius)
    {
        return 1U;
    }
    if(lhs->radius < rhs->radius)
    {
        return 0U;
    }
    if(lhs->x < rhs->x)
    {
        return 1U;
    }
    if(lhs->x > rhs->x)
    {
        return 0U;
    }
    return (lhs->slot_id < rhs->slot_id) ? 1U : 0U;
}

static void beacon_fusion_sort_bucket(beacon_fusion_camera_bucket_t *bucket)
{
    uint8 i;

    for(i = 1U; i < bucket->count; i++)
    {
        uint8 j = i;
        beacon_fusion_observation_t current = bucket->item[i];

        while((j > 0U) &&
              (beacon_fusion_observation_before(&current, &bucket->item[j - 1U]) != 0U))
        {
            bucket->item[j] = bucket->item[j - 1U];
            j--;
        }
        bucket->item[j] = current;
    }
}

static void beacon_fusion_observation_from_target(uint8 camera_id,
                                                  uint8 slot_id,
                                                  const beacon_fusion_camera_target_t *target,
                                                  beacon_fusion_observation_t *observation)
{
    float target_px_x = target->x;
    float target_px_y = target->y;
    beacon_fusion_vec3_t ray;
    float bearing_rad;
    float delta_x;
    float delta_y;

    if((target_px_x < 0.0f) || (target_px_x >= BEACON_FUSION_IMAGE_WIDTH_PX) ||
       (target_px_y < 0.0f) || (target_px_y >= BEACON_FUSION_IMAGE_HEIGHT_PX))
    {
        target_px_x = BEACON_FUSION_ORIGIN_PROJ_X_PX - target->x;
        target_px_y = BEACON_FUSION_ORIGIN_PROJ_Y_PX + target->y;
    }

    ray = beacon_fusion_image_to_ray(camera_id, target_px_x, target_px_y);

    memset(observation, 0, sizeof(*observation));
    observation->camera_id = camera_id;
    observation->slot_id = slot_id;
    observation->x = target_px_x;
    observation->y = target_px_y;
    observation->radius = target->radius;

    if(beacon_fusion_image_to_origin_delta(camera_id, target_px_x, target_px_y, &delta_x, &delta_y) != 0U)
    {
        observation->x_body = delta_x;
        observation->y_body = delta_y;
        observation->range_proxy = sqrtf((delta_x * delta_x) + (delta_y * delta_y));
        if(observation->range_proxy > 0.001f)
        {
            observation->bearing_deg = beacon_fusion_wrap_deg(beacon_fusion_rad_to_deg(atan2f(delta_x,
                                                                                               delta_y)));
        }
    }
    else
    {
        observation->bearing_deg = beacon_fusion_wrap_deg(beacon_fusion_rad_to_deg(atan2f(ray.x, ray.y)));
        bearing_rad = beacon_fusion_deg_to_rad(observation->bearing_deg);
        observation->range_proxy = BEACON_FUSION_FALLBACK_RANGE_MM;
        observation->x_body = sinf(bearing_rad) * BEACON_FUSION_FALLBACK_RANGE_MM;
        observation->y_body = cosf(bearing_rad) * BEACON_FUSION_FALLBACK_RANGE_MM;
    }
    observation->control_x = BEACON_FUSION_ORIGIN_PROJ_X_PX - target_px_x;
    observation->control_y = target_px_y - BEACON_FUSION_ORIGIN_PROJ_Y_PX;
    observation->weight = beacon_fusion_clampf(target->radius / 4.0f, 0.25f, 2.0f);
}

static uint8 beacon_fusion_collect_observations(
    const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT],
    beacon_fusion_camera_bucket_t bucket[BEACON_FUSION_CAMERA_COUNT],
    uint8 *observation_count)
{
    uint8 camera_id;

    memset(bucket, 0, sizeof(beacon_fusion_camera_bucket_t) * BEACON_FUSION_CAMERA_COUNT);
    *observation_count = 0U;

    for(camera_id = 0U; camera_id < BEACON_FUSION_CAMERA_COUNT; camera_id++)
    {
        uint8 slot_id;

        for(slot_id = 0U; slot_id < BEACON_FUSION_CAMERA_TARGETS; slot_id++)
        {
            const beacon_fusion_camera_target_t *target = &camera[camera_id].target[slot_id];

            if(beacon_fusion_target_valid(target) == 0U)
            {
                continue;
            }
            if(bucket[camera_id].count >= BEACON_FUSION_CAMERA_TARGETS)
            {
                continue;
            }

            beacon_fusion_observation_from_target(camera_id,
                                                  slot_id,
                                                  target,
                                                  &bucket[camera_id].item[bucket[camera_id].count]);
            bucket[camera_id].count++;
        }

        *observation_count = (uint8)(*observation_count + bucket[camera_id].count);
        beacon_fusion_sort_bucket(&bucket[camera_id]);
    }

    return 1U;
}

static uint8 beacon_fusion_bucket_max_separated_count(
    const beacon_fusion_camera_bucket_t bucket[BEACON_FUSION_CAMERA_COUNT])
{
    uint8 camera_id;
    uint8 max_count = 0U;

    for(camera_id = 0U; camera_id < BEACON_FUSION_CAMERA_COUNT; camera_id++)
    {
        const beacon_fusion_camera_bucket_t *camera_bucket = &bucket[camera_id];
        uint8 separated[BEACON_FUSION_CAMERA_TARGETS];
        uint8 separated_count = 0U;
        uint8 i;

        memset(separated, 0, sizeof(separated));
        for(i = 0U; i < camera_bucket->count; i++)
        {
            uint8 j;
            uint8 duplicate = 0U;

            for(j = 0U; j < separated_count; j++)
            {
                if(beacon_fusion_angle_abs_diff(camera_bucket->item[i].bearing_deg,
                                                camera_bucket->item[separated[j]].bearing_deg) <=
                   BEACON_FUSION_AUTO_LOCAL_SEP_DEG)
                {
                    duplicate = 1U;
                    break;
                }
            }

            if((duplicate == 0U) && (separated_count < BEACON_FUSION_CAMERA_TARGETS))
            {
                separated[separated_count] = i;
                separated_count++;
            }
        }

        if(separated_count > max_count)
        {
            max_count = separated_count;
        }
    }
    return max_count;
}

static uint8 beacon_fusion_bucket_active_camera_count(
    const beacon_fusion_camera_bucket_t bucket[BEACON_FUSION_CAMERA_COUNT])
{
    uint8 camera_id;
    uint8 active_count = 0U;

    for(camera_id = 0U; camera_id < BEACON_FUSION_CAMERA_COUNT; camera_id++)
    {
        if(bucket[camera_id].count > 0U)
        {
            active_count++;
        }
    }
    return active_count;
}

static uint8 beacon_fusion_bucket_count_over_limit(
    const beacon_fusion_camera_bucket_t bucket[BEACON_FUSION_CAMERA_COUNT],
    uint8 limit)
{
    uint8 camera_id;

    if(limit == 0U)
    {
        return 0U;
    }

    for(camera_id = 0U; camera_id < BEACON_FUSION_CAMERA_COUNT; camera_id++)
    {
        if(bucket[camera_id].count > limit)
        {
            return 1U;
        }
    }
    return 0U;
}

static float beacon_fusion_angle_abs_diff(float lhs_deg, float rhs_deg)
{
    return beacon_fusion_absf(beacon_fusion_wrap_deg(lhs_deg - rhs_deg));
}

static uint8 beacon_fusion_bearing_cluster_count(
    const beacon_fusion_camera_bucket_t bucket[BEACON_FUSION_CAMERA_COUNT])
{
    beacon_fusion_bearing_cluster_t cluster[BEACON_FUSION_MAX_OBSERVATIONS];
    uint8 cluster_count = 0U;
    uint8 camera_id;

    memset(cluster, 0, sizeof(cluster));
    for(camera_id = 0U; camera_id < BEACON_FUSION_CAMERA_COUNT; camera_id++)
    {
        uint8 i;

        for(i = 0U; i < bucket[camera_id].count; i++)
        {
            const beacon_fusion_observation_t *observation = &bucket[camera_id].item[i];
            uint8 best_index = BEACON_FUSION_MAX_OBSERVATIONS;
            float best_diff = 1000000.0f;
            uint8 cluster_id;

            for(cluster_id = 0U; cluster_id < cluster_count; cluster_id++)
            {
                float diff;

                if((cluster[cluster_id].camera_mask & (uint8)(1U << camera_id)) != 0U)
                {
                    continue;
                }

                diff = beacon_fusion_angle_abs_diff(observation->bearing_deg,
                                                    cluster[cluster_id].bearing_deg);
                if(diff < best_diff)
                {
                    best_diff = diff;
                    best_index = cluster_id;
                }
            }

            if((best_index < cluster_count) &&
               (best_diff <= BEACON_FUSION_AUTO_BEARING_CLUSTER_DEG))
            {
                beacon_fusion_bearing_cluster_t *best = &cluster[best_index];
                float sum_weight = best->weight + observation->weight;

                if(sum_weight > 0.0001f)
                {
                    float delta = beacon_fusion_wrap_deg(observation->bearing_deg -
                                                         best->bearing_deg);

                    best->bearing_deg = beacon_fusion_wrap_deg(best->bearing_deg +
                                                               ((delta * observation->weight) /
                                                                sum_weight));
                    best->weight = sum_weight;
                }
                best->camera_mask = (uint8)(best->camera_mask | (uint8)(1U << camera_id));
                if(best->count < 255U)
                {
                    best->count++;
                }
            }
            else if(cluster_count < BEACON_FUSION_MAX_OBSERVATIONS)
            {
                cluster[cluster_count].bearing_deg = observation->bearing_deg;
                cluster[cluster_count].weight = observation->weight;
                cluster[cluster_count].camera_mask = (uint8)(1U << camera_id);
                cluster[cluster_count].count = 1U;
                cluster_count++;
            }
        }
    }

    return cluster_count;
}

static void beacon_fusion_tick_up(uint8 *tick)
{
    if(*tick < 255U)
    {
        (*tick)++;
    }
}

static void beacon_fusion_update_count_evidence(
    const beacon_fusion_camera_bucket_t bucket[BEACON_FUSION_CAMERA_COUNT],
    uint8 observation_count)
{
    uint8 max_separated_count = beacon_fusion_bucket_max_separated_count(bucket);
    uint8 bearing_count = beacon_fusion_bearing_cluster_count(bucket);
    uint8 has_local_three = (max_separated_count >= 3U) ? 1U : 0U;
    uint8 has_global_three =
        ((beacon_fusion_bucket_active_camera_count(bucket) >= BEACON_FUSION_CAMERA_COUNT) &&
         (observation_count >= BEACON_FUSION_AUTO_GLOBAL_THREE_OBS) &&
         (bearing_count >= 3U)) ? 1U : 0U;
    uint8 has_three = ((has_local_three != 0U) || (has_global_three != 0U)) ? 1U : 0U;
    uint8 has_two = (max_separated_count >= 2U) ? 1U : 0U;

    if(has_local_three != 0U)
    {
        beacon_fusion_tick_up(&s_seen_local_three_ticks);
    }
    else
    {
        s_seen_local_three_ticks = 0U;
    }

    if(has_global_three != 0U)
    {
        beacon_fusion_tick_up(&s_seen_global_three_ticks);
    }
    else
    {
        s_seen_global_three_ticks = 0U;
    }

    if(has_three != 0U)
    {
        s_missing_three_ticks = 0U;
    }
    else
    {
        beacon_fusion_tick_up(&s_missing_three_ticks);
    }

    if(has_two != 0U)
    {
        beacon_fusion_tick_up(&s_seen_two_ticks);
        s_missing_two_ticks = 0U;
    }
    else
    {
        s_seen_two_ticks = 0U;
        beacon_fusion_tick_up(&s_missing_two_ticks);
    }
}

static uint8 beacon_fusion_update_auto_count(
    const beacon_fusion_camera_bucket_t bucket[BEACON_FUSION_CAMERA_COUNT],
    uint8 observation_count)
{
    if(s_expected_count != 0U)
    {
        return s_expected_count;
    }

    if(observation_count == 0U)
    {
        s_seen_two_ticks = 0U;
        s_seen_local_three_ticks = 0U;
        s_seen_global_three_ticks = 0U;
        beacon_fusion_tick_up(&s_missing_two_ticks);
        beacon_fusion_tick_up(&s_missing_three_ticks);
        beacon_fusion_tick_up(&s_no_observation_ticks);
        if(s_no_observation_ticks >= BEACON_FUSION_AUTO_NO_OBS_TICKS)
        {
            s_auto_count_state = 0U;
        }
        return s_auto_count_state;
    }

    s_no_observation_ticks = 0U;
    beacon_fusion_update_count_evidence(bucket, observation_count);

    if(s_auto_count_state == 0U)
    {
        s_auto_count_state = 1U;
    }

    if(((s_seen_local_three_ticks >= BEACON_FUSION_AUTO_PROMOTE_TICKS) ||
        (s_seen_global_three_ticks >= BEACON_FUSION_AUTO_GLOBAL_THREE_TICKS)) &&
       (s_auto_count_state < 3U))
    {
        s_auto_count_state = 3U;
    }
    else if((s_seen_two_ticks >= BEACON_FUSION_AUTO_PROMOTE_TICKS) && (s_auto_count_state < 2U))
    {
        s_auto_count_state = 2U;
    }

    if((s_auto_count_state >= 3U) &&
       (s_missing_three_ticks >= BEACON_FUSION_AUTO_THREE_DEMOTE_TICKS))
    {
        s_auto_count_state = (s_seen_two_ticks >= BEACON_FUSION_AUTO_PROMOTE_TICKS) ? 2U : 1U;
        s_missing_three_ticks = 0U;
    }
    if((s_auto_count_state >= 2U) &&
       (s_missing_two_ticks >= BEACON_FUSION_AUTO_TWO_DEMOTE_TICKS))
    {
        s_auto_count_state = 1U;
        s_missing_two_ticks = 0U;
    }

    if(s_auto_count_state > s_auto_max_count)
    {
        s_auto_count_state = s_auto_max_count;
    }

    return s_auto_count_state;
}

static uint8 beacon_fusion_cluster_member_match(
    const beacon_fusion_observation_cluster_t *cluster,
    const beacon_fusion_observation_t *observation)
{
    float diff;
    float threshold;

    if((cluster->camera_mask & (uint8)(1U << observation->camera_id)) != 0U)
    {
        return 0U;
    }

    diff = beacon_fusion_angle_abs_diff(observation->bearing_deg, cluster->bearing_deg);
    threshold = BEACON_FUSION_TRACK_MATCH_DEG + (6.0f - observation->radius) * 3.0f;
    threshold = beacon_fusion_clampf(threshold, 18.0f, 48.0f);
    return (diff <= threshold) ? 1U : 0U;
}

static void beacon_fusion_add_member_to_cluster(beacon_fusion_observation_cluster_t *cluster,
                                                const beacon_fusion_observation_t *observation)
{
    float sum_weight;
    float delta;

    if(cluster->member_count >= BEACON_FUSION_CAMERA_COUNT)
    {
        return;
    }

    cluster->member[cluster->member_count] = *observation;
    cluster->member_count++;
    cluster->camera_mask = (uint8)(cluster->camera_mask | (uint8)(1U << observation->camera_id));

    sum_weight = cluster->weight + observation->weight;
    if(sum_weight > 0.0001f)
    {
        delta = beacon_fusion_wrap_deg(observation->bearing_deg - cluster->bearing_deg);
        cluster->bearing_deg = beacon_fusion_wrap_deg(cluster->bearing_deg +
                                                      ((delta * observation->weight) / sum_weight));
        cluster->weight = sum_weight;
    }

    if(observation->radius > cluster->radius_max)
    {
        cluster->radius_max = observation->radius;
    }
}

static void beacon_fusion_collect_clusters(
    const beacon_fusion_camera_bucket_t bucket[BEACON_FUSION_CAMERA_COUNT],
    beacon_fusion_observation_cluster_t cluster[BEACON_FUSION_MAX_OBSERVATIONS],
    uint8 *cluster_count)
{
    uint8 camera_id;

    memset(cluster, 0, sizeof(beacon_fusion_observation_cluster_t) * BEACON_FUSION_MAX_OBSERVATIONS);
    *cluster_count = 0U;

    for(camera_id = 0U; camera_id < BEACON_FUSION_CAMERA_COUNT; camera_id++)
    {
        uint8 i;

        for(i = 0U; i < bucket[camera_id].count; i++)
        {
            const beacon_fusion_observation_t *observation = &bucket[camera_id].item[i];
            uint8 cluster_id;
            uint8 best_index = BEACON_FUSION_MAX_OBSERVATIONS;
            float best_diff = 1000000.0f;

            for(cluster_id = 0U; cluster_id < *cluster_count; cluster_id++)
            {
                float diff;

                if(beacon_fusion_cluster_member_match(&cluster[cluster_id], observation) == 0U)
                {
                    continue;
                }

                diff = beacon_fusion_angle_abs_diff(observation->bearing_deg,
                                                    cluster[cluster_id].bearing_deg);
                if(diff < best_diff)
                {
                    best_diff = diff;
                    best_index = cluster_id;
                }
            }

            if(best_index < *cluster_count)
            {
                beacon_fusion_add_member_to_cluster(&cluster[best_index], observation);
            }
            else if(*cluster_count < BEACON_FUSION_MAX_OBSERVATIONS)
            {
                beacon_fusion_observation_cluster_t *new_cluster = &cluster[*cluster_count];

                memset(new_cluster, 0, sizeof(*new_cluster));
                new_cluster->bearing_deg = observation->bearing_deg;
                beacon_fusion_add_member_to_cluster(new_cluster, observation);
                (*cluster_count)++;
            }
        }
    }
}

static uint8 beacon_fusion_cluster_before(const beacon_fusion_observation_cluster_t *lhs,
                                          const beacon_fusion_observation_cluster_t *rhs)
{
    if(lhs->member_count > rhs->member_count)
    {
        return 1U;
    }
    if(lhs->member_count < rhs->member_count)
    {
        return 0U;
    }
    if(lhs->radius_max > rhs->radius_max)
    {
        return 1U;
    }
    if(lhs->radius_max < rhs->radius_max)
    {
        return 0U;
    }
    return (lhs->weight > rhs->weight) ? 1U : 0U;
}

static void beacon_fusion_sort_clusters(beacon_fusion_observation_cluster_t cluster[BEACON_FUSION_MAX_OBSERVATIONS],
                                        uint8 cluster_count)
{
    uint8 i;

    for(i = 1U; i < cluster_count; i++)
    {
        uint8 j = i;
        beacon_fusion_observation_cluster_t current = cluster[i];

        while((j > 0U) &&
              (beacon_fusion_cluster_before(&current, &cluster[j - 1U]) != 0U))
        {
            cluster[j] = cluster[j - 1U];
            j--;
        }
        cluster[j] = current;
    }
}

static uint8 beacon_fusion_find_matching_previous(const beacon_fusion_result_t *previous,
                                                  const beacon_fusion_observation_cluster_t *cluster,
                                                  uint8 used_previous[BEACON_FUSION_MAX_BEACONS])
{
    uint8 i;
    uint8 best_index = BEACON_FUSION_MAX_BEACONS;
    float best_diff = 1000000.0f;

    for(i = 0U; i < previous->beacon_count; i++)
    {
        float diff;

        if((previous->beacon[i].valid == 0U) || (used_previous[i] != 0U))
        {
            continue;
        }

        diff = beacon_fusion_angle_abs_diff(cluster->bearing_deg, previous->beacon[i].bearing_deg);
        if((diff <= BEACON_FUSION_TRACK_MATCH_DEG) && (diff < best_diff))
        {
            best_diff = diff;
            best_index = i;
        }
    }

    return best_index;
}

static void beacon_fusion_assign_clusters(
    const beacon_fusion_result_t *previous,
    beacon_fusion_observation_cluster_t cluster[BEACON_FUSION_MAX_OBSERVATIONS],
    uint8 cluster_count,
    beacon_fusion_observation_cluster_t assigned[BEACON_FUSION_MAX_BEACONS],
    uint8 assigned_valid[BEACON_FUSION_MAX_BEACONS],
    uint8 expected_count)
{
    uint8 used_cluster[BEACON_FUSION_MAX_OBSERVATIONS];
    uint8 used_previous[BEACON_FUSION_MAX_BEACONS];
    uint8 i;

    memset(assigned, 0, sizeof(beacon_fusion_observation_cluster_t) * BEACON_FUSION_MAX_BEACONS);
    memset(assigned_valid, 0, BEACON_FUSION_MAX_BEACONS);
    memset(used_cluster, 0, sizeof(used_cluster));
    memset(used_previous, 0, sizeof(used_previous));

    for(i = 0U; i < cluster_count; i++)
    {
        uint8 previous_index = beacon_fusion_find_matching_previous(previous, &cluster[i], used_previous);

        if((previous_index < expected_count) && (previous_index < BEACON_FUSION_MAX_BEACONS))
        {
            assigned[previous_index] = cluster[i];
            assigned_valid[previous_index] = 1U;
            used_cluster[i] = 1U;
            used_previous[previous_index] = 1U;
        }
    }

    for(i = 0U; i < expected_count; i++)
    {
        uint8 cluster_id;

        if(assigned_valid[i] != 0U)
        {
            continue;
        }

        for(cluster_id = 0U; cluster_id < cluster_count; cluster_id++)
        {
            if(used_cluster[cluster_id] == 0U)
            {
                assigned[i] = cluster[cluster_id];
                assigned_valid[i] = 1U;
                used_cluster[cluster_id] = 1U;
                break;
            }
        }
    }
}

static void beacon_fusion_build_observed_beacon(
    const beacon_fusion_observation_t member[BEACON_FUSION_CAMERA_COUNT],
    uint8 member_count,
    beacon_fusion_beacon_t *beacon)
{
    uint8 i;
    float sum_x_body = 0.0f;
    float sum_y_body = 0.0f;
    float sum_control_x = 0.0f;
    float sum_control_y = 0.0f;
    float sum_weight = 0.0f;
    float radius_max = 0.0f;

    memset(beacon, 0, sizeof(*beacon));

    for(i = 0U; i < member_count; i++)
    {
        float weight = member[i].weight;

        sum_x_body += member[i].x_body * weight;
        sum_y_body += member[i].y_body * weight;
        sum_control_x += member[i].control_x * weight;
        sum_control_y += member[i].control_y * weight;
        sum_weight += weight;
        beacon->source_camera_mask = (uint8)(beacon->source_camera_mask | (uint8)(1U << member[i].camera_id));
        if(member[i].radius > radius_max)
        {
            radius_max = member[i].radius;
        }
    }

    if(sum_weight <= 0.0f)
    {
        beacon->valid = 1U;
        beacon->range_proxy = BEACON_FUSION_MISSING_RANGE_PROXY;
        beacon->confidence = 0.0f;
        return;
    }

    beacon->valid = 1U;
    beacon->observation_count = member_count;
    beacon->x_body = sum_x_body / sum_weight;
    beacon->y_body = sum_y_body / sum_weight;
    beacon->control_x = sum_control_x / sum_weight;
    beacon->control_y = sum_control_y / sum_weight;
    beacon->range_proxy = BEACON_FUSION_RADIUS_RANGE_SCALE /
                           beacon_fusion_clampf(radius_max, 0.5f, 20.0f);
    beacon->range_proxy = beacon_fusion_clampf(beacon->range_proxy,
                                               BEACON_FUSION_MIN_RANGE_PROXY,
                                               BEACON_FUSION_MAX_RANGE_PROXY);
    beacon->bearing_deg = beacon_fusion_wrap_deg(beacon_fusion_rad_to_deg(atan2f(beacon->x_body,
                                                                                  beacon->y_body)));
    beacon->confidence = 0.2f + ((float)member_count / (float)BEACON_FUSION_CAMERA_COUNT) * 0.55f;
    beacon->confidence += beacon_fusion_clampf(radius_max / 8.0f, 0.0f, 0.25f);
    beacon->confidence = beacon_fusion_clampf(beacon->confidence, 0.0f, 1.0f);
}

static void beacon_fusion_fill_missing_beacon(const beacon_fusion_result_t *previous,
                                              uint8 index,
                                              beacon_fusion_beacon_t *beacon)
{
    memset(beacon, 0, sizeof(*beacon));
    if((index < previous->beacon_count) && (previous->beacon[index].valid != 0U))
    {
        *beacon = previous->beacon[index];
        beacon->observation_count = 0U;
        beacon->source_camera_mask = 0U;
        beacon->confidence *= 0.90f;
        if(beacon->stable_ticks < 255U)
        {
            beacon->stable_ticks++;
        }
        return;
    }

    beacon->valid = 1U;
    beacon->range_proxy = BEACON_FUSION_MISSING_RANGE_PROXY;
    beacon->confidence = 0.0f;
}

static void beacon_fusion_apply_filter(beacon_fusion_result_t *current,
                                       const beacon_fusion_result_t *previous)
{
    uint8 i;

    for(i = 0U; i < current->beacon_count; i++)
    {
        beacon_fusion_beacon_t *beacon = &current->beacon[i];
        const beacon_fusion_beacon_t *old_beacon = &previous->beacon[i];

        if((beacon->valid == 0U) ||
           (beacon->observation_count == 0U) ||
           (i >= previous->beacon_count) ||
           (old_beacon->valid == 0U))
        {
            if(beacon->stable_ticks == 0U)
            {
                beacon->stable_ticks = 1U;
            }
            continue;
        }

        beacon->bearing_deg = beacon_fusion_wrap_deg((old_beacon->bearing_deg * (1.0f - BEACON_FUSION_FILTER_ALPHA)) +
                                                     (beacon->bearing_deg * BEACON_FUSION_FILTER_ALPHA));
        beacon->x_body = (old_beacon->x_body * (1.0f - BEACON_FUSION_FILTER_ALPHA)) +
                         (beacon->x_body * BEACON_FUSION_FILTER_ALPHA);
        beacon->y_body = (old_beacon->y_body * (1.0f - BEACON_FUSION_FILTER_ALPHA)) +
                         (beacon->y_body * BEACON_FUSION_FILTER_ALPHA);
        beacon->control_x = (old_beacon->control_x * (1.0f - BEACON_FUSION_FILTER_ALPHA)) +
                            (beacon->control_x * BEACON_FUSION_FILTER_ALPHA);
        beacon->control_y = (old_beacon->control_y * (1.0f - BEACON_FUSION_FILTER_ALPHA)) +
                            (beacon->control_y * BEACON_FUSION_FILTER_ALPHA);
        beacon->range_proxy = (old_beacon->range_proxy * (1.0f - BEACON_FUSION_FILTER_ALPHA)) +
                              (beacon->range_proxy * BEACON_FUSION_FILTER_ALPHA);
        beacon->bearing_deg = beacon_fusion_wrap_deg(beacon_fusion_rad_to_deg(atan2f(beacon->x_body,
                                                                                      beacon->y_body)));
        beacon->stable_ticks = old_beacon->stable_ticks;
        if(beacon->stable_ticks < 255U)
        {
            beacon->stable_ticks++;
        }
    }
}

static uint8 beacon_fusion_choose_best(const beacon_fusion_result_t *result)
{
    uint8 i;
    uint8 best = BEACON_FUSION_MAX_BEACONS;
    float best_score = -1000000.0f;

    for(i = 0U; i < result->beacon_count; i++)
    {
        const beacon_fusion_beacon_t *beacon = &result->beacon[i];
        float score;

        if(beacon->valid == 0U)
        {
            continue;
        }
        score = beacon->confidence - (beacon->range_proxy * 0.0002f);
        if(score > best_score)
        {
            best_score = score;
            best = i;
        }
    }

    return best;
}

void beacon_fusion_init(void)
{
    memset(&g_beacon_fusion_result, 0, sizeof(g_beacon_fusion_result));
    g_beacon_fusion_result.best_index = BEACON_FUSION_MAX_BEACONS;
    s_auto_count_state = 0U;
    s_auto_max_count = BEACON_FUSION_AUTO_MAX_COUNT;
    s_expected_count = 0U;
    s_seen_two_ticks = 0U;
    s_seen_local_three_ticks = 0U;
    s_seen_global_three_ticks = 0U;
    s_missing_two_ticks = 0U;
    s_missing_three_ticks = 0U;
    s_no_observation_ticks = 0U;
}

void beacon_fusion_set_auto_max_count(uint8 max_count)
{
    if(max_count == 0U)
    {
        max_count = BEACON_FUSION_AUTO_MAX_COUNT;
    }
    if(max_count > BEACON_FUSION_AUTO_MAX_COUNT)
    {
        max_count = BEACON_FUSION_AUTO_MAX_COUNT;
    }
    s_auto_max_count = max_count;
    if(s_auto_count_state > s_auto_max_count)
    {
        s_auto_count_state = s_auto_max_count;
    }
}

void beacon_fusion_set_expected_count(uint8 expected_count)
{
    if(expected_count > BEACON_FUSION_AUTO_MAX_COUNT)
    {
        expected_count = BEACON_FUSION_AUTO_MAX_COUNT;
    }
    s_expected_count = expected_count;
    if(s_expected_count != 0U)
    {
        s_auto_count_state = s_expected_count;
        beacon_fusion_set_auto_max_count(s_expected_count);
    }
}

void beacon_fusion_update_100HZ(const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT])
{
    beacon_fusion_camera_bucket_t bucket[BEACON_FUSION_CAMERA_COUNT];
    beacon_fusion_observation_cluster_t cluster[BEACON_FUSION_MAX_OBSERVATIONS];
    beacon_fusion_observation_cluster_t assigned[BEACON_FUSION_MAX_BEACONS];
    beacon_fusion_result_t previous = g_beacon_fusion_result;
    beacon_fusion_result_t current;
    uint8 assigned_valid[BEACON_FUSION_MAX_BEACONS];
    uint8 estimated_count;
    uint8 cluster_count;
    uint8 observation_count;
    uint8 beacon_id;

    memset(&current, 0, sizeof(current));
    current.best_index = BEACON_FUSION_MAX_BEACONS;
    current.update_count = previous.update_count + 1U;

    if(camera == 0)
    {
        g_beacon_fusion_result = current;
        return;
    }

    if(beacon_fusion_collect_observations(camera, bucket, &observation_count) == 0U)
    {
        return;
    }

    beacon_fusion_collect_clusters(bucket, cluster, &cluster_count);
    beacon_fusion_sort_clusters(cluster, cluster_count);
    estimated_count = beacon_fusion_update_auto_count(bucket, observation_count);
    if((s_expected_count != 0U) &&
       (beacon_fusion_bucket_count_over_limit(bucket, estimated_count) != 0U))
    {
        return;
    }
    beacon_fusion_assign_clusters(&previous,
                                  cluster,
                                  cluster_count,
                                  assigned,
                                  assigned_valid,
                                  estimated_count);

    current.observation_count = observation_count;
    current.beacon_count = estimated_count;
    for(beacon_id = 0U; beacon_id < estimated_count; beacon_id++)
    {
        if(assigned_valid[beacon_id] == 0U)
        {
            beacon_fusion_fill_missing_beacon(&previous, beacon_id, &current.beacon[beacon_id]);
        }
        else
        {
            beacon_fusion_build_observed_beacon(assigned[beacon_id].member,
                                                assigned[beacon_id].member_count,
                                                &current.beacon[beacon_id]);
        }
    }

    beacon_fusion_apply_filter(&current, &previous);
    if(current.beacon_count > 0U)
    {
        current.best_index = beacon_fusion_choose_best(&current);
    }

    g_beacon_fusion_result = current;
}

const beacon_fusion_result_t *beacon_fusion_get_result(void)
{
    return &g_beacon_fusion_result;
}

void beacon_fusion_analyze(
    const beacon_result_t camera_results[BEACON_CAMERA_COUNT],
    beacon_fusion_result_t *result)
{
    beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT];
    int camera_index;

    if(result == 0)
    {
        return;
    }

    memset(result, 0, sizeof(*result));
    if(camera_results == 0)
    {
        return;
    }

    memset(camera, 0, sizeof(camera));
    for(camera_index = 0; camera_index < BEACON_FUSION_CAMERA_COUNT; camera_index++)
    {
        int target_index;
        const beacon_result_t *camera_result = &camera_results[camera_index];

        for(target_index = 0;
            target_index < camera_result->count &&
            target_index < BEACON_FUSION_CAMERA_TARGETS;
            target_index++)
        {
            const beacon_circle_t *circle = &camera_result->circles[target_index];
            beacon_fusion_camera_target_t *target = &camera[camera_index].target[target_index];

            if(circle->valid == 0U)
            {
                continue;
            }

            target->valid = 1U;
            target->x = BEACON_IMAGE_TARGET_PIXEL_X - circle->x;
            target->y = BEACON_IMAGE_TARGET_PIXEL_Y + circle->y;
            target->radius = circle->radius;
        }
    }

    beacon_fusion_update_100HZ(camera);
    *result = *beacon_fusion_get_result();
}
