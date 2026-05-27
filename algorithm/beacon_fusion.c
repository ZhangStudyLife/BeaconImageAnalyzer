#include "beacon_fusion.h"

#include <math.h>
#include <string.h>

void beacon_fusion_init(void)
{
}

void beacon_fusion_analyze(
    const beacon_result_t camera_results[BEACON_CAMERA_COUNT],
    beacon_fusion_result_t *result)
{
    int camera;
    int i;
    int output_count = 0;
    const float camera_base_angle[BEACON_CAMERA_COUNT] = { 0.0f, 120.0f, 240.0f };

    if (result == 0)
    {
        return;
    }

    memset(result, 0, sizeof(*result));
    if (camera_results == 0)
    {
        return;
    }

    for (camera = 0; camera < BEACON_CAMERA_COUNT; ++camera)
    {
        const beacon_result_t *camera_result = &camera_results[camera];
        for (i = 0;
             i < camera_result->count &&
             i < BEACON_MAX_CIRCLE_COUNT &&
             output_count < BEACON_MAX_FUSION_TARGET_COUNT;
             ++i)
        {
            const beacon_circle_t *circle = &camera_result->circles[i];
            float angle;
            float distance;
            beacon_fusion_target_t *target;

            if (circle->valid == 0)
            {
                continue;
            }

            angle = camera_base_angle[camera] +
                    atanf(circle->x / ((float)BEACON_IMAGE_W * 0.5f)) * 57.2957795f;
            while (angle < 0.0f)
            {
                angle += 360.0f;
            }
            while (angle >= 360.0f)
            {
                angle -= 360.0f;
            }

            distance = (float)BEACON_IMAGE_H / (2.0f * fmaxf(circle->radius, 0.5f));

            target = &result->targets[output_count];
            target->angle_deg = angle;
            target->distance = distance;
            target->x = distance * cosf(angle / 57.2957795f);
            target->y = distance * sinf(angle / 57.2957795f);
            target->camera_index = (unsigned char)camera;
            target->source_index = (unsigned char)i;
            target->valid = 1;
            ++output_count;
        }
    }

    result->count = (unsigned char)output_count;
}
