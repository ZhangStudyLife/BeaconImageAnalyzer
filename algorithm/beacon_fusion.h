#ifndef BEACON_FUSION_H_
#define BEACON_FUSION_H_

#include "beacon_image.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BEACON_CAMERA_COUNT 3
#define BEACON_MAX_FUSION_TARGET_COUNT (BEACON_CAMERA_COUNT * BEACON_MAX_CIRCLE_COUNT)

typedef struct
{
    float angle_deg;
    float distance;
    float x;
    float y;
    unsigned char camera_index;
    unsigned char source_index;
    unsigned char valid;
} beacon_fusion_target_t;

typedef struct
{
    beacon_fusion_target_t targets[BEACON_MAX_FUSION_TARGET_COUNT];
    unsigned char count;
} beacon_fusion_result_t;

void beacon_fusion_init(void);

void beacon_fusion_analyze(
    const beacon_result_t camera_results[BEACON_CAMERA_COUNT],
    beacon_fusion_result_t *result
);

#ifdef __cplusplus
}
#endif

#endif
