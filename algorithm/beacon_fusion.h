#ifndef BEACON_FUSION_H
#define BEACON_FUSION_H

#include "beacon_image.h"
#include "zf_common_typedef.h"
#include "Protocols/CameraSpi/camera_spi_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BEACON_FUSION_CAMERA_COUNT       (3U)
#define BEACON_CAMERA_COUNT              (BEACON_FUSION_CAMERA_COUNT)
#define BEACON_FUSION_CAMERA_TARGETS     (CAMERA_SPI_IMAGE_TARGET_COUNT)
#define BEACON_FUSION_MAX_OBSERVATIONS   (BEACON_FUSION_CAMERA_COUNT * BEACON_FUSION_CAMERA_TARGETS)
#define BEACON_FUSION_MAX_BEACONS        (5U)

typedef struct
{
    uint8 valid;
    float x;
    float y;
    float radius;
} beacon_fusion_camera_target_t;

typedef struct
{
    beacon_fusion_camera_target_t target[BEACON_FUSION_CAMERA_TARGETS];
} beacon_fusion_camera_frame_t;

typedef struct
{
    uint8 valid;
    uint8 source_camera_mask;
    uint8 observation_count;
    uint8 stable_ticks;
    float bearing_deg;
    float range_proxy;
    float x_body;
    float y_body;
    float control_x;
    float control_y;
    float confidence;
} beacon_fusion_beacon_t;

typedef struct
{
    uint8 beacon_count;
    uint8 best_index;
    uint8 observation_count;
    uint32 update_count;
    beacon_fusion_beacon_t beacon[BEACON_FUSION_MAX_BEACONS];
} beacon_fusion_result_t;

extern beacon_fusion_result_t g_beacon_fusion_result;

void beacon_fusion_init(void);
void beacon_fusion_set_auto_max_count(uint8 max_count);
void beacon_fusion_set_expected_count(uint8 expected_count);
void beacon_fusion_update_100HZ(const beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT]);
const beacon_fusion_result_t *beacon_fusion_get_result(void);
void beacon_fusion_analyze(
    const beacon_result_t camera_results[BEACON_CAMERA_COUNT],
    beacon_fusion_result_t *result
);

#ifdef __cplusplus
}
#endif

#endif /* BEACON_FUSION_H */
