#ifndef DOWN_DRAFT_IMAGE_H_
#define DOWN_DRAFT_IMAGE_H_

#include "zf_common_headfile.h"
#include "Image/image_data.h"

#define IMAGE_ALGORITHM_BUILD_ID (0x20261002UL)

#define IPC_REMOTE_PARAM_OP_SET                (1U)
#define IPC_REMOTE_PARAM_OP_GET                (2U)
#define IPC_REMOTE_PARAM_TYPE_FLOAT            (0U)
#define IPC_REMOTE_PARAM_TYPE_INT32            (1U)
#define IPC_REMOTE_PARAM_STATUS_OK             (0U)
#define IPC_REMOTE_PARAM_STATUS_NOT_FOUND      (1U)
#define IPC_REMOTE_PARAM_STATUS_OUT_OF_RANGE   (2U)
#define IPC_REMOTE_PARAM_STATUS_ERROR          (3U)
#define IPC_REMOTE_PARAM_STATUS_MISMATCH       (6U)
#define IPC_REMOTE_PARAM_STATUS_ROLLBACK_FAIL  (8U)

#define IPC_REMOTE_PARAM_ID_C1_BEACON_THR      (0x0300U)
#define IPC_REMOTE_PARAM_ID_C1_EXP_TIME        (0x0301U)
#define IPC_REMOTE_PARAM_ID_C1_SCREEN_MODE     (0x0302U)
#define IPC_REMOTE_PARAM_ID_C1_BEACON_MIN      (0x0303U)
#define IPC_REMOTE_PARAM_ID_C1_EDGE_MIN        (0x0304U)
#define IPC_REMOTE_PARAM_ID_C1_EDGE_THR        (0x0305U)
#define IPC_REMOTE_PARAM_ID_C1_LAMP_THR        (0x0306U)
#define IPC_REMOTE_PARAM_ID_C1_LAMP_MIN        (0x0307U)
#define IPC_REMOTE_PARAM_ID_C1_LAMP_MAX        (0x0308U)
#define IPC_REMOTE_PARAM_ID_C1_LAMP_ELONG      (0x0309U)
#define IPC_REMOTE_PARAM_ID_C1_LAMP_LEN        (0x030AU)
#define IPC_REMOTE_PARAM_ID_C1_NEAR_PAD        (0x030BU)
#define IPC_REMOTE_PARAM_ID_C1_NEAR_MIN        (0x030CU)
#define IPC_REMOTE_PARAM_ID_C1_NEAR_ISO_MIN    (0x030DU)
#define IPC_REMOTE_PARAM_ID_C1_NEAR_BG         (0x030EU)
#define IPC_REMOTE_PARAM_ID_C1_MATCH_DIST      (0x030FU)
#define IPC_REMOTE_PARAM_ID_C1_GATE_DIST       (0x0310U)
#define IPC_REMOTE_PARAM_ID_C1_NEW_DIST        (0x0311U)
#define IPC_REMOTE_PARAM_ID_C1_CONFIRM         (0x0312U)
#define IPC_REMOTE_PARAM_ID_C1_MISSES          (0x0313U)
#define IPC_REMOTE_PARAM_ID_C1_POS_ALPHA       (0x0314U)
#define IPC_REMOTE_PARAM_ID_C1_VEL_ALPHA       (0x0315U)

typedef struct
{
    float x;
    float y;
    float radius;
    float area;
    uint8 valid;
} beacon_circle_t;

typedef struct
{
    float cx;
    float cy;
    float width;
    float length;
    float angle;
    uint8 valid;
} beacon_rect_t;

extern struct image_data image_data[IMAGE_CAMERA_COUNT];

uint16 image_param_count(void);
uint8 image_param_info(uint16 index, uint16 *id, uint8 *type, float *minimum, float *maximum);
uint8 image_param_get(uint8 type, uint16 id, uint32 *actual_bits);
uint8 image_param_set(uint8 type, uint16 id, uint32 value_bits, uint32 *actual_bits);

#endif
