#include "image_down.h"

static const uint16 s_ids[] = {
    IPC_REMOTE_PARAM_ID_C1_BEACON_THR, IPC_REMOTE_PARAM_ID_C1_EXP_TIME,
    IPC_REMOTE_PARAM_ID_C1_SCREEN_MODE, IPC_REMOTE_PARAM_ID_C1_BEACON_MIN,
    IPC_REMOTE_PARAM_ID_C1_EDGE_MIN, IPC_REMOTE_PARAM_ID_C1_EDGE_THR,
    IPC_REMOTE_PARAM_ID_C1_LAMP_THR, IPC_REMOTE_PARAM_ID_C1_LAMP_MIN,
    IPC_REMOTE_PARAM_ID_C1_LAMP_MAX, IPC_REMOTE_PARAM_ID_C1_LAMP_ELONG,
    IPC_REMOTE_PARAM_ID_C1_LAMP_LEN, IPC_REMOTE_PARAM_ID_C1_NEAR_PAD,
    IPC_REMOTE_PARAM_ID_C1_NEAR_MIN, IPC_REMOTE_PARAM_ID_C1_NEAR_ISO_MIN,
    IPC_REMOTE_PARAM_ID_C1_NEAR_BG, IPC_REMOTE_PARAM_ID_C1_MATCH_DIST,
    IPC_REMOTE_PARAM_ID_C1_GATE_DIST, IPC_REMOTE_PARAM_ID_C1_NEW_DIST,
    IPC_REMOTE_PARAM_ID_C1_CONFIRM, IPC_REMOTE_PARAM_ID_C1_MISSES,
    IPC_REMOTE_PARAM_ID_C1_POS_ALPHA, IPC_REMOTE_PARAM_ID_C1_VEL_ALPHA
};

static const uint8 s_types[] = {
    IPC_REMOTE_PARAM_TYPE_INT32, IPC_REMOTE_PARAM_TYPE_INT32,
    IPC_REMOTE_PARAM_TYPE_INT32, IPC_REMOTE_PARAM_TYPE_INT32,
    IPC_REMOTE_PARAM_TYPE_INT32, IPC_REMOTE_PARAM_TYPE_INT32,
    IPC_REMOTE_PARAM_TYPE_INT32, IPC_REMOTE_PARAM_TYPE_INT32,
    IPC_REMOTE_PARAM_TYPE_INT32, IPC_REMOTE_PARAM_TYPE_FLOAT,
    IPC_REMOTE_PARAM_TYPE_FLOAT, IPC_REMOTE_PARAM_TYPE_INT32,
    IPC_REMOTE_PARAM_TYPE_INT32, IPC_REMOTE_PARAM_TYPE_INT32,
    IPC_REMOTE_PARAM_TYPE_INT32, IPC_REMOTE_PARAM_TYPE_FLOAT,
    IPC_REMOTE_PARAM_TYPE_FLOAT, IPC_REMOTE_PARAM_TYPE_FLOAT,
    IPC_REMOTE_PARAM_TYPE_INT32, IPC_REMOTE_PARAM_TYPE_INT32,
    IPC_REMOTE_PARAM_TYPE_FLOAT, IPC_REMOTE_PARAM_TYPE_FLOAT
};

static const float s_minimums[] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f
};

static const float s_maximums[] = {
    255.0f, 636.0f, 2.0f, 22560.0f, 22560.0f, 255.0f,
    255.0f, 22560.0f, 22560.0f, 224.0f, 224.0f, 224.0f,
    22560.0f, 22560.0f, 255.0f, 224.0f, 224.0f, 224.0f,
    255.0f, 255.0f, 1.0f, 1.0f
};

uint16 image_param_count(void)
{
    return (uint16)(sizeof(s_ids) / sizeof(s_ids[0]));
}

uint8 image_param_info(uint16 index, uint16 *id, uint8 *type, float *minimum, float *maximum)
{
    if (index >= image_param_count() || id == 0 || type == 0 || minimum == 0 || maximum == 0)
    {
        return IPC_REMOTE_PARAM_STATUS_ERROR;
    }
    *id = s_ids[index];
    *type = s_types[index];
    *minimum = s_minimums[index];
    *maximum = s_maximums[index];
    return IPC_REMOTE_PARAM_STATUS_OK;
}

uint8 image_param_get(uint8 type, uint16 id, uint32 *actual_bits)
{
    return image_down_remote_param_execute(IPC_REMOTE_PARAM_OP_GET, type, id, 0U, actual_bits);
}

uint8 image_param_set(uint8 type, uint16 id, uint32 value_bits, uint32 *actual_bits)
{
    return image_down_remote_param_execute(IPC_REMOTE_PARAM_OP_SET, type, id, value_bits, actual_bits);
}
