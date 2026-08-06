#include "image.h"

#include <math.h>
#include <string.h>

#define IMAGE_PARAM_GRAY_MAX      (255.0f)
#define IMAGE_PARAM_AREA_MAX      (22560.0f)
#define IMAGE_PARAM_DISTANCE_MAX  (224.0f)
#define IMAGE_PARAM_FRAME_MAX     (255.0f)

typedef uint8 (*image_param_set_handler_t)(uint32 requested_bits,
                                           uint32 *actual_bits);
typedef uint8 (*image_param_get_handler_t)(uint32 *actual_bits);

typedef struct
{
    uint16 id;
    uint8 type;
    void *value_ptr;
    float minimum;
    float maximum;
    image_param_set_handler_t set_handler;
    image_param_get_handler_t get_handler;
} image_param_descriptor_t;

#define PARAM_I(id_, field_, min_, max_) \
    {(id_), IMAGE_PARAM_TYPE_INT32, &(field_), (min_), (max_), NULL, NULL}
#define PARAM_F(id_, field_, min_, max_) \
    {(id_), IMAGE_PARAM_TYPE_FLOAT32, &(field_), (min_), (max_), NULL, NULL}
#define PARAM_CUSTOM_I(id_, min_, max_, set_, get_) \
    {(id_), IMAGE_PARAM_TYPE_INT32, NULL, (min_), (max_), (set_), (get_)}

static uint8 exposure_param_set(uint32 requested_bits, uint32 *actual_bits);
static uint8 exposure_param_get(uint32 *actual_bits);

int32 beacon_edge_threshold = 80;
int32 beacon_track_threshold = 105;
int32 car_lamp_binary_threshold = 200;
int32 car_lamp_upper_threshold = 150;
float car_lamp_upper_y = 64.0f;
float car_lamp_bridge_max_gap = 4.0f;
int32 beacon_min_component_area = 6;
int32 beacon_edge_min_area = 2;
int32 beacon_top_edge_max_area = 50;
int32 beacon_edge_max_area = 60;
int32 car_lamp_min_area = 24;
int32 car_lamp_max_area = 230;
float car_lamp_min_elongation = 1.6f;
float car_lamp_min_length = 12.0f;
int32 beacon_isolated_gray_min = 120;
int32 beacon_isolated_bg_max = 2;
float beacon_local_ring_inner = 3.0f;
float beacon_local_ring_outer = 8.0f;
float lamp_near_beacon_pad = 8.0f;
int32 lamp_near_beacon_min_area = 21;
int32 lamp_near_beacon_gray_min = 150;
int32 lamp_near_beacon_background_max = 20;
float b0_match_distance = 18.0f;
float kalman_gate_distance = 24.0f;
float kalman_new_target_distance = 36.0f;
int32 b0_init_confirm_frames = 2;
int32 beacon_max_misses = 3;
float filter_pos_alpha = 0.65f;
float filter_vel_alpha = 0.30f;
int32 stream_mode = IMAGE_STREAM_MODE_RAW;
float car_lamp_min_width = 3.5f;
float car_lamp_narrow_min_width = 2.7f;
float car_lamp_narrow_min_elongation = 3.5f;
int32 car_lamp_upper_min_area = 120;
float car_lamp_upper_min_length = 22.0f;
float car_lamp_upper_min_width = 5.5f;
float car_lamp_compact_min_y = 20.0f;
int32 car_lamp_compact_min_area = 36;
float car_lamp_compact_min_length = 14.0f;
float car_lamp_compact_min_width = 3.0f;
float car_lamp_compact_min_elongation = 3.0f;
float vertical_glare_min_elongation = 1.8f;
int32 vertical_glare_max_gray = 200;
float linear_min_elongation = 6.0f;
int32 weak_center_threshold = 70;
int32 weak_center_min_area = 3;
int32 weak_center_max_area = 12;
int32 weak_center_min_gray = 90;
int32 weak_center_max_bg = 10;
int32 shape_min_area = 6;
float shape_max_ratio = 2.0f;
int32 shape_min_fill_percent = 60;
int32 shape_small_min_fill_percent = 50;
float top_vertical_min_elongation = 3.0f;
int32 saturated_top_min_gray = 240;


static const image_param_descriptor_t s_params[] =
{
    PARAM_I(IMAGE_PARAM_ID_BEACON_EDGE_THRESHOLD, beacon_edge_threshold, 0, IMAGE_PARAM_GRAY_MAX),
    PARAM_I(IMAGE_PARAM_ID_BEACON_TRACK_THRESHOLD, beacon_track_threshold, 0, IMAGE_PARAM_GRAY_MAX),
    PARAM_I(IMAGE_PARAM_ID_CAR_LAMP_BINARY_THRESHOLD, car_lamp_binary_threshold, 0, IMAGE_PARAM_GRAY_MAX),
    PARAM_I(IMAGE_PARAM_ID_CAR_LAMP_UPPER_THRESHOLD, car_lamp_upper_threshold, 0, IMAGE_PARAM_GRAY_MAX),
    PARAM_F(IMAGE_PARAM_ID_CAR_LAMP_UPPER_Y, car_lamp_upper_y, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_F(IMAGE_PARAM_ID_CAR_LAMP_BRIDGE_MAX_GAP, car_lamp_bridge_max_gap, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_I(IMAGE_PARAM_ID_BEACON_MIN_COMPONENT_AREA, beacon_min_component_area, 0, IMAGE_PARAM_AREA_MAX),
    PARAM_I(IMAGE_PARAM_ID_BEACON_EDGE_MIN_AREA, beacon_edge_min_area, 0, IMAGE_PARAM_AREA_MAX),
    PARAM_I(IMAGE_PARAM_ID_BEACON_TOP_EDGE_MAX_AREA, beacon_top_edge_max_area, 0, IMAGE_PARAM_AREA_MAX),
    PARAM_I(IMAGE_PARAM_ID_BEACON_EDGE_MAX_AREA, beacon_edge_max_area, 0, IMAGE_PARAM_AREA_MAX),
    PARAM_I(IMAGE_PARAM_ID_CAR_LAMP_MIN_AREA, car_lamp_min_area, 0, IMAGE_PARAM_AREA_MAX),
    PARAM_I(IMAGE_PARAM_ID_CAR_LAMP_MAX_AREA, car_lamp_max_area, 0, IMAGE_PARAM_AREA_MAX),
    PARAM_F(IMAGE_PARAM_ID_CAR_LAMP_MIN_ELONGATION, car_lamp_min_elongation, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_F(IMAGE_PARAM_ID_CAR_LAMP_MIN_LENGTH, car_lamp_min_length, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_I(IMAGE_PARAM_ID_BEACON_ISOLATED_GRAY_MIN, beacon_isolated_gray_min, 0, IMAGE_PARAM_GRAY_MAX),
    PARAM_I(IMAGE_PARAM_ID_BEACON_ISOLATED_BG_MAX, beacon_isolated_bg_max, 0, IMAGE_PARAM_GRAY_MAX),
    PARAM_F(IMAGE_PARAM_ID_BEACON_LOCAL_RING_INNER, beacon_local_ring_inner, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_F(IMAGE_PARAM_ID_BEACON_LOCAL_RING_OUTER, beacon_local_ring_outer, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_F(IMAGE_PARAM_ID_LAMP_NEAR_BEACON_PAD, lamp_near_beacon_pad, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_I(IMAGE_PARAM_ID_LAMP_NEAR_BEACON_MIN_AREA, lamp_near_beacon_min_area, 0, IMAGE_PARAM_AREA_MAX),
    PARAM_I(IMAGE_PARAM_ID_LAMP_NEAR_BEACON_GRAY_MIN, lamp_near_beacon_gray_min, 0, IMAGE_PARAM_GRAY_MAX),
    PARAM_I(IMAGE_PARAM_ID_LAMP_NEAR_BEACON_BACKGROUND_MAX, lamp_near_beacon_background_max, 0, IMAGE_PARAM_GRAY_MAX),
    PARAM_F(IMAGE_PARAM_ID_B0_MATCH_DISTANCE, b0_match_distance, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_F(IMAGE_PARAM_ID_KALMAN_GATE_DISTANCE, kalman_gate_distance, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_F(IMAGE_PARAM_ID_KALMAN_NEW_TARGET_DISTANCE, kalman_new_target_distance, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_I(IMAGE_PARAM_ID_B0_INIT_CONFIRM_FRAMES, b0_init_confirm_frames, 1, IMAGE_PARAM_FRAME_MAX),
    PARAM_I(IMAGE_PARAM_ID_BEACON_MAX_MISSES, beacon_max_misses, 0, IMAGE_PARAM_FRAME_MAX),
    PARAM_F(IMAGE_PARAM_ID_FILTER_POS_ALPHA, filter_pos_alpha, 0, 1),
    PARAM_F(IMAGE_PARAM_ID_FILTER_VEL_ALPHA, filter_vel_alpha, 0, 1),
    PARAM_I(IMAGE_PARAM_ID_STREAM_MODE, stream_mode, IMAGE_STREAM_MODE_RAW, IMAGE_STREAM_MODE_DETECTED_OVERLAY),
    PARAM_F(IMAGE_PARAM_ID_CAR_LAMP_MIN_WIDTH, car_lamp_min_width, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_F(IMAGE_PARAM_ID_CAR_LAMP_NARROW_MIN_WIDTH, car_lamp_narrow_min_width, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_F(IMAGE_PARAM_ID_CAR_LAMP_NARROW_MIN_ELONGATION, car_lamp_narrow_min_elongation, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_I(IMAGE_PARAM_ID_CAR_LAMP_UPPER_MIN_AREA, car_lamp_upper_min_area, 0, IMAGE_PARAM_AREA_MAX),
    PARAM_F(IMAGE_PARAM_ID_CAR_LAMP_UPPER_MIN_LENGTH, car_lamp_upper_min_length, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_F(IMAGE_PARAM_ID_CAR_LAMP_UPPER_MIN_WIDTH, car_lamp_upper_min_width, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_F(IMAGE_PARAM_ID_CAR_LAMP_COMPACT_MIN_Y, car_lamp_compact_min_y, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_I(IMAGE_PARAM_ID_CAR_LAMP_COMPACT_MIN_AREA, car_lamp_compact_min_area, 0, IMAGE_PARAM_AREA_MAX),
    PARAM_F(IMAGE_PARAM_ID_CAR_LAMP_COMPACT_MIN_LENGTH, car_lamp_compact_min_length, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_F(IMAGE_PARAM_ID_CAR_LAMP_COMPACT_MIN_WIDTH, car_lamp_compact_min_width, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_F(IMAGE_PARAM_ID_CAR_LAMP_COMPACT_MIN_ELONGATION, car_lamp_compact_min_elongation, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_F(IMAGE_PARAM_ID_VERTICAL_GLARE_MIN_ELONGATION, vertical_glare_min_elongation, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_I(IMAGE_PARAM_ID_VERTICAL_GLARE_MAX_GRAY, vertical_glare_max_gray, 0, IMAGE_PARAM_GRAY_MAX),
    PARAM_F(IMAGE_PARAM_ID_LINEAR_MIN_ELONGATION, linear_min_elongation, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_I(IMAGE_PARAM_ID_WEAK_CENTER_THRESHOLD, weak_center_threshold, 0, IMAGE_PARAM_GRAY_MAX),
    PARAM_I(IMAGE_PARAM_ID_WEAK_CENTER_MIN_AREA, weak_center_min_area, 0, IMAGE_PARAM_AREA_MAX),
    PARAM_I(IMAGE_PARAM_ID_WEAK_CENTER_MAX_AREA, weak_center_max_area, 0, IMAGE_PARAM_AREA_MAX),
    PARAM_I(IMAGE_PARAM_ID_WEAK_CENTER_MIN_GRAY, weak_center_min_gray, 0, IMAGE_PARAM_GRAY_MAX),
    PARAM_I(IMAGE_PARAM_ID_WEAK_CENTER_MAX_BG, weak_center_max_bg, 0, IMAGE_PARAM_GRAY_MAX),
    PARAM_I(IMAGE_PARAM_ID_SHAPE_MIN_AREA, shape_min_area, 0, IMAGE_PARAM_AREA_MAX),
    PARAM_F(IMAGE_PARAM_ID_SHAPE_MAX_RATIO, shape_max_ratio, 1, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_I(IMAGE_PARAM_ID_SHAPE_MIN_FILL_PERCENT, shape_min_fill_percent, 0, 100),
    PARAM_I(IMAGE_PARAM_ID_SHAPE_SMALL_MIN_FILL_PERCENT, shape_small_min_fill_percent, 0, 100),
    PARAM_F(IMAGE_PARAM_ID_TOP_VERTICAL_MIN_ELONGATION, top_vertical_min_elongation, 0, IMAGE_PARAM_DISTANCE_MAX),
    PARAM_I(IMAGE_PARAM_ID_SATURATED_TOP_MIN_GRAY, saturated_top_min_gray, 0, IMAGE_PARAM_GRAY_MAX),
    PARAM_I(IMAGE_PARAM_ID_BEACON_BINARY_THRESHOLD, g_beacon_binary_threshold, 0, IMAGE_PARAM_GRAY_MAX),
    PARAM_CUSTOM_I(IMAGE_PARAM_ID_EXP_TIME, IMAGE_EXP_TIME_MIN, IMAGE_EXP_TIME_MAX,
                   exposure_param_set, exposure_param_get)
};

typedef char image_param_count_must_match[(sizeof(s_params) / sizeof(s_params[0]) == 57U) ? 1 : -1];

uint16 image_param_count(void)
{
    return (uint16)(sizeof(s_params) / sizeof(s_params[0]));
}

uint8 image_param_info(uint16 index,
                       uint16 *parameter_id,
                       uint8 *type,
                       float *minimum,
                       float *maximum)
{
    const image_param_descriptor_t *param;

    if((index >= image_param_count()) || (parameter_id == NULL) || (type == NULL) ||
       (minimum == NULL) || (maximum == NULL))
    {
        return IMAGE_PARAM_STATUS_ERROR;
    }
    param = &s_params[index];
    *parameter_id = param->id;
    *type = param->type;
    *minimum = param->minimum;
    *maximum = param->maximum;
    return IMAGE_PARAM_STATUS_OK;
}

static const image_param_descriptor_t *find_param(uint16 id)
{
    uint16 index;
    for(index = 0U; index < (uint16)(sizeof(s_params) / sizeof(s_params[0])); index++)
    {
        if(s_params[index].id == id) return &s_params[index];
    }
    return NULL;
}

static uint32 value_bits(const image_param_descriptor_t *param)
{
    uint32 bits = 0U;
    memcpy(&bits, param->value_ptr, sizeof(bits));
    return bits;
}

static int32 bits_to_int32(uint32 bits)
{
    int32 value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static uint32 int32_to_bits(int32 value)
{
    uint32 bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static uint8 exposure_param_set(uint32 requested_bits, uint32 *actual_bits)
{
    int32 actual_value;
    uint8 status = image_camera_param_set_exposure(
        bits_to_int32(requested_bits), &actual_value);
    *actual_bits = int32_to_bits(actual_value);
    return status;
}

static uint8 exposure_param_get(uint32 *actual_bits)
{
    int32 actual_value;
    uint8 status = image_camera_param_get_exposure(&actual_value);
    *actual_bits = int32_to_bits(actual_value);
    return status;
}

static uint8 current_values_valid(const image_param_descriptor_t *param)
{
    float value;
    int32 int_value;

    if(param->type == IMAGE_PARAM_TYPE_FLOAT32)
    {
        memcpy(&value, param->value_ptr, sizeof(value));
        if((isfinite(value) == 0) || (value < param->minimum) || (value > param->maximum)) return 0U;
    }
    else
    {
        memcpy(&int_value, param->value_ptr, sizeof(int_value));
        if(((float)int_value < param->minimum) || ((float)int_value > param->maximum)) return 0U;
    }

    return ((car_lamp_min_area <= car_lamp_max_area) &&
            (beacon_local_ring_inner < beacon_local_ring_outer) &&
            (weak_center_min_area <= weak_center_max_area)) ? 1U : 0U;
}

uint8 image_param_set(uint8 type, uint16 id, uint32 requested_bits, uint32 *actual_bits)
{
    const image_param_descriptor_t *param;
    uint32 previous_bits;

    if(actual_bits == NULL) return IMAGE_PARAM_STATUS_ERROR;
    *actual_bits = 0U;

    param = find_param(id);
    if(param == NULL) return IMAGE_PARAM_STATUS_NOT_FOUND;
    if(type != param->type) return IMAGE_PARAM_STATUS_MISMATCH;
    if(param->set_handler != NULL)
    {
        return param->set_handler(requested_bits, actual_bits);
    }
    *actual_bits = value_bits(param);
    if(*actual_bits == requested_bits) return IMAGE_PARAM_STATUS_OK;

    previous_bits = *actual_bits;
    memcpy(param->value_ptr, &requested_bits, sizeof(requested_bits));
    if(current_values_valid(param) == 0U)
    {
        memcpy(param->value_ptr, &previous_bits, sizeof(previous_bits));
        return IMAGE_PARAM_STATUS_OUT_OF_RANGE;
    }

    *actual_bits = value_bits(param);
    if(id != IMAGE_PARAM_ID_STREAM_MODE)
    {
        image_algorithm_params_changed();
    }
    return (*actual_bits == requested_bits) ? IMAGE_PARAM_STATUS_OK : IMAGE_PARAM_STATUS_ERROR;
}

uint8 image_param_get(uint8 type, uint16 id, uint32 *actual_bits)
{
    const image_param_descriptor_t *param;

    if(actual_bits == NULL) return IMAGE_PARAM_STATUS_ERROR;
    *actual_bits = 0U;
    param = find_param(id);
    if(param == NULL) return IMAGE_PARAM_STATUS_NOT_FOUND;
    if(type != param->type) return IMAGE_PARAM_STATUS_MISMATCH;
    if(param->get_handler != NULL)
    {
        return param->get_handler(actual_bits);
    }
    *actual_bits = value_bits(param);
    return IMAGE_PARAM_STATUS_OK;
}

uint8 image_param_set_int32(uint16 id, int32 value, int32 *actual_value)
{
    uint32 actual_bits;
    uint8 status;
    if(actual_value == NULL) return IMAGE_PARAM_STATUS_ERROR;
    status = image_param_set(IMAGE_PARAM_TYPE_INT32, id, int32_to_bits(value), &actual_bits);
    *actual_value = bits_to_int32(actual_bits);
    return status;
}

uint8 image_param_get_int32(uint16 id, int32 *actual_value)
{
    uint32 actual_bits;
    uint8 status;
    if(actual_value == NULL) return IMAGE_PARAM_STATUS_ERROR;
    status = image_param_get(IMAGE_PARAM_TYPE_INT32, id, &actual_bits);
    *actual_value = bits_to_int32(actual_bits);
    return status;
}
