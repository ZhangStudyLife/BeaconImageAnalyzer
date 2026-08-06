#ifndef IMAGE_H_
#define IMAGE_H_

#include "zf_common_headfile.h"
#include "Image/image_data.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float x;
    float y;
    float radius;
    float area;
    unsigned char valid;
} beacon_circle_t;

typedef struct
{
    float cx;
    float cy;
    float width;
    float length;
    float angle;
    unsigned char valid;
} beacon_rect_t;

#define IMAGE_PARAM_TYPE_FLOAT32                         (0U) /* 参数值为IEEE-754单精度浮点位模式 */
#define IMAGE_PARAM_TYPE_INT32                           (1U) /* 参数值为有符号32位整数位模式 */
#define IMAGE_ALGORITHM_BUILD_ID                         (0x20260931UL)

#define IMAGE_PARAM_ID_BEACON_EDGE_THRESHOLD          (0x0100U) /* 边缘信标阈值 */
#define IMAGE_PARAM_ID_BEACON_TRACK_THRESHOLD         (0x0101U) /* 跟踪补强阈值 */
#define IMAGE_PARAM_ID_CAR_LAMP_BINARY_THRESHOLD      (0x0102U) /* 车灯普通阈值 */
#define IMAGE_PARAM_ID_CAR_LAMP_UPPER_THRESHOLD       (0x0103U) /* 车灯上部阈值 */
#define IMAGE_PARAM_ID_CAR_LAMP_UPPER_Y               (0x0104U) /* 车灯上部区域边界 */
#define IMAGE_PARAM_ID_CAR_LAMP_BRIDGE_MAX_GAP        (0x0105U) /* 车灯横向断点桥接距离 */
#define IMAGE_PARAM_ID_BEACON_MIN_COMPONENT_AREA      (0x0106U) /* 普通信标最小面积 */
#define IMAGE_PARAM_ID_BEACON_EDGE_MIN_AREA           (0x0107U) /* 边缘信标最小面积 */
#define IMAGE_PARAM_ID_BEACON_TOP_EDGE_MAX_AREA       (0x0108U) /* 顶部信标最大面积 */
#define IMAGE_PARAM_ID_BEACON_EDGE_MAX_AREA           (0x0109U) /* 侧边信标最大面积 */
#define IMAGE_PARAM_ID_CAR_LAMP_MIN_AREA              (0x010AU) /* 车灯最小面积 */
#define IMAGE_PARAM_ID_CAR_LAMP_MAX_AREA              (0x010BU) /* 车灯普通最大面积 */
#define IMAGE_PARAM_ID_CAR_LAMP_MIN_ELONGATION        (0x010DU) /* 车灯最小长宽比 */
#define IMAGE_PARAM_ID_CAR_LAMP_MIN_LENGTH            (0x010FU) /* 后摄车灯最小长度 */
#define IMAGE_PARAM_ID_BEACON_ISOLATED_GRAY_MIN       (0x0110U) /* 孤立点最小峰值灰度 */
#define IMAGE_PARAM_ID_BEACON_ISOLATED_BG_MAX         (0x0111U) /* 孤立点最大背景灰度 */
#define IMAGE_PARAM_ID_BEACON_LOCAL_RING_INNER        (0x0112U) /* 背景环内半径 */
#define IMAGE_PARAM_ID_BEACON_LOCAL_RING_OUTER        (0x0113U) /* 背景环外半径 */
#define IMAGE_PARAM_ID_LAMP_NEAR_BEACON_PAD           (0x0114U) /* 车灯邻域外扩距离 */
#define IMAGE_PARAM_ID_LAMP_NEAR_BEACON_MIN_AREA      (0x0115U) /* 近车灯信标最小面积 */
#define IMAGE_PARAM_ID_LAMP_NEAR_BEACON_GRAY_MIN      (0x0116U) /* 近车灯信标最小灰度 */
#define IMAGE_PARAM_ID_LAMP_NEAR_BEACON_BACKGROUND_MAX (0x0117U) /* 近车灯信标最大背景 */
#define IMAGE_PARAM_ID_B0_MATCH_DISTANCE              (0x0118U) /* 信标匹配距离 */
#define IMAGE_PARAM_ID_KALMAN_GATE_DISTANCE           (0x0119U) /* 已确认目标门控距离 */
#define IMAGE_PARAM_ID_KALMAN_NEW_TARGET_DISTANCE     (0x011AU) /* 新目标重建距离 */
#define IMAGE_PARAM_ID_B0_INIT_CONFIRM_FRAMES         (0x011BU) /* 目标确认帧数 */
#define IMAGE_PARAM_ID_BEACON_MAX_MISSES              (0x011CU) /* 信标最大丢失帧数 */
#define IMAGE_PARAM_ID_FILTER_POS_ALPHA                (0x011DU) /* 位置滤波系数 */
#define IMAGE_PARAM_ID_FILTER_VEL_ALPHA                (0x011EU) /* 速度滤波系数 */
#define IMAGE_PARAM_ID_STREAM_MODE                     (0x0120U) /* 图传内容模式 */
#define IMAGE_PARAM_ID_CAR_LAMP_MIN_WIDTH              (0x0121U)
#define IMAGE_PARAM_ID_CAR_LAMP_NARROW_MIN_WIDTH       (0x0122U)
#define IMAGE_PARAM_ID_CAR_LAMP_NARROW_MIN_ELONGATION  (0x0123U)
#define IMAGE_PARAM_ID_CAR_LAMP_UPPER_MIN_AREA         (0x0124U)
#define IMAGE_PARAM_ID_CAR_LAMP_UPPER_MIN_LENGTH       (0x0125U)
#define IMAGE_PARAM_ID_CAR_LAMP_UPPER_MIN_WIDTH        (0x0126U)
#define IMAGE_PARAM_ID_CAR_LAMP_COMPACT_MIN_Y          (0x0127U)
#define IMAGE_PARAM_ID_CAR_LAMP_COMPACT_MIN_AREA       (0x0128U)
#define IMAGE_PARAM_ID_CAR_LAMP_COMPACT_MIN_LENGTH     (0x0129U)
#define IMAGE_PARAM_ID_CAR_LAMP_COMPACT_MIN_WIDTH      (0x012AU)
#define IMAGE_PARAM_ID_CAR_LAMP_COMPACT_MIN_ELONGATION (0x012BU)
#define IMAGE_PARAM_ID_VERTICAL_GLARE_MIN_ELONGATION    (0x012FU)
#define IMAGE_PARAM_ID_VERTICAL_GLARE_MAX_GRAY          (0x0130U)
#define IMAGE_PARAM_ID_LINEAR_MIN_ELONGATION            (0x0131U)
#define IMAGE_PARAM_ID_WEAK_CENTER_THRESHOLD            (0x0132U)
#define IMAGE_PARAM_ID_WEAK_CENTER_MIN_AREA             (0x0133U)
#define IMAGE_PARAM_ID_WEAK_CENTER_MAX_AREA             (0x0134U)
#define IMAGE_PARAM_ID_WEAK_CENTER_MIN_GRAY             (0x0135U)
#define IMAGE_PARAM_ID_WEAK_CENTER_MAX_BG               (0x0136U)
#define IMAGE_PARAM_ID_SHAPE_MIN_AREA                   (0x0137U)
#define IMAGE_PARAM_ID_SHAPE_MAX_RATIO                  (0x0138U)
#define IMAGE_PARAM_ID_SHAPE_MIN_FILL_PERCENT           (0x0139U)
#define IMAGE_PARAM_ID_SHAPE_SMALL_MIN_FILL_PERCENT     (0x013AU)
#define IMAGE_PARAM_ID_TOP_VERTICAL_MIN_ELONGATION      (0x013FU)
#define IMAGE_PARAM_ID_SATURATED_TOP_MIN_GRAY           (0x0140U)
#define IMAGE_PARAM_ID_BEACON_BINARY_THRESHOLD          (0x0141U)
#define IMAGE_PARAM_ID_EXP_TIME                         (0x0142U)

#define IMAGE_BEACON_BINARY_THRESHOLD_DEFAULT        (120)
#define IMAGE_EXP_TIME_MIN                            (0)
#define IMAGE_EXP_TIME_MAX                            (636)

#define IMAGE_STREAM_MODE_RAW                         (0)
#define IMAGE_STREAM_MODE_LAMP_BINARY                 (1)
#define IMAGE_STREAM_MODE_BEACON_BINARY               (2)
#define IMAGE_STREAM_MODE_DETECTED_OVERLAY            (3)

#define IMAGE_PARAM_STATUS_OK                         (0U)
#define IMAGE_PARAM_STATUS_NOT_FOUND                  (1U)
#define IMAGE_PARAM_STATUS_OUT_OF_RANGE               (2U)
#define IMAGE_PARAM_STATUS_ERROR                      (3U)
#define IMAGE_PARAM_STATUS_MISMATCH                   (6U)
#define IMAGE_PARAM_STATUS_ROLLBACK_FAIL              (8U)

extern struct image_data g_image_data;
extern int32 g_beacon_binary_threshold;
extern int32 beacon_edge_threshold;
extern int32 beacon_track_threshold;
extern int32 car_lamp_binary_threshold;
extern int32 car_lamp_upper_threshold;
extern float car_lamp_upper_y;
extern float car_lamp_bridge_max_gap;
extern int32 beacon_min_component_area;
extern int32 beacon_edge_min_area;
extern int32 beacon_top_edge_max_area;
extern int32 beacon_edge_max_area;
extern int32 car_lamp_min_area;
extern int32 car_lamp_max_area;
extern float car_lamp_min_elongation;
extern float car_lamp_min_length;
extern int32 beacon_isolated_gray_min;
extern int32 beacon_isolated_bg_max;
extern float beacon_local_ring_inner;
extern float beacon_local_ring_outer;
extern float lamp_near_beacon_pad;
extern int32 lamp_near_beacon_min_area;
extern int32 lamp_near_beacon_gray_min;
extern int32 lamp_near_beacon_background_max;
extern float b0_match_distance;
extern float kalman_gate_distance;
extern float kalman_new_target_distance;
extern int32 b0_init_confirm_frames;
extern int32 beacon_max_misses;
extern float filter_pos_alpha;
extern float filter_vel_alpha;
extern int32 stream_mode;
extern float car_lamp_min_width;
extern float car_lamp_narrow_min_width;
extern float car_lamp_narrow_min_elongation;
extern int32 car_lamp_upper_min_area;
extern float car_lamp_upper_min_length;
extern float car_lamp_upper_min_width;
extern float car_lamp_compact_min_y;
extern int32 car_lamp_compact_min_area;
extern float car_lamp_compact_min_length;
extern float car_lamp_compact_min_width;
extern float car_lamp_compact_min_elongation;
extern float vertical_glare_min_elongation;
extern int32 vertical_glare_max_gray;
extern float linear_min_elongation;
extern int32 weak_center_threshold;
extern int32 weak_center_min_area;
extern int32 weak_center_max_area;
extern int32 weak_center_min_gray;
extern int32 weak_center_max_bg;
extern int32 shape_min_area;
extern float shape_max_ratio;
extern int32 shape_min_fill_percent;
extern int32 shape_small_min_fill_percent;
extern float top_vertical_min_elongation;
extern int32 saturated_top_min_gray;

void image_init(void);
void image_update(void);
uint8 *image_get_frame_buffer(void);
uint8 *image_prepare_stream_frame(void);

/**
 * @brief 按参数类型和位模式设置一个图像运行时参数。
 * @param type IMAGE_PARAM_TYPE_FLOAT32或IMAGE_PARAM_TYPE_INT32。
 * @param parameter_id 稳定参数ID或动态面积单元ID。
 * @param value_bits 请求值的32位位模式。
 * @param actual_bits 输出最终实际值的32位位模式。
 * @return IMAGE_PARAM_STATUS_*状态码。
 */
uint8 image_param_set(uint8 type, uint16 parameter_id, uint32 value_bits, uint32 *actual_bits);

/**
 * @brief 按参数类型和位模式读取一个图像运行时参数。
 * @param type IMAGE_PARAM_TYPE_FLOAT32或IMAGE_PARAM_TYPE_INT32。
 * @param parameter_id 稳定参数ID或动态面积单元ID。
 * @param actual_bits 输出实际值的32位位模式。
 * @return IMAGE_PARAM_STATUS_*状态码。
 */
uint8 image_param_get(uint8 type, uint16 parameter_id, uint32 *actual_bits);
uint16 image_param_count(void);
uint8 image_param_info(uint16 index,
                       uint16 *parameter_id,
                       uint8 *type,
                       float *minimum,
                       float *maximum);
uint8 image_param_set_int32(uint16 parameter_id, int32 value, int32 *actual_value);
uint8 image_param_get_int32(uint16 parameter_id, int32 *actual_value);

/* ImageParams模块调用的算法/摄像头适配接口。 */
void image_algorithm_params_changed(void);
uint8 image_camera_param_get_exposure(int32 *actual_value);
uint8 image_camera_param_set_exposure(int32 value, int32 *actual_value);

#ifdef __cplusplus
}
#endif

#endif
