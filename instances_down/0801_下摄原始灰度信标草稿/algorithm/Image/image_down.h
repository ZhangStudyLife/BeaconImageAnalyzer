#ifndef DOWN_DRAFT_IMAGE_DOWN_H_
#define DOWN_DRAFT_IMAGE_DOWN_H_

#include "image.h"

extern int32 g_image_down_beacon_binary_threshold;
extern int32 g_image_down_beacon_min_area;
extern int32 g_image_down_side_edge_min_area;
extern int32 g_image_down_side_edge_threshold;
extern int32 g_image_down_car_lamp_binary_threshold;
extern int32 g_image_down_car_lamp_min_area;
extern int32 g_image_down_car_lamp_max_area;
extern float g_image_down_car_lamp_min_elongation;
extern float g_image_down_car_lamp_min_length;
extern int32 g_image_down_near_lamp_pad;
extern int32 g_image_down_near_lamp_min_area;
extern int32 g_image_down_near_lamp_isolated_min_area;
extern int32 g_image_down_near_lamp_background_max;
extern float g_image_down_match_distance;
extern float g_image_down_gate_distance;
extern float g_image_down_new_target_distance;
extern int32 g_image_down_confirm_frames;
extern int32 g_image_down_max_misses;
extern float g_image_down_filter_pos_alpha;
extern float g_image_down_filter_vel_alpha;

void image_down_init(void);
uint8 image_down_update(void);
uint8 *image_down_get_frame_buffer(void);
const uint8 *image_down_get_binary_buffer(void);
const uint8 *image_down_get_car_lamp_binary_buffer(void);
uint8 image_down_remote_param_execute(uint8 op,
                                      uint8 type,
                                      uint16 param_id,
                                      uint32 value_bits,
                                      uint32 *actual_bits);

#endif
