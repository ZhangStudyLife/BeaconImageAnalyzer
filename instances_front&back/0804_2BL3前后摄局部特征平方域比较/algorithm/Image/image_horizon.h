#ifndef IMAGE_HORIZON_H_
#define IMAGE_HORIZON_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMAGE_HORIZON_WIDTH   (188U)
#define IMAGE_HORIZON_HEIGHT  (120U)

extern float g_image_horizon_y[IMAGE_HORIZON_WIDTH];
extern uint8 g_image_horizon_column_valid[IMAGE_HORIZON_WIDTH];
extern uint8 g_image_horizon_valid;
extern uint8 g_image_horizon_extrapolated;

void image_horizon_init(void);
void image_horizon_update(uint8 board_id,
                          float roll_deg,
                          float pitch_deg,
                          float height_mm,
                          uint8 attitude_valid,
                          uint8 height_valid);
uint8 image_horizon_get_y(uint16 x, float *y);
void image_horizon_draw_overlay(uint8 *image, uint8 gray);

#ifdef __cplusplus
}
#endif

#endif
