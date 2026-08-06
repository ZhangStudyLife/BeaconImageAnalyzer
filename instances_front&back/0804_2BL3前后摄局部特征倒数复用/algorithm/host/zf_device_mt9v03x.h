#ifndef BEACON_2BL3_HOST_MT9V03X_H
#define BEACON_2BL3_HOST_MT9V03X_H

#include "zf_common_headfile.h"

#define MT9V03X_W 188
#define MT9V03X_H 120
#define MT9V03X_IMAGE_SIZE (MT9V03X_W * MT9V03X_H)

extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern volatile uint8 mt9v03x_finish_flag;
extern uint16 g_mt9v03x_exp_time;

uint8 mt9v03x_init(void);

#endif
