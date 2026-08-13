#ifndef BEACON_0813_DOWN_DESKTOP_IMAGE_H_
#define BEACON_0813_DOWN_DESKTOP_IMAGE_H_

#include "image_down.h"

#define IMAGE_ALGORITHM_BUILD_ID (0xB839E292UL)

uint16 image_param_count(void);
uint8 image_param_info(uint16 index,
                       uint16 *id,
                       uint8 *type,
                       float *minimum,
                       float *maximum);
uint8 image_param_get(uint8 type, uint16 id, uint32 *actual_bits);
uint8 image_param_set(uint8 type,
                      uint16 id,
                      uint32 value_bits,
                      uint32 *actual_bits);

#endif
