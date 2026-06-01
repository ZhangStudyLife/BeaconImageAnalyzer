#ifndef IMAGE_RESULT_H
#define IMAGE_RESULT_H

#include "image.h"
#include "zf_device_mt9v03x.h"

constexpr int BEACON_IMAGE_W = MT9V03X_W;
constexpr int BEACON_IMAGE_H = MT9V03X_H;
constexpr int BEACON_MAX_CIRCLE_COUNT = IMAGE_MAX_BEACON_COUNT;
constexpr int BEACON_MAX_CAR_LAMP_COUNT = IMAGE_MAX_CAR_LAMP_COUNT;
constexpr float BEACON_IMAGE_TARGET_PIXEL_X = (float)MT9V03X_W * 0.5f;
constexpr float BEACON_IMAGE_TARGET_PIXEL_Y = (float)MT9V03X_H * 0.5f;

struct beacon_result_t
{
    beacon_circle_t circles[BEACON_MAX_CIRCLE_COUNT] = {};
    uint8 count = 0U;
    beacon_circle_t beacons[BEACON_MAX_CIRCLE_COUNT] = {};
    uint8 beacon_count = 0U;
    beacon_rect_t car_lamps[BEACON_MAX_CAR_LAMP_COUNT] = {};
    uint8 car_lamp_count = 0U;
};

#endif
