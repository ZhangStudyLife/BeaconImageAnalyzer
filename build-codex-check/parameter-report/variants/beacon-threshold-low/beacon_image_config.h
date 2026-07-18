#ifndef BEACON_IMAGE_CONFIG_H_
#define BEACON_IMAGE_CONFIG_H_

#define BEACON_AREA_CALIBRATION_ENABLED 1
#define BEACON_BINARY_THRESHOLD_DEFAULT 90

/*
 * 0: 前摄标定表；1: 后摄标定表。
 * 运行时也可调用 beacon_image_set_camera_board_id() 切换。
 */
#define BEACON_DEFAULT_CAMERA_BOARD_ID 0

#endif

