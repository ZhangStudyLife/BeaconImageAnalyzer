#ifndef BEACON_IMAGE_CONFIG_H_
#define BEACON_IMAGE_CONFIG_H_

/* 固定二值化阈值，与车端 image.c 保持一致 */
#define BEACON_BINARY_THRESHOLD     100

/* 连通域面积过滤 */
#define BEACON_MIN_COMPONENT_AREA   4
#define BEACON_MAX_COMPONENT_AREA   5000

/* 队列大小 = 图像像素数 */
#define BEACON_QUEUE_SIZE           (BEACON_IMAGE_W * BEACON_IMAGE_H)

#endif
