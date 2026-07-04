#ifndef IMAGE_H_
#define IMAGE_H_

#ifdef BEACON_ANALYZER_ADAPTER
typedef unsigned char uint8;
#include "image_data.h"
#else
#include "zf_common_headfile.h"
#include "Image/image_data.h"
#endif

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

extern struct image_data g_image_data;

void image_init(void);
void image_update(void);
uint8 *image_get_frame_buffer(void);

#ifdef __cplusplus
}
#endif

#endif
