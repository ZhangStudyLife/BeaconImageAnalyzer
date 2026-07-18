#include <string.h>

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"
#include "Image/image.h"

uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
volatile uint8 mt9v03x_finish_flag;
uint16 g_mt9v03x_exp_time;

uint8 mt9v03x_init(void)
{
    return 0U;
}

void flash_init(void)
{
}

uint8 flash_check(uint32 sector, uint32 page)
{
    (void)sector;
    (void)page;
    return 0U;
}

void flash_read_page(uint32 sector, uint32 page, uint32 *buffer, uint32 length)
{
    (void)sector;
    (void)page;
    memset(buffer, 0, length * sizeof(*buffer));
}

void flash_erase_page(uint32 sector, uint32 page)
{
    (void)sector;
    (void)page;
}

void flash_write_page(uint32 sector, uint32 page, const uint32 *buffer, uint32 length)
{
    (void)sector;
    (void)page;
    (void)buffer;
    (void)length;
}

void codex_2bl3_process_frame(
    const uint8 frame[MT9V03X_H][MT9V03X_W])
{
    memcpy(mt9v03x_image, frame, sizeof(mt9v03x_image));
    mt9v03x_finish_flag = 1U;
    image_update();
}
