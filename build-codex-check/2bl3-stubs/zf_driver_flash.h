#ifndef CODEX_2BL3_ZF_DRIVER_FLASH_H
#define CODEX_2BL3_ZF_DRIVER_FLASH_H

#include "zf_common_headfile.h"

#define FLASH_PAGE_NUM 64U
#define FLASH_PAGE_SIZE 4096U

void flash_init(void);
uint8 flash_check(uint32 sector, uint32 page);
void flash_read_page(uint32 sector, uint32 page, uint32 *buffer, uint32 length);
void flash_erase_page(uint32 sector, uint32 page);
void flash_write_page(uint32 sector, uint32 page, const uint32 *buffer, uint32 length);

#endif
