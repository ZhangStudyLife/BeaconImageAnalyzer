#ifndef DOWN_DRAFT_IMAGE_DEBUG_SCREEN_H_
#define DOWN_DRAFT_IMAGE_DEBUG_SCREEN_H_

#include "zf_common_headfile.h"

#define IMAGE_DEBUG_SCREEN_MODE_DATA           (0U)
#define IMAGE_DEBUG_SCREEN_MODE_RAW            (1U)
#define IMAGE_DEBUG_SCREEN_MODE_BEACON_BINARY  (2U)
#define IMAGE_DEBUG_SCREEN_MODE_LAMP_BINARY    (3U)
#define IMAGE_DEBUG_SCREEN_MODE_OVERLAY        (4U)

uint8 ImageDebugScreen_SetMode(uint8 mode);
uint8 ImageDebugScreen_GetMode(void);

#endif
