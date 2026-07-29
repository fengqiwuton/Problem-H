#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "headfile.h"

/* Maximum age of camera data before considered invalid (in main loop ticks * LOOP_DT_MS) */
#define CAMERA_TIMEOUT_MS   200

void camera_init(void);
void camera_uart_rx(uint8_t data);
void camera_update_timeout(uint16_t dt_ms);

/* Returns 1 if camera data is fresh (within timeout) */
uint8_t camera_is_valid(void);

#endif
