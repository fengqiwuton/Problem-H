#ifndef __TASK3_CONTROL_H__
#define __TASK3_CONTROL_H__

#include "headfile.h"

typedef enum
{
    TASK3_WAIT_CAMERA = 0,
    TASK3_WAIT_START,
    TASK3_GO_PLUS,
    TASK3_GO_MINUS,
    TASK3_HOLD_MINUS,
    TASK3_CAMERA_LOST,
    TASK3_TIMEOUT
} task3_state_t;

void task3_init(void);
void task3_start(void);
void task3_update(uint16_t dt_ms);
void task3_show_oled(void);
task3_state_t task3_get_state(void);

#endif
