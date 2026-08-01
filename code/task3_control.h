#ifndef __TASK3_CONTROL_H__
#define __TASK3_CONTROL_H__

#include "headfile.h"
#include "task3_controller.h"

void task3_init(void);
void task3_start(void);
void task3_update(uint16_t dt_ms);
void task3_show_oled(void);
void task3_send_debug(void);
task3_state_t task3_get_state(void);
task3_controller_output_t task3_get_output(void);

#endif
