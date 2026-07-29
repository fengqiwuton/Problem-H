#ifndef __APP_MOTOR_H__
#define __APP_MOTOR_H__

#include "headfile.h"

typedef enum
{
    APP_MOTOR_LEFT = 0,
    APP_MOTOR_RIGHT = 1
} app_motor_side_t;

void app_motor_init(void);
void app_motor_set(int16_t left_speed, int16_t right_speed);
void app_motor_forward(int16_t speed);
void app_motor_backward(int16_t speed);
void app_motor_turn_left(int16_t speed);
void app_motor_turn_right(int16_t speed);
void app_motor_stop(void);

#endif

