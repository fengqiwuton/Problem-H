#include "app_motor.h"
#include "motor.h"

static int16_t app_motor_abs_i16(int16_t value)
{
    return value < 0 ? (int16_t)-value : value;
}

void app_motor_init(void)
{
    IIC_Motor_Init();
    app_motor_stop();
}

void app_motor_set(int16_t left_speed, int16_t right_speed)
{
    control_speed(left_speed, left_speed, right_speed, right_speed);
}

void app_motor_forward(int16_t speed)
{
    speed = app_motor_abs_i16(speed);
    app_motor_set(speed, speed);
}

void app_motor_backward(int16_t speed)
{
    speed = app_motor_abs_i16(speed);
    app_motor_set((int16_t)-speed, (int16_t)-speed);
}

void app_motor_turn_left(int16_t speed)
{
    speed = app_motor_abs_i16(speed);
    app_motor_set((int16_t)-speed, speed);
}

void app_motor_turn_right(int16_t speed)
{
    speed = app_motor_abs_i16(speed);
    app_motor_set(speed, (int16_t)-speed);
}

void app_motor_stop(void)
{
    app_motor_set(0, 0);
}

