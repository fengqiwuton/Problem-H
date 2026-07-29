#include "ball_balance.h"

int ball_position_raw = 0;
uint8_t ball_position_valid = 0;

static int ball_target_raw = 0;
static int last_pos_error = 0;
static uint16_t servo_pulse_us = SERVO_CENTER_US;
static app_servo_t balance_servo;

static int limit_int_bb(int value, int min_value, int max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

void ball_balance_init(void)
{
    app_servo_init(&balance_servo, TIM_3, TIM3_CH3);
    app_servo_config_pulse(&balance_servo, SERVO_MIN_US, SERVO_CENTER_US, SERVO_MAX_US);
    app_servo_set_pulse_us(&balance_servo, SERVO_CENTER_US);
    servo_pulse_us = SERVO_CENTER_US;

    ball_target_raw = 0;
    ball_position_raw = 0;
    ball_position_valid = 0;
    last_pos_error = 0;
}

void ball_balance_set_target(int target_raw)
{
    ball_target_raw = target_raw;
}

void ball_balance_servo_center(void)
{
    servo_pulse_us = SERVO_CENTER_US;
    app_servo_set_pulse_us(&balance_servo, SERVO_CENTER_US);
    last_pos_error = 0;
}

int ball_balance_pos_error(void)
{
    return ball_position_raw - ball_target_raw;
}

/* Single-loop PD: ball position → servo pulse directly.
 * No beam angle sensor needed — camera provides the feedback. */
void ball_balance_update(void)
{
    int pos_error;
    int delta;

    if (!ball_position_valid)
    {
        app_servo_set_pulse_us(&balance_servo, servo_pulse_us);
        return;
    }

    pos_error = ball_target_raw - ball_position_raw;

    delta = (BALANCE_KP * pos_error + BALANCE_KD * (pos_error - last_pos_error)) / 100;
    delta = limit_int_bb(delta, -BALANCE_MAX_DELTA, BALANCE_MAX_DELTA);

    servo_pulse_us = SERVO_CENTER_US + delta;
    servo_pulse_us = limit_int_bb(servo_pulse_us, SERVO_MIN_US, SERVO_MAX_US);

    app_servo_set_pulse_us(&balance_servo, servo_pulse_us);
    last_pos_error = pos_error;
}
