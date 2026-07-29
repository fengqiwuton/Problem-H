#include "app_servo.h"

static uint16_t app_servo_limit_u16(uint16_t value, uint16_t min_value, uint16_t max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

void app_servo_init(app_servo_t *servo, TIMn_enum tim, TIMn_CHn_enum channel)
{
    if (servo == 0)
    {
        return;
    }

    servo->tim = tim;
    servo->channel = channel;
    servo->freq_hz = 50;
    servo->min_us = 500;
    servo->mid_us = 1500;
    servo->max_us = 2500;
    servo->min_angle = 0.0f;
    servo->max_angle = 180.0f;

    pwm_init(tim, channel, servo->freq_hz);
    app_servo_set_pulse_us(servo, servo->mid_us);
}

void app_servo_config_pulse(app_servo_t *servo, uint16_t min_us, uint16_t mid_us, uint16_t max_us)
{
    if (servo == 0)
    {
        return;
    }

    servo->min_us = min_us;
    servo->mid_us = mid_us;
    servo->max_us = max_us;
}

void app_servo_set_pulse_us(app_servo_t *servo, uint16_t pulse_us)
{
    uint32_t period_us;
    uint32_t duty;

    if (servo == 0 || servo->freq_hz == 0)
    {
        return;
    }

    pulse_us = app_servo_limit_u16(pulse_us, servo->min_us, servo->max_us);
    period_us = 1000000UL / servo->freq_hz;
    duty = (uint32_t)pulse_us * MAX_DUTY / period_us;
    if (duty > MAX_DUTY)
    {
        duty = MAX_DUTY;
    }
    pwm_update(servo->tim, servo->channel, (uint16_t)duty);
}

void app_servo_set_angle(app_servo_t *servo, float angle)
{
    float ratio;
    uint16_t pulse;

    if (servo == 0)
    {
        return;
    }

    if (angle < servo->min_angle)
    {
        angle = servo->min_angle;
    }
    if (angle > servo->max_angle)
    {
        angle = servo->max_angle;
    }

    ratio = (angle - servo->min_angle) / (servo->max_angle - servo->min_angle);
    pulse = (uint16_t)(servo->min_us + ratio * (servo->max_us - servo->min_us));
    app_servo_set_pulse_us(servo, pulse);
}

void app_servo_stop(app_servo_t *servo)
{
    if (servo == 0)
    {
        return;
    }

    pwm_update(servo->tim, servo->channel, 0);
}

