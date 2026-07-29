#ifndef __APP_SERVO_H__
#define __APP_SERVO_H__

#include "headfile.h"

typedef struct
{
    TIMn_enum tim;
    TIMn_CHn_enum channel;
    uint16_t freq_hz;
    uint16_t min_us;
    uint16_t mid_us;
    uint16_t max_us;
    float min_angle;
    float max_angle;
} app_servo_t;

void app_servo_init(app_servo_t *servo, TIMn_enum tim, TIMn_CHn_enum channel);
void app_servo_config_pulse(app_servo_t *servo, uint16_t min_us, uint16_t mid_us, uint16_t max_us);
void app_servo_set_pulse_us(app_servo_t *servo, uint16_t pulse_us);
void app_servo_set_angle(app_servo_t *servo, float angle);
void app_servo_stop(app_servo_t *servo);

#endif

