#ifndef __APP_HCSR04_H__
#define __APP_HCSR04_H__

#include "headfile.h"

typedef struct
{
    GPIOn_enum trig_port;
    Pinx_enum trig_pin;
    GPIOn_enum echo_port;
    Pinx_enum echo_pin;
    uint32_t timeout_us;
    uint32_t last_echo_us;
    uint16_t last_distance_mm;
    uint8_t last_ok;
} app_hcsr04_t;

void app_hcsr04_init(app_hcsr04_t *sensor, GPIOn_enum trig_port, Pinx_enum trig_pin,
                     GPIOn_enum echo_port, Pinx_enum echo_pin);
uint8_t app_hcsr04_read(app_hcsr04_t *sensor);
uint16_t app_hcsr04_distance_mm(const app_hcsr04_t *sensor);
uint32_t app_hcsr04_echo_us(const app_hcsr04_t *sensor);
uint8_t app_hcsr04_ok(const app_hcsr04_t *sensor);
uint8_t app_hcsr04_obstacle(const app_hcsr04_t *sensor, uint16_t threshold_mm);

#endif
