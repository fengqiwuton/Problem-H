#include "app_hcsr04.h"

#define HCSR04_DEFAULT_TIMEOUT_US 30000UL
#define HCSR04_TRIGGER_US         12

static uint8_t hcsr04_wait_level(app_hcsr04_t *sensor, uint8_t level, uint32_t timeout_us)
{
    uint32_t elapsed = 0;

    while (gpio_get(sensor->echo_port, sensor->echo_pin) != level)
    {
        if (elapsed >= timeout_us)
        {
            return 0;
        }
        delay_us(1);
        elapsed++;
    }

    return 1;
}

void app_hcsr04_init(app_hcsr04_t *sensor, GPIOn_enum trig_port, Pinx_enum trig_pin,
                     GPIOn_enum echo_port, Pinx_enum echo_pin)
{
    if (sensor == 0)
    {
        return;
    }

    sensor->trig_port = trig_port;
    sensor->trig_pin = trig_pin;
    sensor->echo_port = echo_port;
    sensor->echo_pin = echo_pin;
    sensor->timeout_us = HCSR04_DEFAULT_TIMEOUT_US;
    sensor->last_echo_us = 0;
    sensor->last_distance_mm = 0;
    sensor->last_ok = 0;

    gpio_init(trig_port, trig_pin, OUT_PP);
    gpio_init(echo_port, echo_pin, ID);
    gpio_set(trig_port, trig_pin, 0);
}

uint8_t app_hcsr04_read(app_hcsr04_t *sensor)
{
    uint32_t echo_us = 0;

    if (sensor == 0)
    {
        return 0;
    }

    sensor->last_ok = 0;
    gpio_set(sensor->trig_port, sensor->trig_pin, 0);
    delay_us(2);
    gpio_set(sensor->trig_port, sensor->trig_pin, 1);
    delay_us(HCSR04_TRIGGER_US);
    gpio_set(sensor->trig_port, sensor->trig_pin, 0);

    if (!hcsr04_wait_level(sensor, 1, sensor->timeout_us))
    {
        sensor->last_echo_us = 0;
        sensor->last_distance_mm = 0;
        return 0;
    }

    while (gpio_get(sensor->echo_port, sensor->echo_pin))
    {
        if (echo_us >= sensor->timeout_us)
        {
            sensor->last_echo_us = echo_us;
            sensor->last_distance_mm = 0;
            return 0;
        }
        delay_us(1);
        echo_us++;
    }

    sensor->last_echo_us = echo_us;
    sensor->last_distance_mm = (uint16_t)((echo_us * 343UL) / 2000UL);
    sensor->last_ok = 1;
    return 1;
}

uint16_t app_hcsr04_distance_mm(const app_hcsr04_t *sensor)
{
    return sensor == 0 ? 0 : sensor->last_distance_mm;
}

uint32_t app_hcsr04_echo_us(const app_hcsr04_t *sensor)
{
    return sensor == 0 ? 0 : sensor->last_echo_us;
}

uint8_t app_hcsr04_ok(const app_hcsr04_t *sensor)
{
    return sensor == 0 ? 0 : sensor->last_ok;
}

uint8_t app_hcsr04_obstacle(const app_hcsr04_t *sensor, uint16_t threshold_mm)
{
    if (sensor == 0 || !sensor->last_ok)
    {
        return 0;
    }

    return sensor->last_distance_mm <= threshold_mm;
}
