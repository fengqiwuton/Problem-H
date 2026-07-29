#include "app_key.h"

void app_key_init(app_key_t *key, GPIOn_enum port, Pinx_enum pin, uint8_t active_level, uint16_t debounce_ms)
{
    if (key == 0)
    {
        return;
    }

    key->port = port;
    key->pin = pin;
    key->active_level = active_level ? 1 : 0;
    key->debounce_ms = debounce_ms;
    key->elapsed_ms = 0;
    key->pressed_event = 0;
    key->released_event = 0;

    gpio_init(port, pin, active_level ? ID : IU);
    key->last_raw_level = gpio_get(port, pin);
    key->stable_level = key->last_raw_level;
}

void app_key_update(app_key_t *key, uint16_t dt_ms)
{
    uint8_t raw;

    if (key == 0)
    {
        return;
    }

    raw = gpio_get(key->port, key->pin);
    if (raw != key->last_raw_level)
    {
        key->last_raw_level = raw;
        key->elapsed_ms = 0;
        return;
    }

    if (key->elapsed_ms < key->debounce_ms)
    {
        key->elapsed_ms += dt_ms;
        return;
    }

    if (raw != key->stable_level)
    {
        key->stable_level = raw;
        if (raw == key->active_level)
        {
            key->pressed_event = 1;
        }
        else
        {
            key->released_event = 1;
        }
    }
}

uint8_t app_key_is_pressed(const app_key_t *key)
{
    if (key == 0)
    {
        return 0;
    }

    return key->stable_level == key->active_level;
}

uint8_t app_key_take_pressed(app_key_t *key)
{
    uint8_t event;

    if (key == 0)
    {
        return 0;
    }

    event = key->pressed_event;
    key->pressed_event = 0;
    return event;
}

uint8_t app_key_take_released(app_key_t *key)
{
    uint8_t event;

    if (key == 0)
    {
        return 0;
    }

    event = key->released_event;
    key->released_event = 0;
    return event;
}

