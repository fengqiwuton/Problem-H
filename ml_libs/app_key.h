#ifndef __APP_KEY_H__
#define __APP_KEY_H__

#include "headfile.h"

typedef struct
{
    GPIOn_enum port;
    Pinx_enum pin;
    uint8_t active_level;
    uint8_t stable_level;
    uint8_t last_raw_level;
    uint8_t pressed_event;
    uint8_t released_event;
    uint16_t debounce_ms;
    uint16_t elapsed_ms;
} app_key_t;

void app_key_init(app_key_t *key, GPIOn_enum port, Pinx_enum pin, uint8_t active_level, uint16_t debounce_ms);
void app_key_update(app_key_t *key, uint16_t dt_ms);
uint8_t app_key_is_pressed(const app_key_t *key);
uint8_t app_key_take_pressed(app_key_t *key);
uint8_t app_key_take_released(app_key_t *key);

#endif

