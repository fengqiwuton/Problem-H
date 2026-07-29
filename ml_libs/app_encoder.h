#ifndef __APP_ENCODER_H__
#define __APP_ENCODER_H__

#include "headfile.h"

typedef struct
{
    GPIOn_enum a_port;
    Pinx_enum a_pin;
    GPIOn_enum b_port;
    Pinx_enum b_pin;
    int32_t count;
    int32_t last_count;
    int16_t speed_cps;
    uint8_t reverse;
    uint8_t last_state;
} app_encoder_t;

void app_encoder_init(app_encoder_t *encoder, GPIOn_enum a_port, Pinx_enum a_pin, GPIOn_enum b_port, Pinx_enum b_pin, uint8_t reverse);
void app_encoder_update(app_encoder_t *encoder);
void app_encoder_speed_update(app_encoder_t *encoder, uint16_t dt_ms);
void app_encoder_reset(app_encoder_t *encoder);
int32_t app_encoder_count(const app_encoder_t *encoder);
int16_t app_encoder_speed_cps(const app_encoder_t *encoder);

#endif

