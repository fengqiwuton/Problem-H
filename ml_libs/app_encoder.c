#include "app_encoder.h"

static const int8_t encoder_table[16] =
{
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0
};

static uint8_t app_encoder_read_state(const app_encoder_t *encoder)
{
    uint8_t a;
    uint8_t b;

    a = gpio_get(encoder->a_port, encoder->a_pin);
    b = gpio_get(encoder->b_port, encoder->b_pin);
    return (uint8_t)((a << 1) | b);
}

void app_encoder_init(app_encoder_t *encoder, GPIOn_enum a_port, Pinx_enum a_pin, GPIOn_enum b_port, Pinx_enum b_pin, uint8_t reverse)
{
    if (encoder == 0)
    {
        return;
    }

    encoder->a_port = a_port;
    encoder->a_pin = a_pin;
    encoder->b_port = b_port;
    encoder->b_pin = b_pin;
    encoder->count = 0;
    encoder->last_count = 0;
    encoder->speed_cps = 0;
    encoder->reverse = reverse ? 1 : 0;

    gpio_init(a_port, a_pin, IU);
    gpio_init(b_port, b_pin, IU);
    encoder->last_state = app_encoder_read_state(encoder);
}

void app_encoder_update(app_encoder_t *encoder)
{
    uint8_t now_state;
    uint8_t index;
    int8_t step;

    if (encoder == 0)
    {
        return;
    }

    now_state = app_encoder_read_state(encoder);
    index = (uint8_t)((encoder->last_state << 2) | now_state);
    step = encoder_table[index];
    if (encoder->reverse)
    {
        step = -step;
    }
    encoder->count += step;
    encoder->last_state = now_state;
}

void app_encoder_speed_update(app_encoder_t *encoder, uint16_t dt_ms)
{
    int32_t delta;

    if (encoder == 0 || dt_ms == 0)
    {
        return;
    }

    delta = encoder->count - encoder->last_count;
    encoder->speed_cps = (int16_t)(delta * 1000 / dt_ms);
    encoder->last_count = encoder->count;
}

void app_encoder_reset(app_encoder_t *encoder)
{
    if (encoder == 0)
    {
        return;
    }

    encoder->count = 0;
    encoder->last_count = 0;
    encoder->speed_cps = 0;
}

int32_t app_encoder_count(const app_encoder_t *encoder)
{
    return encoder == 0 ? 0 : encoder->count;
}

int16_t app_encoder_speed_cps(const app_encoder_t *encoder)
{
    return encoder == 0 ? 0 : encoder->speed_cps;
}

