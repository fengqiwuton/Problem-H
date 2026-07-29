#include "app_adc_filter.h"

static uint8_t app_adc_limit_count(uint8_t count)
{
    return count > APP_ADC_MAX_CHANNELS ? APP_ADC_MAX_CHANNELS : count;
}

void app_adc_filter_init(app_adc_filter_t *adc_filter, ADCx_enum adc, const ADCINx_enum *channels, uint8_t count, uint8_t alpha_percent)
{
    uint8_t i;

    if (adc_filter == 0 || channels == 0)
    {
        return;
    }

    adc_filter->adc = adc;
    adc_filter->count = app_adc_limit_count(count);
    adc_filter->alpha_percent = alpha_percent > 100 ? 100 : alpha_percent;

    for (i = 0; i < adc_filter->count; i++)
    {
        adc_filter->channels[i] = channels[i];
        adc_init(adc, channels[i]);
        adc_filter->raw[i] = adc_get(adc, channels[i]);
        adc_filter->filtered[i] = adc_filter->raw[i];
    }
}

void app_adc_filter_update(app_adc_filter_t *adc_filter)
{
    uint8_t i;
    uint16_t value;
    uint32_t filtered;

    if (adc_filter == 0)
    {
        return;
    }

    for (i = 0; i < adc_filter->count; i++)
    {
        value = adc_get(adc_filter->adc, adc_filter->channels[i]);
        adc_filter->raw[i] = value;
        filtered = (uint32_t)adc_filter->filtered[i] * (100 - adc_filter->alpha_percent);
        filtered += (uint32_t)value * adc_filter->alpha_percent;
        adc_filter->filtered[i] = (uint16_t)(filtered / 100);
    }
}

uint16_t app_adc_filter_raw(const app_adc_filter_t *adc_filter, uint8_t index)
{
    if (adc_filter == 0 || index >= adc_filter->count)
    {
        return 0;
    }

    return adc_filter->raw[index];
}

uint16_t app_adc_filter_value(const app_adc_filter_t *adc_filter, uint8_t index)
{
    if (adc_filter == 0 || index >= adc_filter->count)
    {
        return 0;
    }

    return adc_filter->filtered[index];
}

