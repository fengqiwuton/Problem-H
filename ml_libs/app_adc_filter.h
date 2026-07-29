#ifndef __APP_ADC_FILTER_H__
#define __APP_ADC_FILTER_H__

#include "headfile.h"

#define APP_ADC_MAX_CHANNELS 8

typedef struct
{
    ADCx_enum adc;
    ADCINx_enum channels[APP_ADC_MAX_CHANNELS];
    uint16_t raw[APP_ADC_MAX_CHANNELS];
    uint16_t filtered[APP_ADC_MAX_CHANNELS];
    uint8_t count;
    uint8_t alpha_percent;
} app_adc_filter_t;

void app_adc_filter_init(app_adc_filter_t *adc_filter, ADCx_enum adc, const ADCINx_enum *channels, uint8_t count, uint8_t alpha_percent);
void app_adc_filter_update(app_adc_filter_t *adc_filter);
uint16_t app_adc_filter_raw(const app_adc_filter_t *adc_filter, uint8_t index);
uint16_t app_adc_filter_value(const app_adc_filter_t *adc_filter, uint8_t index);

#endif

