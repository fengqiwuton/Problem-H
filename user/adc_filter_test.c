#include "app_board.h"
#include "app_module_test.h"

void adc_filter_test_run(void)
{
    app_adc_filter_t adc_filter;
    const ADCINx_enum channels[] = {BOARD_ADC_CH0, BOARD_ADC_CH1};

    OLED_Init();
    OLED_Clear();
    app_adc_filter_init(&adc_filter, BOARD_ADC_UNIT, channels, 2, 20);

    while (1)
    {
        app_adc_filter_update(&adc_filter);
        OLED_ShowString(1, 1, "ADC TEST");
        OLED_ShowString(2, 1, "A0:");
        OLED_ShowNum(2, 4, app_adc_filter_value(&adc_filter, 0), 4);
        OLED_ShowString(3, 1, "A1:");
        OLED_ShowNum(3, 4, app_adc_filter_value(&adc_filter, 1), 4);
        delay_ms(100);
    }
}

