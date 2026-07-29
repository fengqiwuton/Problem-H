#include "app_module_test.h"

void app_test_oled_basic(void)
{
    oled_test_run();
}

void app_test_uart_basic(void)
{
    uart_test_run();
}

void app_test_adc_oled(void)
{
    adc_filter_test_run();
}

void app_test_pwm_servo(void)
{
    servo_test_run();
}

void app_test_motor_basic(void)
{
    motor_test_run();
}
