#ifndef __APP_MODULE_TEST_H__
#define __APP_MODULE_TEST_H__

#include "headfile.h"

/*
 * One-file-one-test entry points.
 * Put exactly one xxx_test_run() in main() when testing a module.
 * Each test keeps OLED output short, normally within three lines.
 */
void oled_test_run(void);
void uart_test_run(void);
void adc_filter_test_run(void);
void key_test_run(void);
void servo_test_run(void);
void spi_soft_test_run(void);
void encoder_test_run(void);
void hcsr04_test_run(void);
void motor_test_run(void);

/* Old names kept as wrappers so previous main.c experiments still build. */
void app_test_oled_basic(void);
void app_test_uart_basic(void);
void app_test_adc_oled(void);
void app_test_pwm_servo(void);
void app_test_motor_basic(void);

#endif
