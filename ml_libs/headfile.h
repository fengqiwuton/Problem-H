#ifndef _headfile_h_
#define _headfile_h_

#include "stm32f10x.h"                  // Device header
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

#include "ml_uart.h"
#include "ml_tim.h"
#include "ml_pwm.h"
#include "ml_oled.h"
#include "ml_delay.h"
#include "ml_gpio.h"
#include "ml_nvic.h"
#include "ml_adc.h"
#include "ml_exti.h"
#include "ml_i2c.h"
#include "ml_mpu6050.h"
#include "ml_hmc5883l.h"

#include "app_adc_filter.h"
#include "app_encoder.h"
#include "app_hcsr04.h"
#include "app_key.h"
#include "app_servo.h"
#include "app_spi_soft.h"
#include "app_uart_frame.h"

#include "motor.h"
#include "pid.h"
#include "gray_track.h"
#include "filter.h"

/* Ball balance competition modules */
#include "camera.h"
#include "ball_balance.h"
#include "app_oled_timer.h"
#endif
