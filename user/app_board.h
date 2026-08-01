#ifndef __APP_BOARD_H__
#define __APP_BOARD_H__

#include "headfile.h"

/*
 * Board-level hardware mapping for Ball-Balance Car.
 * Based on car_example, modified pin assignments.
 */

/* UART devices */
#define BOARD_UART_DEBUG          UART_1
#define BOARD_UART_TRACK          UART_1   /* PA9(TX) PA10(RX) — 循迹模块 */
#define BOARD_UART_MOTOR          UART_2   /* PA2(TX) PA3(RX) — 电机驱动 */

#define BOARD_UART_DEBUG_BAUD     115200
#define BOARD_UART_TRACK_BAUD     115200
#define BOARD_UART_MOTOR_BAUD     115200

/*
 * I2C sensors: software I2C on PB6(SCL), PB7(SDA).
 * PB10/PB11 are reserved for the SCS serial-bus servo.
 */
#define BOARD_I2C_SCL_PORT        GPIO_B
#define BOARD_I2C_SCL_PIN         Pin_6
#define BOARD_I2C_SDA_PORT        GPIO_B
#define BOARD_I2C_SDA_PIN         Pin_7

/* MPU6050 interrupt pin on PA7 */
#define BOARD_MPU_INT_PORT        GPIO_A
#define BOARD_MPU_INT_PIN         Pin_7

/* Start button: PB1, active low */
#define BOARD_START_KEY_PORT      GPIO_B
#define BOARD_START_KEY_PIN       Pin_1
#define BOARD_START_KEY_ACTIVE    0

/* Compatibility defines for test files (original car_example names) */
#define BOARD_KEY1_PORT           GPIO_C
#define BOARD_KEY1_PIN            Pin_13
#define BOARD_KEY_ACTIVE_LEVEL    0

/* Track PD tuning keys (used during development, not in competition) */
#define BOARD_TRACK_KEY_ACTIVE    0
#define BOARD_TRACK_KEY_MODE_PORT GPIO_B
#define BOARD_TRACK_KEY_MODE_PIN  Pin_1
#define BOARD_TRACK_KEY_UP_PORT   GPIO_A
#define BOARD_TRACK_KEY_UP_PIN    Pin_6
#define BOARD_TRACK_KEY_DOWN_PORT GPIO_B
#define BOARD_TRACK_KEY_DOWN_PIN  Pin_12

/* Competition task select keys */
#define BOARD_TASK_KEY_ACTIVE     0
#define BOARD_TASK_KEY_PORT       GPIO_A
#define BOARD_TASK_KEY_PIN        Pin_6

/* Fitec SC09 serial bus servo: PB10 / USART3_TX, 1 Mbps. */
#define BOARD_SCS_SERVO_ID         1
#define BOARD_SCS_SERVO_MIN        453
#define BOARD_SCS_SERVO_CENTER     670
#define BOARD_SCS_SERVO_MAX        760

/* Verified safe range for the water-pipe linkage during balance tests. */
/* Measured mechanical level position of the water-pipe linkage. */
#define BOARD_BALANCE_SERVO_NEUTRAL  740
/* Initial hard-coded trial window around the measured level position. */
#define BOARD_BALANCE_SERVO_SAFE_MIN 635
#define BOARD_BALANCE_SERVO_SAFE_MAX 800
#define BOARD_BALANCE_SERVO_CAL_STEP   2

/*
 * Task 3 OpenMV link. "UART(3)" below is OpenMV's UART3, not STM32 USART3.
 * STM32 USART3/PB10 remains exclusively assigned to the SCS servo.
 * OpenMV P4 (TX) -> STM32 PA4 (software-UART RX)
 * OpenMV P5 (RX) <- STM32 PA5 (software-UART TX, held idle-high for now)
 */
#define BOARD_OPENMV_UART_RX_PORT GPIO_A
#define BOARD_OPENMV_UART_RX_PIN  Pin_4
#define BOARD_OPENMV_UART_TX_PORT GPIO_A
#define BOARD_OPENMV_UART_TX_PIN  Pin_5
#define BOARD_OPENMV_UART_BAUD    9600

/* Servo potentiometer feedback: PA1 (ADC_Channel_1).
 * If servo is 5V powered, add a 10K+20K voltage divider to keep ADC < 3.3V. */
#define BOARD_SERVO_POT_ADC       ADC_1
#define BOARD_SERVO_POT_CH        ADC_Channel_1

/* Calibration: record ADC values at known beam angles, then fill in below.
 * ADC_NEUTRAL = reading when beam is horizontal (ball stays still at center)
 * ADC_PER_DEG  = ADC counts per degree of beam tilt */
#define SERVO_POT_ADC_NEUTRAL     2048
#define SERVO_POT_ADC_PER_DEG     22

/* Optional ADC test channels */
#define BOARD_ADC_UNIT            ADC_1
#define BOARD_ADC_CH0             ADC_Channel_0
#define BOARD_ADC_CH1             ADC_Channel_1

/* Optional software SPI pins */
#define BOARD_SPI_SCK_PORT        GPIO_B
#define BOARD_SPI_SCK_PIN         Pin_13
#define BOARD_SPI_MISO_PORT       GPIO_B
#define BOARD_SPI_MISO_PIN        Pin_14
#define BOARD_SPI_MOSI_PORT       GPIO_B
#define BOARD_SPI_MOSI_PIN        Pin_15
#define BOARD_SPI_CS_PORT         GPIO_B
#define BOARD_SPI_CS_PIN          Pin_12

/* Optional AB encoder test pins */
#define BOARD_ENCODER_A_PORT      GPIO_A
#define BOARD_ENCODER_A_PIN       Pin_0
#define BOARD_ENCODER_B_PORT      GPIO_A
#define BOARD_ENCODER_B_PIN       Pin_1

/* HC-SR04 ultrasonic module. Do not enable it together with Task 3 OpenMV UART. */
#define BOARD_HCSR04_TRIG_PORT    GPIO_A
#define BOARD_HCSR04_TRIG_PIN     Pin_4
#define BOARD_HCSR04_ECHO_PORT    GPIO_A
#define BOARD_HCSR04_ECHO_PIN     Pin_5

#endif
