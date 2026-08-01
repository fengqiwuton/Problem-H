#ifndef __OPENMV_UART_H__
#define __OPENMV_UART_H__

#include "headfile.h"

typedef struct
{
    /* The OpenMV protocol carries whole millimetres; the controller stores
     * the converted value in 0.1 mm. valid is zero for an explicit "$L#". */
    int16_t position_0p1mm;
    uint8_t sequence;
    uint8_t valid;
} openmv_uart_frame_t;

/* PA4 receives OpenMV P4 at 9600 bit/s using EXTI4 + TIM4 sampling.
 * PA5 is reserved as the return line and stays high until a command protocol
 * is needed. STM32 USART3 is deliberately not used here. */
void openmv_uart_init(void);
void openmv_uart_exti4_irq(void);
void openmv_uart_tim4_irq(void);
uint8_t openmv_uart_read_ball(openmv_uart_frame_t *frame);

#endif
