#include "scs_servo.h"

#define INST_WRITE  0x03
#define SC_ADDR_GOAL_POS   42
#define SC_ADDR_TORQUE_EN  40

static uint8_t g_id = 1;

/* 直接写UART3寄存器发送, 不等标志位 */
static void u3_send(uint8_t b)
{
    while ((USART3->SR & (1 << 7)) == 0);  /* 等TXE */
    USART3->DR = b;
}

static void scs_send_packet(uint8_t id, uint8_t cmd, uint8_t *params, uint8_t param_len)
{
    uint8_t checksum = id + (param_len + 2) + cmd;
    uint8_t i;
    u3_send(0xFF); u3_send(0xFF);
    u3_send(id); u3_send(param_len + 2); u3_send(cmd);
    for (i = 0; i < param_len; i++) { u3_send(params[i]); checksum += params[i]; }
    u3_send(~checksum & 0xFF);
}

void scs_init(uint8_t servo_id)
{
    g_id = servo_id;

    RCC->APB1ENR |= 1 << 18;    /* USART3 clock */
    RCC->APB2ENR |= 1 << 3;     /* GPIOB clock */

    /* PB10 = AF_PP 50MHz */
    GPIOB->CRH &= ~(0xF << 8);
    GPIOB->CRH |= (0xB << 8);

    /* 波特率 */
    USART3->BRR = (2 << 4) | 4;

    /* UE + TE */
    USART3->CR1 = (1 << 13) | (1 << 3);

    /* 等UART稳定 */
    delay_ms(5);
}

void scs_write_pos(uint8_t id, uint16_t pos, uint16_t tm, uint16_t speed)
{
    uint8_t p[7];
    if (pos > 1023) pos = 1023;
    p[0] = SC_ADDR_GOAL_POS;
    p[1] = (pos >> 8) & 0xFF; p[2] = pos & 0xFF;
    p[3] = (tm >> 8) & 0xFF;   p[4] = tm & 0xFF;
    p[5] = (speed >> 8) & 0xFF; p[6] = speed & 0xFF;
    scs_send_packet(id, INST_WRITE, p, 7);
}

void scs_torque_enable(uint8_t id, uint8_t enable)
{
    uint8_t p[2];
    p[0] = SC_ADDR_TORQUE_EN; p[1] = enable ? 1 : 0;
    scs_send_packet(id, INST_WRITE, p, 2);
}
