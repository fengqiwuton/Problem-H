#include "scs_servo.h"

#define INST_READ   0x02
#define INST_WRITE  0x03
#define SC_ADDR_GOAL_POS    42
#define SC_ADDR_TORQUE_EN   40
#define SC_ADDR_PRESENT_POS 56

#define SCS_STATUS_MAX_LEN   16
#define SCS_REPLY_TIMEOUT_US 2000

/* SC09 is a single-wire, 1 Mbps SCS/SCServo bus on USART3_TX (PB10). */
static void scs_bus_tx_mode(void)
{
    USART3->CR1 |= USART_CR1_TE | USART_CR1_RE;
}

static void scs_bus_rx_mode(void)
{
    /* In HDSEL mode, clearing TE releases PB10 so the servo can reply. */
    USART3->CR1 &= ~USART_CR1_TE;
    USART3->CR1 |= USART_CR1_RE;
}

static void u3_send(uint8_t b)
{
    while ((USART3->SR & USART_SR_TXE) == 0);
    USART3->DR = b;
}

static void u3_wait_tc(void)
{
    while ((USART3->SR & USART_SR_TC) == 0);
}

static void u3_clear_rx(void)
{
    volatile uint16_t sr;
    volatile uint16_t dr;

    /* Reading SR then DR clears RXNE and any stale overrun/error flag. */
    while (USART3->SR & (USART_SR_RXNE | USART_SR_ORE |
                         USART_SR_NE | USART_SR_FE | USART_SR_PE))
    {
        sr = USART3->SR;
        dr = USART3->DR;
        (void)sr;
        (void)dr;
    }
}

static uint8_t u3_receive(uint8_t *data, uint16_t timeout_us)
{
    while (timeout_us--)
    {
        if (USART3->SR & USART_SR_RXNE)
        {
            *data = (uint8_t)USART3->DR;
            return 1;
        }
        delay_us(1);
    }
    return 0;
}

static void scs_send_packet(uint8_t id, uint8_t cmd,
                            const uint8_t *params, uint8_t param_len)
{
    uint8_t checksum = id + (param_len + 2) + cmd;
    uint8_t i;

    scs_bus_tx_mode();
    u3_send(0xFF); u3_send(0xFF);
    u3_send(id); u3_send(param_len + 2); u3_send(cmd);
    for (i = 0; i < param_len; i++)
    {
        u3_send(params[i]);
        checksum += params[i];
    }
    u3_send(~checksum & 0xFF);
}

void scs_init(uint8_t servo_id)
{
    (void)servo_id;

    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    RCC->APB2ENR |= 1 << 3;     /* GPIOB clock */

    /* PB10 is the one-wire half-duplex SCS data line. */
    GPIOB->CRH &= ~(0xF << 8);
    GPIOB->CRH |= (0xB << 8);   /* AF_PP 50 MHz, required by STM32F1 HDSEL */

    /* PCLK1 = 36 MHz, USARTDIV = 2.25 at 1 Mbps. */
    USART3->BRR = 0x0024;

    /* HDSEL internally joins Tx/Rx on PB10, so PB11 remains unused. */
    USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    USART3->CR2 = 0;
    USART3->CR3 = USART_CR3_HDSEL;

    delay_ms(5);
}

void scs_write_pos(uint8_t id, uint16_t pos, uint16_t tm, uint16_t speed)
{
    uint8_t p[7];
    if (pos > 1023) pos = 1023;
    p[0] = SC_ADDR_GOAL_POS;
    /* Keep the existing byte order because it has been verified on this SCS009. */
    p[1] = (pos >> 8) & 0xFF;   p[2] = pos & 0xFF;
    p[3] = (tm >> 8) & 0xFF;    p[4] = tm & 0xFF;
    p[5] = (speed >> 8) & 0xFF; p[6] = speed & 0xFF;
    scs_send_packet(id, INST_WRITE, p, 7);
}

void scs_torque_enable(uint8_t id, uint8_t enable)
{
    uint8_t p[2];
    p[0] = SC_ADDR_TORQUE_EN;
    p[1] = enable ? 1 : 0;
    scs_send_packet(id, INST_WRITE, p, 2);
}

uint8_t scs_read_present_pos(uint8_t id, uint16_t *position)
{
    uint8_t request[2] = { SC_ADDR_PRESENT_POS, 2 };
    uint8_t data;
    uint8_t state = 0;
    uint8_t reply_id = 0;
    uint8_t reply_len = 0;
    uint8_t byte_index = 0;
    uint8_t status[SCS_STATUS_MAX_LEN];
    uint8_t checksum = 0;
    uint8_t answer_ok = 0;

    if (position == 0) return 0;

    /* Discard loopback bytes from earlier write commands before querying. */
    u3_clear_rx();
    scs_send_packet(id, INST_READ, request, 2);
    u3_wait_tc();

    /* The outgoing query is visible to the receiver on the shared wire. */
    scs_bus_rx_mode();
    u3_clear_rx();

    while (u3_receive(&data, SCS_REPLY_TIMEOUT_US))
    {
        if (state == 0)
        {
            if (data == 0xFF) state = 1;
        }
        else if (state == 1)
        {
            state = (data == 0xFF) ? 2 : 0;
        }
        else if (state == 2)
        {
            reply_id = data;
            checksum = data;
            state = 3;
        }
        else if (state == 3)
        {
            reply_len = data;
            if (reply_len < 2 || reply_len > SCS_STATUS_MAX_LEN)
            {
                state = 0;
            }
            else
            {
                checksum += data;
                byte_index = 0;
                state = 4;
            }
        }
        else
        {
            /* reply_len = status/error byte + data bytes + checksum */
            if (byte_index < reply_len - 1)
            {
                status[byte_index++] = data;
                checksum += data;
            }
            else
            {
                if (reply_id == id && ((uint8_t)(~checksum) == data) &&
                    byte_index >= 3 && status[0] == 0)
                {
                    /* SCS009 uses the SCS big-endian multi-byte convention. */
                    *position = ((uint16_t)status[1] << 8) |
                                (uint16_t)status[2];
                    answer_ok = 1;
                }
                break;
            }
        }
    }

    /* Restore normal command transmission after every query. */
    scs_bus_tx_mode();
    return answer_ok;
}
