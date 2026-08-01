#include "openmv_uart.h"
#include "app_board.h"

#define OPENMV_UART_BIT_US        (1000000UL / BOARD_OPENMV_UART_BAUD)
#define OPENMV_UART_FIRST_BIT_US  (OPENMV_UART_BIT_US + OPENMV_UART_BIT_US / 2)
#define OPENMV_UART_RX_BUF_SIZE   64

static volatile uint8_t rx_buffer[OPENMV_UART_RX_BUF_SIZE];
static volatile uint8_t rx_write_index = 0;
static volatile uint8_t rx_read_index = 0;
static volatile uint8_t rx_busy = 0;
static volatile uint8_t rx_bit_index = 0;
static volatile uint8_t rx_data = 0;

static uint8_t parse_state = 0;
static uint8_t parse_sequence = 0;
static uint8_t parse_has_digit = 0;
static int8_t parse_sign = 1;
static int32_t parse_value = 0;

static void openmv_uart_push_byte(uint8_t data)
{
    uint8_t next_index = (uint8_t)((rx_write_index + 1) % OPENMV_UART_RX_BUF_SIZE);

    if (next_index == rx_read_index)
    {
        rx_read_index = (uint8_t)((rx_read_index + 1) % OPENMV_UART_RX_BUF_SIZE);
    }

    rx_buffer[rx_write_index] = data;
    rx_write_index = next_index;
}

void openmv_uart_init(void)
{
    gpio_init(BOARD_OPENMV_UART_RX_PORT, BOARD_OPENMV_UART_RX_PIN, IU);
    gpio_init(BOARD_OPENMV_UART_TX_PORT, BOARD_OPENMV_UART_TX_PIN, OUT_PP);
    gpio_set(BOARD_OPENMV_UART_TX_PORT, BOARD_OPENMV_UART_TX_PIN, 1);

    /* EXTI4 is connected to PA4 and recognizes the falling start edge. */
    RCC->APB2ENR |= 1 << 0; /* AFIO clock */
    AFIO->EXTICR[1] &= ~0x000F;
    EXTI->IMR |= 1 << 4;
    EXTI->EMR &= ~(1 << 4);
    EXTI->RTSR &= ~(1 << 4);
    EXTI->FTSR |= 1 << 4;
    EXTI->PR = 1 << 4;

    /* TIM4 runs at 1 MHz, so its ARR is directly expressed in microseconds. */
    RCC->APB1ENR |= 1 << 2;
    TIM4->CR1 = 0;
    TIM4->PSC = 72 - 1;
    TIM4->ARR = OPENMV_UART_BIT_US - 1;
    TIM4->CNT = 0;
    TIM4->SR = 0;
    TIM4->DIER &= ~1;

    NVIC_init(1, 10);  /* EXTI4_IRQn */
    NVIC_init(0, 30);  /* TIM4_IRQn */
}

void openmv_uart_exti4_irq(void)
{
    if (!(EXTI->PR & (1 << 4)))
    {
        return;
    }

    EXTI->PR = 1 << 4;
    if (rx_busy || gpio_get(BOARD_OPENMV_UART_RX_PORT, BOARD_OPENMV_UART_RX_PIN))
    {
        return;
    }

    rx_busy = 1;
    rx_bit_index = 0;
    rx_data = 0;
    EXTI->IMR &= ~(1 << 4);

    TIM4->CR1 &= ~1;
    TIM4->CNT = 0;
    TIM4->ARR = OPENMV_UART_FIRST_BIT_US - 1;
    TIM4->SR = 0;
    TIM4->DIER |= 1;
    TIM4->CR1 |= 1;
}

void openmv_uart_tim4_irq(void)
{
    if (!(TIM4->SR & 1))
    {
        return;
    }

    TIM4->SR &= ~1;
    if (rx_bit_index < 8)
    {
        if (gpio_get(BOARD_OPENMV_UART_RX_PORT, BOARD_OPENMV_UART_RX_PIN))
        {
            rx_data |= (uint8_t)(1 << rx_bit_index);
        }
        rx_bit_index++;
        TIM4->ARR = OPENMV_UART_BIT_US - 1;
        TIM4->CNT = 0;
        return;
    }

    /* The ninth sample is the stop bit. Drop incomplete/corrupted bytes. */
    TIM4->CR1 &= ~1;
    TIM4->DIER &= ~1;
    rx_busy = 0;
    EXTI->PR = 1 << 4;
    EXTI->IMR |= 1 << 4;
    if (gpio_get(BOARD_OPENMV_UART_RX_PORT, BOARD_OPENMV_UART_RX_PIN))
    {
        openmv_uart_push_byte(rx_data);
    }
}

uint8_t openmv_uart_read_ball(openmv_uart_frame_t *frame)
{
    uint8_t data;

    while (rx_read_index != rx_write_index)
    {
        data = rx_buffer[rx_read_index];
        rx_read_index = (uint8_t)((rx_read_index + 1) % OPENMV_UART_RX_BUF_SIZE);

        switch (parse_state)
        {
            case 0:
                if (data == '$') parse_state = 1;
                break;
            case 1:
                if (data == 'B')
                {
                    parse_state = 2;
                }
                else if (data == 'L')
                {
                    parse_state = 5;
                }
                else
                {
                    parse_state = 0;
                }
                break;
            case 2:
                if (data == ',')
                {
                    parse_state = 3;
                    parse_sign = 1;
                    parse_value = 0;
                    parse_has_digit = 0;
                }
                else
                {
                    parse_state = 0;
                }
                break;
            case 3:
                if (data == '-')
                {
                    parse_sign = -1;
                    parse_state = 4;
                }
                else if (data == '+')
                {
                    parse_state = 4;
                }
                else if (data >= '0' && data <= '9')
                {
                    parse_value = data - '0';
                    parse_has_digit = 1;
                    parse_state = 4;
                }
                else
                {
                    parse_state = 0;
                }
                break;
            case 4:
                if (data >= '0' && data <= '9')
                {
                    if (parse_value < 32768)
                    {
                        parse_value = parse_value * 10 + data - '0';
                    }
                    parse_has_digit = 1;
                }
                else if (data == '#' && parse_has_digit)
                {
                    parse_value *= parse_sign;
                    /* OpenMV sends whole millimetres. Convert to the 0.1 mm
                     * unit used by Task 3 without overflowing int16_t. */
                    if (parse_value > 3276) parse_value = 3276;
                    if (parse_value < -3276) parse_value = -3276;
                    frame->position_0p1mm = (int16_t)(parse_value * 10);
                    frame->sequence = ++parse_sequence;
                    frame->valid = 1;
                    parse_state = 0;
                    return 1;
                }
                else
                {
                    parse_state = (data == '$') ? 1 : 0;
                }
                break;
            case 5:
                if (data == '#')
                {
                    frame->position_0p1mm = 0;
                    frame->sequence = ++parse_sequence;
                    frame->valid = 0;
                    parse_state = 0;
                    return 1;
                }
                parse_state = (data == '$') ? 1 : 0;
                break;
            default:
                parse_state = 0;
                break;
        }
    }

    return 0;
}
