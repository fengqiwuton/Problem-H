#include "stm32f10x.h"
#include "headfile.h"

extern void track_uart_rx(uint8_t data);

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & 1)
    {
        TIM2->SR &= ~1;
    }
}

void TIM3_IRQHandler(void)
{
    if (TIM3->SR & 1)
    {
        pid_control();
        TIM3->SR &= ~1;
    }
}

void TIM4_IRQHandler(void)
{
    if (TIM4->SR & 1)
    {
        TIM4->SR &= ~1;
    }
}

void USART1_IRQHandler(void)
{
    uint16_t sr = USART1->SR;
    uint8_t data;

    if (sr & (0x20 | 0x08 | 0x04 | 0x02))
    {
        data = (uint8_t)USART1->DR;
        if (sr & 0x20)
        {
            track_uart_rx(data);
        }
    }
}

void USART2_IRQHandler(void)
{
    uint16_t sr = USART2->SR;
    uint8_t data;

    if (sr & (0x20 | 0x08 | 0x04 | 0x02))
    {
        data = (uint8_t)USART2->DR;
        (void)data;
    }
}

void USART3_IRQHandler(void)
{
    uint16_t sr = USART3->SR;
    uint8_t data;

    if (sr & (0x20 | 0x08 | 0x04 | 0x02))
    {
        data = (uint8_t)USART3->DR;
        if (sr & 0x20)
        {
            track_uart_rx(data);
        }
    }
}

void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & (1 << 0))
    {
        EXTI->PR = 1 << 0;
    }
}

void EXTI1_IRQHandler(void)
{
    if (EXTI->PR & (1 << 1))
    {
        EXTI->PR = 1 << 1;
    }
}

void EXTI2_IRQHandler(void)
{
    if (EXTI->PR & (1 << 2))
    {
        if (gpio_get(GPIO_A, Pin_3))
            Encoder_count1--;
        else
            Encoder_count1++;

        EXTI->PR = 1 << 2;
    }
}

void EXTI3_IRQHandler(void)
{
    if (EXTI->PR & (1 << 3))
    {
        EXTI->PR = 1 << 3;
    }
}

void EXTI4_IRQHandler(void)
{
    if (EXTI->PR & (1 << 4))
    {
        if (gpio_get(GPIO_A, Pin_5))
            Encoder_count2++;
        else
            Encoder_count2--;

        EXTI->PR = 1 << 4;
    }
}

void EXTI9_5_IRQHandler(void)
{
    if (EXTI->PR & (1 << 5))
    {
        EXTI->PR = 1 << 5;
    }

    if (EXTI->PR & (1 << 6))
    {
        EXTI->PR = 1 << 6;
    }

    if (EXTI->PR & (1 << 7))
    {
        MPU6050_GetData();
        HMC5883L_GetData();

        roll_gyro += (float)gx / 16.4f * 0.005f;
        pitch_gyro += (float)gy / 16.4f * 0.005f;
        yaw_gyro += (float)gz / 16.4f * 0.005f;

        roll_acc = atan((float)ay / az) * 57.296f;
        pitch_acc = -atan((float)ax / az) * 57.296f;
        yaw_acc = atan((float)ay / ax) * 57.296f;

        yaw_hmc = atan2((float)hmc_x, (float)hmc_y) * 57.296f;

        roll_Kalman = Kalman_Filter(&KF_Roll, roll_acc, (float)gx / 16.4f);
        pitch_Kalman = Kalman_Filter(&KF_Pitch, pitch_acc, (float)gy / 16.4f);
        yaw_Kalman = Kalman_Filter(&KF_Yaw, yaw_hmc, (float)gz / 16.4f);

        EXTI->PR = 1 << 7;
    }

    if (EXTI->PR & (1 << 8))
    {
        EXTI->PR = 1 << 8;
    }

    if (EXTI->PR & (1 << 9))
    {
        EXTI->PR = 1 << 9;
    }
}
