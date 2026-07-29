#include "app_board.h"
#include "app_module_test.h"

void servo_test_run(void)
{
    app_servo_t servo;
    uint8_t index = 0;
    const uint8_t angles[] = {0, 90, 180};

    OLED_Init();
    OLED_Clear();
    app_servo_init(&servo, BOARD_SERVO_TIM, BOARD_SERVO_CH);

    while (1)
    {
        app_servo_set_angle(&servo, (float)angles[index]);
        OLED_ShowString(1, 1, "SERVO TEST");
        OLED_ShowString(2, 1, "ANG:");
        OLED_ShowNum(2, 5, angles[index], 3);
        OLED_ShowString(3, 1, "PWM 50HZ");

        index++;
        if (index >= sizeof(angles))
        {
            index = 0;
        }
        delay_ms(800);
    }
}

