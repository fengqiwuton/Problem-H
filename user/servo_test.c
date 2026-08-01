#include "app_board.h"
#include "app_module_test.h"
#include "scs_servo.h"

void servo_test_run(void)
{
    uint8_t index = 0;
    const uint16_t positions[] =
    {
        BOARD_SCS_SERVO_MIN,
        BOARD_SCS_SERVO_CENTER,
        BOARD_SCS_SERVO_MAX
    };

    OLED_Init();
    OLED_Clear();
    scs_init(BOARD_SCS_SERVO_ID);
    scs_torque_enable(BOARD_SCS_SERVO_ID, 1);
    delay_ms(20);

    while (1)
    {
        scs_write_pos(BOARD_SCS_SERVO_ID, positions[index], 0, 300);
        OLED_ShowString(1, 1, "SCS SERVO TEST");
        OLED_ShowString(2, 1, "Pos:");
        OLED_ShowNum(2, 5, positions[index], 4);
        OLED_ShowString(3, 1, "PB10 UART3 1M");

        index++;
        if (index >= (sizeof(positions) / sizeof(positions[0])))
        {
            index = 0;
        }
        delay_ms(1000);
    }
}

