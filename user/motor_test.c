#include "app_board.h"
#include "app_motor.h"
#include "app_module_test.h"

static void motor_test_show(char *state, uint16_t speed)
{
    OLED_ShowString(1, 1, "MOTOR TEST");
    OLED_ShowString(2, 1, "STA:");
    OLED_ShowString(2, 5, state);
    OLED_ShowString(3, 1, "SPD:");
    OLED_ShowNum(3, 5, speed, 4);
}

void motor_test_run(void)
{
    OLED_Init();
    OLED_Clear();
    app_motor_init();

    while (1)
    {
        motor_test_show("FWD ", 120);
        app_motor_forward(120);
        delay_ms(1000);

        motor_test_show("BACK", 120);
        app_motor_backward(120);
        delay_ms(1000);

        motor_test_show("LEFT", 100);
        app_motor_turn_left(100);
        delay_ms(600);

        motor_test_show("RGHT", 100);
        app_motor_turn_right(100);
        delay_ms(600);

        motor_test_show("STOP", 0);
        app_motor_stop();
        delay_ms(1000);
    }
}

