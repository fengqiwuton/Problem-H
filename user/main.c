#include "headfile.h"
#include "app_board.h"
#include "scs_servo.h"

int main(void)
{
    app_key_t btn;
    uint16_t pos = SCS_POS_CENTER;
    int8_t dir = 1;
    uint8_t paused = 0;

    /* Motor */
    uart_init(UART_2, 115200, 1);
    control_speed(0, 0, 0, 0);
    delay_ms(100);
    motor_init();
    control_speed(0, 0, 0, 0);

    /* OLED */
    OLED_Init();
    OLED_Clear();

    /* Servo → 默认670 */
    OLED_ShowString(1, 1, "Servo Init...");
    scs_init(1);
    scs_torque_enable(1, 1);
    delay_ms(100);
    scs_write_pos(1, SCS_POS_CENTER, 0, 500);
    OLED_ShowString(1, 1, "Center:670   ");

    /* Button */
    app_key_init(&btn, BOARD_START_KEY_PORT, BOARD_START_KEY_PIN,
                 BOARD_START_KEY_ACTIVE, 30);

    while (1)
    {
        app_key_update(&btn, 30);
        if (app_key_take_pressed(&btn))
            paused = !paused;

        if (!paused)
        {
            pos += dir * 2;
            if (pos >= SCS_POS_MAX) { pos = SCS_POS_MAX; dir = -1; }
            if (pos <= SCS_POS_MIN) { pos = SCS_POS_MIN; dir = 1;  }
            scs_write_pos(1, pos, 0, 300);
        }

        OLED_ShowString(2, 1, "Pos:");
        OLED_ShowNum(2, 5, pos, 4);
        OLED_ShowString(2, 10, paused ? "HOLD" : "RUN ");
        OLED_ShowString(3, 1, "Range:453~760");
        OLED_ShowString(4, 1, "PB1: pause");

        delay_ms(30);
    }
}
