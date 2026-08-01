#include "headfile.h"
#include "app_board.h"
#include "task3_asym_move.h"

int main(void)
{

    /* Motor */
    uart_init(UART_2, 115200, 1);
    control_speed(0, 0, 0, 0);
    delay_ms(100);
    motor_init();
    control_speed(0, 0, 0, 0);

    /* OLED */
    OLED_Init();
    OLED_Clear();

    OLED_ShowString(1, 1, "T3 Asym Fine");
    task3_asym_move_init();

    while (1)
    {
        task3_asym_move_update(10);
        delay_ms(10);
    }
}
