#include "app_board.h"
#include "app_module_test.h"

void hcsr04_test_run(void)
{
    app_hcsr04_t sensor;

    OLED_Init();
    OLED_Clear();
    app_hcsr04_init(&sensor, BOARD_HCSR04_TRIG_PORT, BOARD_HCSR04_TRIG_PIN,
                    BOARD_HCSR04_ECHO_PORT, BOARD_HCSR04_ECHO_PIN);

    while (1)
    {
        app_hcsr04_read(&sensor);
        OLED_ShowString(1, 1, "HCSR04 TEST");
        OLED_ShowString(2, 1, "D:");
        OLED_ShowNum(2, 3, app_hcsr04_distance_mm(&sensor), 4);
        OLED_ShowString(2, 8, "mm");
        OLED_ShowString(3, 1, "OK:");
        OLED_ShowNum(3, 4, app_hcsr04_ok(&sensor), 1);
        OLED_ShowString(3, 7, "OBS:");
        OLED_ShowNum(3, 11, app_hcsr04_obstacle(&sensor, 200), 1);
        delay_ms(100);
    }
}
