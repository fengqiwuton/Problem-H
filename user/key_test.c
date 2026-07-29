#include "app_board.h"
#include "app_module_test.h"

void key_test_run(void)
{
    app_key_t key;
    uint32_t press_count = 0;

    OLED_Init();
    OLED_Clear();
    app_key_init(&key, BOARD_KEY1_PORT, BOARD_KEY1_PIN, BOARD_KEY_ACTIVE_LEVEL, 20);

    while (1)
    {
        app_key_update(&key, 10);
        if (app_key_take_pressed(&key))
        {
            press_count++;
        }

        OLED_ShowString(1, 1, "KEY TEST");
        OLED_ShowString(2, 1, "STA:");
        OLED_ShowString(2, 5, app_key_is_pressed(&key) ? "DOWN" : "UP  ");
        OLED_ShowString(3, 1, "CNT:");
        OLED_ShowNum(3, 5, press_count, 4);
        delay_ms(10);
    }
}

