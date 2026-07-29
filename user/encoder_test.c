#include "app_board.h"
#include "app_module_test.h"

void encoder_test_run(void)
{
    app_encoder_t encoder;
    uint16_t tick = 0;

    OLED_Init();
    OLED_Clear();
    app_encoder_init(&encoder, BOARD_ENCODER_A_PORT, BOARD_ENCODER_A_PIN,
                     BOARD_ENCODER_B_PORT, BOARD_ENCODER_B_PIN, 0);

    while (1)
    {
        app_encoder_update(&encoder);
        tick += 5;
        if (tick >= 100)
        {
            app_encoder_speed_update(&encoder, tick);
            tick = 0;
        }

        OLED_ShowString(1, 1, "ENC TEST");
        OLED_ShowString(2, 1, "CNT:");
        OLED_ShowSignedNum(2, 5, app_encoder_count(&encoder), 5);
        OLED_ShowString(3, 1, "CPS:");
        OLED_ShowSignedNum(3, 5, app_encoder_speed_cps(&encoder), 4);
        delay_ms(5);
    }
}

