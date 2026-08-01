#include "headfile.h"
#include "app_board.h"
#include "track_control.h"

#define LOOP_DT_MS              10
#define OLED_UPDATE_MS          200
#define INIT_STEP_DELAY_MS      300
#define TIME_LIMIT_MS           20000

static uint32_t oled_tick = 0;
static app_key_t btn;
static uint8_t running = 0;
static uint32_t elapsed_ms = 0;

static void show_time(uint8_t row, uint8_t col, uint32_t ms)
{
    uint16_t sec = ms / 1000;
    uint16_t cs = (ms % 1000) / 10;
    OLED_ShowNum(row, col, sec, 2);
    OLED_ShowString(row, col + 2, ".");
    OLED_ShowNum(row, col + 3, cs, 2);
}

static void show_signed_num(uint8_t row, uint8_t col, int value, uint8_t len)
{
    if (value < 0) { OLED_ShowString(row, col, "-"); OLED_ShowNum(row, col + 1, -value, len); }
    else           { OLED_ShowString(row, col, " "); OLED_ShowNum(row, col + 1, value, len); }
}

static void oled_show(void)
{
    Track_Info_t info = track_get_info();
    uint8_t i;

    OLED_ShowString(1, 1, running ? "RUN " : "STOP");
    OLED_ShowString(1, 6, "T:"); show_time(1, 8, elapsed_ms);

    OLED_ShowString(2, 1, "E"); show_signed_num(2, 2, info.error, 3);
    OLED_ShowString(2, 7, "T"); show_signed_num(2, 8, info.turn, 3);
    OLED_ShowString(2, 13, "AC"); OLED_ShowNum(2, 15, info.active_count, 1);

    for (i = 0; i < 8; i++)
        OLED_ShowChar(3, 1 + i * 2, (info.bits >> (7 - i)) & 0x01 ? '1' : '-');

    OLED_ShowString(4, 1, "F"); OLED_ShowNum(4, 2, info.frame_count, 3);
    OLED_ShowString(4, 6, "L"); OLED_ShowNum(4, 7, info.lost_count, 3);
}

int main(void)
{
    uart_init(UART_2, 115200, 1);
    control_speed(0, 0, 0, 0);
    delay_ms(50);
    motor_init();
    track_car_stop();

    OLED_Init(); OLED_Clear();
    track_control_init();
    delay_ms(INIT_STEP_DELAY_MS);

    app_key_init(&btn, BOARD_START_KEY_PORT, BOARD_START_KEY_PIN,
                 BOARD_START_KEY_ACTIVE, 20);
    OLED_Clear();

    while (1)
    {
        app_key_update(&btn, LOOP_DT_MS);
        if (app_key_take_pressed(&btn))
        {
            if (running) { running = 0; track_car_stop(); }
            else         { running = 1; elapsed_ms = 0; }
        }

        if (running)
        {
            elapsed_ms += LOOP_DT_MS;
            track_follow_update();

            Track_Info_t info = track_get_info();

            /* 停车条件: 超时 或 ≥4灯同时亮 */
            if (elapsed_ms >= TIME_LIMIT_MS || info.active_count >= 4)
            {
                track_car_stop();
                running = 0;
            }
        }
        else
        {
            track_car_stop();
        }

        oled_tick += LOOP_DT_MS;
        if (oled_tick >= OLED_UPDATE_MS)
        {
            oled_tick = 0;
            oled_show();
        }

        delay_ms(LOOP_DT_MS);
    }
}
