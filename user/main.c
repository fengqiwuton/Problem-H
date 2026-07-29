#include "headfile.h"
#include "app_board.h"
#include "track_control.h"

#define LOOP_DT_MS              10
#define OLED_UPDATE_MS          200
#define INIT_STEP_DELAY_MS      300

static uint32_t oled_tick = 0;

/* ── 从 car_example 原版搬过来的 OLED 辅助函数 ── */

static int signed_to_int(float value)
{
    if (value >= 0.0f) return (int)(value + 0.5f);
    return (int)(value - 0.5f);
}

static void show_signed_num(uint8_t row, uint8_t col, int value, uint8_t len)
{
    if (value < 0)
    {
        OLED_ShowString(row, col, "-");
        OLED_ShowNum(row, col + 1, -value, len);
    }
    else
    {
        OLED_ShowString(row, col, " ");
        OLED_ShowNum(row, col + 1, value, len);
    }
}

/* ── 照搬 car_example 原版 motor_stop_on_boot ── */
static void motor_stop_on_boot(void)
{
    uint8_t i;
    uart_init(UART_2, 115200, 1);
    USART2->CR1 &= ~(1 << 5);
    for (i = 0; i < 5; i++)
    {
        control_speed(0, 0, 0, 0);
        delay_ms(20);
    }
}

/* ── 照搬 car_example 原版 oled_show_run，只显示循迹数据 ── */
static void oled_show_run(void)
{
    Track_Info_t info = track_get_info();

    OLED_ShowString(1, 1, "S");
    OLED_ShowNum(1, 2, 0, 1);
    OLED_ShowString(1, 4, "F");
    OLED_ShowNum(1, 5, info.frame_count, 3);
    OLED_ShowString(1, 9, "NF");
    OLED_ShowNum(1, 11, info.no_frame_count, 3);

    OLED_ShowString(2, 1, "Raw");
    OLED_ShowHexNum(2, 5, info.raw, 2);
    OLED_ShowString(2, 9, "Sen");
    OLED_ShowHexNum(2, 13, info.bits, 2);

    OLED_ShowString(3, 1, "E");
    show_signed_num(3, 2, info.error, 2);
    OLED_ShowString(3, 6, "T");
    show_signed_num(3, 7, info.turn, 3);
    OLED_ShowString(3, 12, "L");
    OLED_ShowNum(3, 13, info.lost_count, 3);

    OLED_ShowString(4, 1, "AC");
    OLED_ShowNum(4, 3, info.active_count, 2);
    OLED_ShowString(4, 6, "D");
    OLED_ShowNum(4, 7, info.d_frame_count, 4);
    OLED_ShowString(4, 12, "A");
    OLED_ShowNum(4, 13, info.a_frame_count, 4);
}

int main(void)
{
    motor_stop_on_boot();

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "Track Test");
    OLED_ShowString(2, 1, "Init...");
    delay_ms(500);

    /* 循迹模块初始化（原 car_example 方式） */
    track_control_init();
    delay_ms(INIT_STEP_DELAY_MS);

    /* 电机初始化 */
    motor_init();
    track_car_stop();
    delay_ms(INIT_STEP_DELAY_MS);

    OLED_Clear();

    while (1)
    {
        track_follow_update();

        oled_tick += LOOP_DT_MS;
        if (oled_tick >= OLED_UPDATE_MS)
        {
            oled_tick = 0;
            oled_show_run();
        }

        delay_ms(LOOP_DT_MS);
    }
}
