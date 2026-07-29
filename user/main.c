#include "headfile.h"
#include "app_board.h"
#include "track_control.h"

#define LOOP_DT_MS              10
#define OLED_UPDATE_MS          200
#define INIT_STEP_DELAY_MS      300

/* Stop line detection */
#define STOP_LINE_ACTIVE_MIN    4       /* ≥7 sensors = stop line */
#define STOP_LINE_CONFIRM_MS    30      /* hold 30ms to confirm */
#define LEAVE_A_CONFIRM_MS     500      /* 离开A点后的死区时间 */

static uint32_t oled_tick = 0;
static app_key_t start_key;

/* ── OLED helpers (car_example 原版) ── */
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

static void show_time(uint8_t row, uint8_t col, uint32_t ms)
{
    uint16_t sec = ms / 1000;
    uint16_t cs = (ms % 1000) / 10;
    OLED_ShowNum(row, col, sec, 2);
    OLED_ShowString(row, col + 2, ".");
    OLED_ShowNum(row, col + 3, cs, 2);
}

/* ── Init ── */
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

/* ── Stop line detector ── */
typedef enum
{
    STOP_WAIT_LEAVE_A = 0,   /* 先离开A点 */
    STOP_ARMED,              /* 已离开，等待回到A */
    STOP_DONE                /* 检测到A，停车 */
} Stop_State_t;

static Stop_State_t stop_state = STOP_WAIT_LEAVE_A;
static uint32_t stop_line_ms = 0;
static uint32_t leave_a_ms = 0;

/* 返回1=检测到停止线 */
static uint8_t stop_line_check(Track_Info_t *info, uint32_t sys_ms)
{
    switch (stop_state)
    {
        case STOP_WAIT_LEAVE_A:
            /* 车在A点起步，先等离开A（传感器降到正常值） */
            if (info->active_count < STOP_LINE_ACTIVE_MIN)
            {
                if (leave_a_ms == 0)
                    leave_a_ms = sys_ms;
                if (sys_ms - leave_a_ms > LEAVE_A_CONFIRM_MS)
                {
                    stop_state = STOP_ARMED;
                    stop_line_ms = 0;
                }
            }
            else
            {
                leave_a_ms = 0;
            }
            break;

        case STOP_ARMED:
            /* 等待检测停止线（≥7个传感器同时亮） */
            if (info->active_count >= STOP_LINE_ACTIVE_MIN)
            {
                if (stop_line_ms == 0)
                    stop_line_ms = sys_ms;
                if (sys_ms - stop_line_ms > STOP_LINE_CONFIRM_MS)
                {
                    stop_state = STOP_DONE;
                    return 1;
                }
            }
            else
            {
                stop_line_ms = 0;
            }
            break;

        case STOP_DONE:
            return 1;
    }
    return 0;
}

/* ── OLED: 任务运行中 ── */
static void oled_show_run(uint32_t elapsed_ms)
{
    Track_Info_t info = track_get_info();

    /* Line1: time */
    OLED_ShowString(1, 1, "T:");
    show_time(1, 3, elapsed_ms);
    OLED_ShowString(1, 9, "Lap1");

    /* Line2: stop state */
    OLED_ShowString(2, 1, "St:");
    if (stop_state == STOP_WAIT_LEAVE_A)
        OLED_ShowString(2, 4, "LeaveA");
    else if (stop_state == STOP_ARMED)
        OLED_ShowString(2, 4, "Armed ");
    else
        OLED_ShowString(2, 4, "DONE  ");

    /* Line3: tracking */
    OLED_ShowString(3, 1, "E");
    show_signed_num(3, 2, info.error, 3);
    OLED_ShowString(3, 7, "AC");
    OLED_ShowNum(3, 9, info.active_count, 1);
    OLED_ShowString(3, 12, "F");
    OLED_ShowNum(3, 13, info.frame_count, 2);

    /* Line4: speed hint */
    OLED_ShowString(4, 1, "NF");
    OLED_ShowNum(4, 3, info.no_frame_count, 2);
    OLED_ShowString(4, 7, "L");
    OLED_ShowNum(4, 8, info.lost_count, 3);
}

/* ── OLED: 结果显示 ── */
static void oled_show_result(uint32_t elapsed_ms)
{
    OLED_ShowString(1, 1, "===FINISH===");
    OLED_ShowString(2, 1, "Time:");
    show_time(2, 6, elapsed_ms);
    if (elapsed_ms <= 20000)
        OLED_ShowString(3, 1, "PASS <=20s");
    else
        OLED_ShowString(3, 1, "OVER 20s!");
    OLED_ShowString(4, 1, "Press RST");
}

int main(void)
{
    uint32_t sys_ms = 0;
    uint32_t start_ms = 0;
    uint32_t elapsed_ms = 0;
    uint8_t running = 0;

    motor_stop_on_boot();

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "Task2: 1 Lap");
    delay_ms(500);

    /* 循迹 */
    track_control_init();
    delay_ms(INIT_STEP_DELAY_MS);

    /* 电机 */
    motor_init();
    track_car_stop();
    delay_ms(INIT_STEP_DELAY_MS);

    /* 等待按键 */
    OLED_Clear();
    OLED_ShowString(1, 1, "Place at A");
    OLED_ShowString(2, 1, "Press PB1");
    OLED_ShowString(3, 1, "to Start");

    app_key_init(&start_key, BOARD_START_KEY_PORT, BOARD_START_KEY_PIN,
                 BOARD_START_KEY_ACTIVE, 20);

    while (!app_key_take_pressed(&start_key))
    {
        track_car_stop();
        track_read_line_error();
        app_key_update(&start_key, LOOP_DT_MS);
        delay_ms(LOOP_DT_MS);
    }

    /* 启动！ */
    OLED_Clear();
    OLED_ShowString(1, 1, "GO!");
    running = 1;
    start_ms = sys_ms;
    stop_state = STOP_WAIT_LEAVE_A;
    leave_a_ms = 0;
    stop_line_ms = 0;

    while (1)
    {
        sys_ms += LOOP_DT_MS;

        if (running)
        {
            elapsed_ms = sys_ms - start_ms;
            track_follow_update();

            Track_Info_t info = track_get_info();

            /* 超时保护 */
            if (elapsed_ms >= 25000)
            {
                track_car_stop();
                running = 0;
            }

            /* 检测停止线 */
            if (stop_line_check(&info, sys_ms))
            {
                track_car_stop();
                running = 0;
            }

            /* OLED */
            oled_tick += LOOP_DT_MS;
            if (oled_tick >= OLED_UPDATE_MS)
            {
                oled_tick = 0;
                oled_show_run(elapsed_ms);
            }
        }
        else
        {
            /* 停车后显示结果 */
            track_car_stop();
            oled_tick += LOOP_DT_MS;
            if (oled_tick >= OLED_UPDATE_MS)
            {
                oled_tick = 0;
                oled_show_result(elapsed_ms);
            }
        }

        delay_ms(LOOP_DT_MS);
    }
}
