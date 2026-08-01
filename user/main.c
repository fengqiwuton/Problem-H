#define APP_STANDALONE_TASK3 1

#if APP_STANDALONE_TASK3

#include "headfile.h"
#include "app_board.h"
#include "task3_control.h"

#define LOOP_DT_MS       10
#define DISPLAY_DT_MS   100

#define TASK3_DEMCR_REG       (*(volatile uint32_t *)0xE000EDFCUL)
#define TASK3_DWT_CTRL_REG    (*(volatile uint32_t *)0xE0001000UL)
#define TASK3_DWT_CYCCNT_REG  (*(volatile uint32_t *)0xE0001004UL)
#define TASK3_DEMCR_TRCENA    (1UL << 24)
#define TASK3_DWT_CYCCNTENA   (1UL << 0)

typedef struct
{
    uint32_t last_cycle;
    uint32_t remainder_cycles;
} Task3_Clock_t;

static void task3_dwt_init(Task3_Clock_t *clock)
{
    TASK3_DEMCR_REG |= TASK3_DEMCR_TRCENA;
    TASK3_DWT_CYCCNT_REG = 0U;
    TASK3_DWT_CTRL_REG |= TASK3_DWT_CYCCNTENA;
    clock->last_cycle = 0U;
    clock->remainder_cycles = 0U;
}

static uint16_t task3_elapsed_ms(Task3_Clock_t *clock)
{
    uint32_t cycles_per_ms = SystemCoreClock / 1000U;
    uint32_t current_cycle = TASK3_DWT_CYCCNT_REG;
    uint32_t elapsed_cycles = current_cycle - clock->last_cycle;
    uint32_t total_cycles;
    uint32_t elapsed_ms;

    clock->last_cycle = current_cycle;
    if (cycles_per_ms == 0U)
    {
        return 1U;
    }

    total_cycles = elapsed_cycles + clock->remainder_cycles;
    elapsed_ms = total_cycles / cycles_per_ms;
    clock->remainder_cycles = total_cycles % cycles_per_ms;
    return elapsed_ms > 65535UL ? 65535U : (uint16_t)elapsed_ms;
}

int main(void)
{
    app_key_t start_key;
    Task3_Clock_t clock;
    uint32_t display_ms = 0;
    task3_state_t last_display_state;

    uart_init(BOARD_UART_DEBUG, BOARD_UART_DEBUG_BAUD, 1);
    uart_init(BOARD_UART_MOTOR, BOARD_UART_MOTOR_BAUD, 1);
    control_speed(0, 0, 0, 0);
    delay_ms(50);
    motor_init();
    control_speed(0, 0, 0, 0);

    OLED_Init();
    OLED_Clear();
    app_key_init(&start_key, BOARD_START_KEY_PORT,
                 BOARD_START_KEY_PIN, BOARD_START_KEY_ACTIVE, 20);
    task3_init();
    task3_dwt_init(&clock);
    last_display_state = task3_get_state();

    while (1)
    {
        uint16_t dt_ms = task3_elapsed_ms(&clock);
        task3_state_t state;

        app_key_update(&start_key, dt_ms);
        if (app_key_take_pressed(&start_key)) task3_start();
        task3_update(dt_ms);
        state = task3_get_state();

        if (state != TASK3_GO_PLUS && state != TASK3_GO_MINUS)
        {
            display_ms += dt_ms;
            if (state != last_display_state || display_ms >= DISPLAY_DT_MS)
            {
                display_ms = 0;
                task3_show_oled();
                task3_send_debug();
            }
        }
        else display_ms = 0;
        last_display_state = state;
        delay_ms(LOOP_DT_MS);
    }
}

#else

#include "headfile.h"
#include "app_board.h"
#include "track_control.h"

#define LOOP_DT_MS              10
#define OLED_UPDATE_MS          200
#define OLED_PAGE_MS            1000
#define INIT_STEP_DELAY_MS      300
#define TIME_LIMIT_MS           20000

#define TRACK_DEMCR_REG       (*(volatile uint32_t *)0xE000EDFCUL)
#define TRACK_DWT_CTRL_REG    (*(volatile uint32_t *)0xE0001000UL)
#define TRACK_DWT_CYCCNT_REG  (*(volatile uint32_t *)0xE0001004UL)
#define TRACK_DEMCR_TRCENA    (1UL << 24)
#define TRACK_DWT_CYCCNTENA   (1UL << 0)

typedef struct
{
    uint32_t last_cycle;
    uint32_t remainder_cycles;
} Track_Clock_t;

static uint32_t oled_tick = 0;
static uint32_t oled_page_tick = 0;
static app_key_t btn;
static uint8_t running = 0;
static uint8_t oled_show_diagnostics = 1;
static uint32_t elapsed_ms = 0;

static void track_clock_init(Track_Clock_t *clock)
{
    TRACK_DEMCR_REG |= TRACK_DEMCR_TRCENA;
    TRACK_DWT_CYCCNT_REG = 0U;
    TRACK_DWT_CTRL_REG |= TRACK_DWT_CYCCNTENA;
    clock->last_cycle = 0U;
    clock->remainder_cycles = 0U;
}

static uint16_t track_clock_elapsed_ms(Track_Clock_t *clock)
{
    uint32_t cycles_per_ms = SystemCoreClock / 1000U;
    uint32_t current_cycle = TRACK_DWT_CYCCNT_REG;
    uint32_t elapsed_cycles = current_cycle - clock->last_cycle;
    uint32_t total_cycles;
    uint32_t elapsed;

    clock->last_cycle = current_cycle;
    if (cycles_per_ms == 0U)
        return 0U;

    total_cycles = elapsed_cycles + clock->remainder_cycles;
    elapsed = total_cycles / cycles_per_ms;
    clock->remainder_cycles = total_cycles % cycles_per_ms;
    return elapsed > 65535UL ? 65535U : (uint16_t)elapsed;
}

static void track_wait_cycle(uint32_t cycle_start)
{
    uint32_t cycles_per_ms = SystemCoreClock / 1000U;
    uint32_t target_cycles = cycles_per_ms * LOOP_DT_MS;
    uint32_t elapsed_cycles = TRACK_DWT_CYCCNT_REG - cycle_start;

    if (cycles_per_ms == 0U || elapsed_cycles >= target_cycles)
        return;

    if (target_cycles - elapsed_cycles > cycles_per_ms)
    {
        uint32_t remaining_ms = (target_cycles - elapsed_cycles) / cycles_per_ms;
        if (remaining_ms > 1U)
            delay_ms(remaining_ms - 1U);
    }

    while ((uint32_t)(TRACK_DWT_CYCCNT_REG - cycle_start) < target_cycles)
    {
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

static void show_signed_num(uint8_t row, uint8_t col, int value, uint8_t len)
{
    if (value < 0) { OLED_ShowString(row, col, "-"); OLED_ShowNum(row, col + 1, -value, len); }
    else           { OLED_ShowString(row, col, " "); OLED_ShowNum(row, col + 1, value, len); }
}

static char phase_char(Track_Controller_Phase_t phase)
{
    switch (phase)
    {
        case TRACK_PHASE_CURVE:           return 'C';
        case TRACK_PHASE_RECOVERY_HOLD:   return 'H';
        case TRACK_PHASE_RECOVERY_SEARCH: return 'R';
        case TRACK_PHASE_LOST_STOP:       return 'L';
        default:                          return 'S';
    }
}

static void request_run_stop(void)
{
    if (running)
    {
        running = 0;
        track_car_request_stop();
    }
}

static void oled_show(void)
{
    Track_Info_t info = track_get_info();
    Track_PD_t pd = track_pd_get();
    uint8_t i;

    OLED_ShowString(1, 1, running ? "RUN " : "STOP");
    OLED_ShowChar(1, 6, track_car_is_braking() ? 'B' : phase_char(info.phase));
    OLED_ShowString(1, 8, "T:"); show_time(1, 10, elapsed_ms);

    OLED_ShowString(2, 1, "E"); show_signed_num(2, 2, info.error, 3);
    OLED_ShowString(2, 7, "T"); show_signed_num(2, 8, info.turn, 3);
    OLED_ShowString(2, 13, "AC"); OLED_ShowNum(2, 15, info.active_count, 1);

    for (i = 0; i < 8; i++)
        OLED_ShowChar(3, 1 + i * 2, (info.bits >> (7 - i)) & 0x01 ? '1' : '-');

    OLED_ShowString(4, 1, "                ");
    if (oled_show_diagnostics)
    {
        OLED_ShowString(4, 1, "F"); OLED_ShowNum(4, 2, info.d_frame_count, 5);
        OLED_ShowString(4, 8, "L"); OLED_ShowNum(4, 9, info.lost_ms, 5);
    }
    else
    {
        OLED_ShowString(4, 1, "P"); OLED_ShowNum(4, 2, pd.kp_x100, 3);
        OLED_ShowString(4, 6, "D"); OLED_ShowNum(4, 7, pd.kd_x100, 3);
    }
}

int main(void)
{
    Track_Clock_t clock;

    uart_init(UART_2, 115200, 1);
    control_speed(0, 0, 0, 0);
    delay_ms(50);
    motor_init();
    track_car_stop_immediate();

    OLED_Init(); OLED_Clear();
    track_control_init();
    delay_ms(INIT_STEP_DELAY_MS);

    app_key_init(&btn, BOARD_START_KEY_PORT, BOARD_START_KEY_PIN,
                 BOARD_START_KEY_ACTIVE, 20);
    OLED_Clear();
    track_clock_init(&clock);

    while (1)
    {
        uint32_t cycle_start = TRACK_DWT_CYCCNT_REG;
        uint16_t dt_ms = track_clock_elapsed_ms(&clock);
        uint8_t started = 0U;

        app_key_update(&btn, dt_ms);

        if (running == 0U)
        {
            track_car_stop_update(dt_ms);
        }

        if (app_key_take_pressed(&btn))
        {
            if (running)
            {
                request_run_stop();
            }
            else if (track_car_is_braking() == 0U)
            {
                track_control_start();
                elapsed_ms = 0;
                running = 1;
                started = 1U;
            }
        }

        if (running)
        {
            if ((started == 0U) && (elapsed_ms < TIME_LIMIT_MS))
            {
                if (dt_ms >= TIME_LIMIT_MS - elapsed_ms)
                    elapsed_ms = TIME_LIMIT_MS;
                else
                    elapsed_ms += dt_ms;
            }

            track_follow_update(dt_ms);

            Track_Info_t info = track_get_info();

            /* 停车条件: 超时 或 ≥4灯同时亮 */
            if (info.finish_detected || info.stop_requested || elapsed_ms >= TIME_LIMIT_MS)
                request_run_stop();
        }

        oled_tick += dt_ms;
        if (oled_tick >= OLED_UPDATE_MS)
        {
            oled_tick = 0;
            oled_show();
        }

        oled_page_tick += dt_ms;
        if (oled_page_tick >= OLED_PAGE_MS)
        {
            oled_page_tick = 0;
            oled_show_diagnostics = !oled_show_diagnostics;
        }

        track_wait_cycle(cycle_start);
    }
}

#endif
