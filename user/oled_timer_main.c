/**
 * oled_timer_main.c
 *
 * OLED stopwatch / countdown timer test — 2-button version.
 *
 * Hardware:
 *   OLED:  PB8 (SCL), PB9 (SDA)
 *   Key1:  PB1  — Start / Pause / Resume
 *   Key2:  PA6  — short: Lap  |  long (>=1.5s): Reset
 */

#include "headfile.h"
#include "app_board.h"
#include "app_oled_timer.h"
#include "app_module_test.h"

#define LOOP_DT_MS       20
#define LONG_PRESS_MS    1500    /* 1.5s = reset */

static app_key_t key_start;       /* PB1 */
static app_key_t key_func;        /* PA6 */
static uint32_t   key_func_hold_ms = 0;       /* accumulates while pressed */
static uint32_t   key_func_hold_saved = 0;    /* snapshot on release */

static void keys_init(void)
{
    app_key_init(&key_start, BOARD_START_KEY_PORT, BOARD_START_KEY_PIN,
                 BOARD_START_KEY_ACTIVE, 20);
    app_key_init(&key_func, BOARD_TASK_KEY_PORT, BOARD_TASK_KEY_PIN,
                 BOARD_TASK_KEY_ACTIVE, 20);
    key_func_hold_ms   = 0;
    key_func_hold_saved = 0;
}

static void keys_update(void)
{
    app_key_update(&key_start, LOOP_DT_MS);
    app_key_update(&key_func,  LOOP_DT_MS);

    if (app_key_is_pressed(&key_func))
    {
        key_func_hold_ms += LOOP_DT_MS;
    }
    else if (key_func_hold_ms > 0)
    {
        /* Just released: snapshot the hold time before clearing */
        key_func_hold_saved = key_func_hold_ms;
        key_func_hold_ms = 0;
    }
}

static void process_keys(void)
{
    OledTimer_t *tmr = &g_stopwatch;

    /* ── PB1: Start / Pause / Resume ── */
    if (app_key_take_pressed(&key_start))
    {
        switch (tmr->state)
        {
            case TIMER_STATE_IDLE:
            case TIMER_STATE_ALARM:
                oled_timer_start(tmr);
                break;
            case TIMER_STATE_RUNNING:
                oled_timer_pause(tmr);
                break;
            case TIMER_STATE_PAUSED:
                oled_timer_start(tmr);   /* resume */
                break;
            default:
                break;
        }
    }

    /* ── PA6: short = Lap, long (>=1.5s) = Reset ── */
    if (app_key_take_released(&key_func))
    {
        if (key_func_hold_saved >= LONG_PRESS_MS)
        {
            oled_timer_reset(tmr);
        }
        else
        {
            if (tmr->state == TIMER_STATE_RUNNING)
                oled_timer_lap(tmr);
        }
        key_func_hold_saved = 0;
    }
}

/* ═══════════════════════════════════════════════════════════════════ */

void oled_timer_test_run(void)
{
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "OLED Timer Test");
    OLED_ShowString(2, 1, "2-Key Version");
    OLED_ShowString(3, 1, "PB1:Start  PA6:Lap");
    OLED_ShowString(4, 1, "Hold PA6:Reset");
    delay_ms(800);

    oled_timer_hw_init();
    keys_init();
    oled_timer_init(&g_stopwatch);
    OLED_Clear();

    while (1)
    {
        keys_update();
        process_keys();
        oled_timer_update(&g_stopwatch);
        delay_ms(LOOP_DT_MS);
    }
}
