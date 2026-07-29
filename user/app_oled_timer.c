/**
 * app_oled_timer.c
 *
 * OLED stopwatch / countdown timer for STM32F103.
 *
 * Timing: uses the Cortex-M3 DWT cycle counter (no interrupts needed).
 *   DWT->CYCCNT runs at HCLK (72 MHz), so 1 ms = 72000 cycles.
 *
 * Hardware:
 *   OLED:  PB8 (SCL), PB9 (SDA) — software I2C, 0x78
 */

#include "app_oled_timer.h"
#include "app_board.h"

/*
 * DWT (Data Watchpoint and Trace) registers — not defined in this
 * project's older CMSIS headers, so we define them here manually.
 */
#define DWT_BASE        0xE0001000u
#define DWT_CYCCNT      (*(volatile uint32_t *)(DWT_BASE + 0x04))
#define DWT_CTRL        (*(volatile uint32_t *)(DWT_BASE + 0x00))

/* CoreDebug DEMCR (Debug Exception and Monitor Control Register) */
#define CoreDebug_DEMCR     (*(volatile uint32_t *)(0xE000EDFCu))
#define CoreDebug_TRCENA    (1u << 24)   /* trace enable */
#define DWT_CYCCNTENA       (1u << 0)    /* cycle counter enable */

/* ── Number of CPU cycles per millisecond (HCLK / 1000) ── */
#define CYCLES_PER_MS  72000u

/* ── Global stopwatch ── */
OledTimer_t g_stopwatch;

/* ═══════════════════════════════════════════════════════════════════ */
/* ── DWT-based microsecond-accurate tick ── */
/* ═══════════════════════════════════════════════════════════════════ */

/*
 * Read the DWT cycle counter and convert to milliseconds.
 * The counter is 32-bit and wraps ~every 59.6 seconds at 72 MHz.
 * All elapsed-time calculations use unsigned 32-bit subtraction,
 * so a single wraparound is handled correctly (as long as the
 * measured interval is < ~59.6 seconds).
 */
static uint32_t tick_now(void)
{
    return DWT_CYCCNT / CYCLES_PER_MS;
}

/* ═══════════════════════════════════════════════════════════════════ */
/* ── Internal display helpers ── */
/* ═══════════════════════════════════════════════════════════════════ */

static void disp_time(uint8_t row, uint8_t col, uint32_t ms)
{
    uint16_t min = (uint16_t)(ms / 60000);
    uint16_t sec = (uint16_t)((ms % 60000) / 1000);
    uint16_t cs  = (uint16_t)((ms % 1000) / 10);

    OLED_ShowNum(row, col,     min, 2);
    OLED_ShowString(row, col + 2, ":");
    OLED_ShowNum(row, col + 3, sec, 2);
    OLED_ShowString(row, col + 5, ".");
    OLED_ShowNum(row, col + 6, cs,  2);
}

static void disp_time_compact(uint8_t row, uint8_t col, uint32_t ms)
{
    uint16_t min = (uint16_t)(ms / 60000);
    uint16_t sec = (uint16_t)((ms % 60000) / 1000);
    uint16_t cs  = (uint16_t)((ms % 1000) / 10);

    if (min > 0)
    {
        OLED_ShowNum(row, col, min, 2);
        OLED_ShowString(row, col + 2, ":");
        OLED_ShowNum(row, col + 3, sec, 2);
        OLED_ShowString(row, col + 5, ".");
        OLED_ShowNum(row, col + 6, cs, 2);
    }
    else if (sec >= 10)
    {
        OLED_ShowNum(row, col, sec, 2);
        OLED_ShowString(row, col + 2, ".");
        OLED_ShowNum(row, col + 3, cs, 2);
        OLED_ShowString(row, col + 5, "s  ");
    }
    else
    {
        OLED_ShowNum(row, col, sec, 1);
        OLED_ShowString(row, col + 1, ".");
        OLED_ShowNum(row, col + 2, cs, 2);
        OLED_ShowString(row, col + 4, "s   ");
    }
}

static const char *str_state(OledTimer_State_t s)
{
    if (s == TIMER_STATE_IDLE)    return "IDLE ";
    if (s == TIMER_STATE_RUNNING) return "RUN  ";
    if (s == TIMER_STATE_PAUSED)  return "PAUSE";
    if (s == TIMER_STATE_ALARM)   return "ALARM";
    return "???? ";
}

static const char *str_mode(OledTimer_Mode_t m)
{
    return (m == TIMER_MODE_COUNT_DOWN) ? "COUNTDOWN" : "STOPWATCH";
}

/* ═══════════════════════════════════════════════════════════════════ */
/* ── Public API ── */
/* ═══════════════════════════════════════════════════════════════════ */

void oled_timer_init(OledTimer_t *tmr)
{
    if (!tmr) return;

    tmr->state           = TIMER_STATE_IDLE;
    tmr->mode            = TIMER_MODE_COUNT_UP;
    tmr->start_tick      = 0;
    tmr->paused_ms       = 0;
    tmr->elapsed_ms      = 0;
    tmr->target_ms       = 30000;
    tmr->lap_prev_tick   = 0;
    tmr->lap_best_ms     = 0xFFFFFFFF;
    tmr->lap_last_ms     = 0;
    tmr->lap_count       = 0;
    tmr->lap_show_frames = 0;
    tmr->force_redraw    = 1;
}

void oled_timer_set_countdown(OledTimer_t *tmr, uint32_t target_ms)
{
    if (!tmr) return;
    oled_timer_reset(tmr);
    tmr->mode       = TIMER_MODE_COUNT_DOWN;
    tmr->target_ms  = target_ms;
    tmr->force_redraw = 1;
}

void oled_timer_start(OledTimer_t *tmr)
{
    if (!tmr) return;

    if (tmr->state == TIMER_STATE_IDLE || tmr->state == TIMER_STATE_ALARM)
    {
        tmr->start_tick      = tick_now();
        tmr->paused_ms       = 0;
        tmr->lap_prev_tick   = tmr->start_tick;
        tmr->lap_count       = 0;
        tmr->lap_best_ms     = 0xFFFFFFFF;
        tmr->lap_last_ms     = 0;
        tmr->lap_show_frames = 0;
        tmr->state           = TIMER_STATE_RUNNING;
    }
    else if (tmr->state == TIMER_STATE_PAUSED)
    {
        tmr->start_tick    = tick_now();
        tmr->lap_prev_tick = tmr->start_tick;
        tmr->state         = TIMER_STATE_RUNNING;
    }

    tmr->force_redraw = 1;
}

void oled_timer_pause(OledTimer_t *tmr)
{
    if (!tmr) return;
    if (tmr->state != TIMER_STATE_RUNNING) return;

    tmr->elapsed_ms  = tmr->paused_ms + (tick_now() - tmr->start_tick);
    tmr->paused_ms   = tmr->elapsed_ms;
    tmr->state       = TIMER_STATE_PAUSED;
    tmr->force_redraw = 1;
}

void oled_timer_reset(OledTimer_t *tmr)
{
    if (!tmr) return;

    tmr->state           = TIMER_STATE_IDLE;
    tmr->start_tick      = 0;
    tmr->paused_ms       = 0;
    tmr->elapsed_ms      = 0;
    tmr->lap_prev_tick   = 0;
    tmr->lap_best_ms     = 0xFFFFFFFF;
    tmr->lap_last_ms     = 0;
    tmr->lap_count       = 0;
    tmr->lap_show_frames = 0;
    tmr->force_redraw    = 1;
}

void oled_timer_lap(OledTimer_t *tmr)
{
    uint32_t now_tick;
    uint32_t lap_ms;

    if (!tmr) return;
    if (tmr->state != TIMER_STATE_RUNNING) return;

    now_tick = tick_now();

    if (tmr->lap_count == 0)
        lap_ms = (now_tick - tmr->start_tick) + tmr->paused_ms;
    else
        lap_ms = now_tick - tmr->lap_prev_tick;

    tmr->lap_last_ms  = lap_ms;
    tmr->lap_prev_tick = now_tick;

    if (lap_ms < tmr->lap_best_ms)
        tmr->lap_best_ms = lap_ms;

    tmr->lap_count++;
    tmr->lap_show_frames = 10;
    tmr->force_redraw = 1;
}

/* ═══════════════════════════════════════════════════════════════════ */
/* ── Periodic update ── */
/* ═══════════════════════════════════════════════════════════════════ */

void oled_timer_update(OledTimer_t *tmr)
{
    uint32_t display_ms;

    if (!tmr) return;

    /* Compute current elapsed from DWT tick */
    if (tmr->state == TIMER_STATE_RUNNING)
    {
        tmr->elapsed_ms = (tick_now() - tmr->start_tick) + tmr->paused_ms;
    }

    /* Countdown alarm */
    if (tmr->mode == TIMER_MODE_COUNT_DOWN &&
        tmr->state == TIMER_STATE_RUNNING &&
        tmr->elapsed_ms >= tmr->target_ms)
    {
        tmr->elapsed_ms  = tmr->target_ms;
        tmr->state       = TIMER_STATE_ALARM;
        tmr->force_redraw = 1;
    }

    /* Lap display timeout */
    if (tmr->lap_show_frames > 0)
    {
        tmr->lap_show_frames--;
        if (tmr->lap_show_frames == 0)
            tmr->force_redraw = 1;
    }

    /* Display value */
    if (tmr->mode == TIMER_MODE_COUNT_DOWN && tmr->state != TIMER_STATE_IDLE)
        display_ms = (tmr->elapsed_ms >= tmr->target_ms) ? 0 : (tmr->target_ms - tmr->elapsed_ms);
    else
        display_ms = tmr->elapsed_ms;

    /* OLED redraw */
    if (tmr->force_redraw)
    {
        tmr->force_redraw = 0;

        OLED_ShowString(1, 1, (char *)str_mode(tmr->mode));
        OLED_ShowString(1, 11, (char *)str_state(tmr->state));
        disp_time(2, 3, display_ms);

        if (tmr->lap_show_frames > 0 && tmr->lap_count > 0)
        {
            OLED_ShowString(3, 1, "L");
            OLED_ShowNum(3, 2, tmr->lap_count, 2);
            OLED_ShowString(3, 4, ":");
            disp_time_compact(3, 5, tmr->lap_last_ms);
        }
        else if (tmr->state == TIMER_STATE_IDLE)
        {
            OLED_ShowString(3, 1, "PB1:Start PA6:Lap ");
        }
        else if (tmr->state == TIMER_STATE_ALARM)
        {
            OLED_ShowString(3, 1, "*** TIME'S UP! ***");
        }
        else
        {
            if (tmr->lap_best_ms != 0xFFFFFFFF)
            {
                OLED_ShowString(3, 1, "Best:");
                disp_time_compact(3, 6, tmr->lap_best_ms);
            }
            else
            {
                OLED_ShowString(3, 1, "PA6:Lap  Hold:Reset");
            }
        }

        if (tmr->state == TIMER_STATE_IDLE)
            OLED_ShowString(4, 1, "Hold PA6 1.5s: Reset");
        else if (tmr->state == TIMER_STATE_RUNNING)
            OLED_ShowString(4, 1, "PB1:Pause          ");
        else if (tmr->state == TIMER_STATE_PAUSED)
            OLED_ShowString(4, 1, "PB1:Resume  Hold:Reset");
        else if (tmr->state == TIMER_STATE_ALARM)
            OLED_ShowString(4, 1, "PB1:Restart Hold:Reset");
    }
    else if (tmr->state == TIMER_STATE_RUNNING)
    {
        disp_time(2, 3, display_ms);
    }
}

/* ═══════════════════════════════════════════════════════════════════ */
/* ── Read elapsed without touching OLED ── */
/* ═══════════════════════════════════════════════════════════════════ */

uint32_t oled_timer_get_elapsed(OledTimer_t *tmr)
{
    if (!tmr) return 0;

    if (tmr->state == TIMER_STATE_RUNNING)
        return (tick_now() - tmr->start_tick) + tmr->paused_ms;
    else
        return tmr->elapsed_ms;
}

/* ═══════════════════════════════════════════════════════════════════ */
/* ── Hardware init — enable DWT cycle counter ── */
/* ═══════════════════════════════════════════════════════════════════ */

void oled_timer_hw_init(void)
{
    /*
     * Enable the DWT cycle counter — a free-running 32-bit counter
     * at HCLK (72 MHz).  No interrupts needed, just read DWT_CYCCNT.
     */
    CoreDebug_DEMCR |= CoreDebug_TRCENA;    /* enable trace block */
    DWT_CYCCNT = 0;                         /* reset counter */
    DWT_CTRL   |= DWT_CYCCNTENA;            /* enable cycle counter */
}
