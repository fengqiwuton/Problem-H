#ifndef __APP_OLED_TIMER_H__
#define __APP_OLED_TIMER_H__

#include "headfile.h"

/* ── Timer mode ── */
typedef enum
{
    TIMER_MODE_COUNT_UP = 0,
    TIMER_MODE_COUNT_DOWN
} OledTimer_Mode_t;

/* ── Timer state ── */
typedef enum
{
    TIMER_STATE_IDLE = 0,
    TIMER_STATE_RUNNING,
    TIMER_STATE_PAUSED,
    TIMER_STATE_ALARM
} OledTimer_State_t;

/* ── Main stopwatch struct ── */
typedef struct
{
    OledTimer_State_t state;
    OledTimer_Mode_t  mode;

    /*
     * Timing internals:
     *   When RUNNING:  elapsed_ms = (g_timer_tick_ms - start_tick) + paused_ms
     *   When PAUSED:   elapsed_ms = paused_ms (frozen)
     *   When IDLE:     elapsed_ms = 0
     */
    uint32_t start_tick;        /* g_timer_tick_ms snapshot at last start/resume */
    uint32_t paused_ms;         /* accumulated ms saved at last pause */

    uint32_t target_ms;         /* countdown target (ms) */
    uint32_t elapsed_ms;        /* current display value (ms), updated each tick */

    /* Lap tracking */
    uint32_t lap_prev_tick;     /* g_timer_tick_ms at previous lap (or start) */
    uint32_t lap_best_ms;       /* best lap time (ms), 0xFFFFFFFF = none */
    uint32_t lap_last_ms;       /* last lap time (ms) */
    uint8_t  lap_count;
    uint8_t  lap_show_frames;   /* remaining OLED frames to show lap time */

    uint8_t  force_redraw;      /* non-zero = full OLED repaint needed */
} OledTimer_t;

/* ── Global stopwatch instance ── */
extern OledTimer_t g_stopwatch;

/* ── Public API ── */
void oled_timer_init(OledTimer_t *tmr);
void oled_timer_set_countdown(OledTimer_t *tmr, uint32_t target_ms);

void oled_timer_start(OledTimer_t *tmr);
void oled_timer_pause(OledTimer_t *tmr);
void oled_timer_reset(OledTimer_t *tmr);
void oled_timer_lap(OledTimer_t *tmr);

/* Call every main-loop iteration; refreshes OLED when needed */
void oled_timer_update(OledTimer_t *tmr);

/* Read current elapsed ms without touching OLED (for external display code) */
uint32_t oled_timer_get_elapsed(OledTimer_t *tmr);

/* ── Hardware init: TIM4 1ms tick ── */
void oled_timer_hw_init(void);

#endif /* __APP_OLED_TIMER_H__ */
