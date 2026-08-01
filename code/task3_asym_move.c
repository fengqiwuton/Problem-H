#include "task3_asym_move.h"
#include "app_board.h"
#include "app_key.h"
#include "scs_servo.h"

/*
 * Task 3 - hard-coded, no-vision route:
 *   0 cm -> approximately +5 cm -> approximately -5 cm -> hold level.
 *
 * Each move is made of asymmetric micro-pulses. A strong/long tilt breaks
 * static friction; a weak/short tilt back reduces the pipe angle before the
 * next pulse. With no camera, the pulse count is the distance calibration.
 */
#define T3_PLUS_DIRECTION              1  /* Change to -1 if + phase goes left. */

/* First leg: deliberately short. Increase PLUS_CYCLES only if it turns before
 * the ball gets close to +5 cm; decrease it if it passes +5 cm. */
#define T3_PLUS_PUSH_OFFSET            45
#define T3_PLUS_RELAX_OFFSET           12
#define T3_PLUS_PUSH_MS               220
#define T3_PLUS_RELAX_MS               80
#define T3_PLUS_CYCLES                  1

/* Second leg travels about twice as far (+5 cm -> -5 cm), so it starts with
 * more pulses. Tune MINUS_CYCLES after the first-leg count is confirmed. */
#define T3_MINUS_PUSH_OFFSET          102
#define T3_MINUS_RELAX_OFFSET          25
#define T3_MINUS_PUSH_MS              190
#define T3_MINUS_RELAX_MS              80
#define T3_MINUS_CYCLES                 2

#define T3_PLUS_REST_MS                30
#define T3_MINUS_REST_MS              100
#define T3_SETTLE_MS                  300
#define T3_REVERSAL_DWELL_MS           40
#define T3_BOOT_WAIT_MS              5000
#define T3_SERVO_STEP                   5
#define T3_SERVO_SPEED                250

typedef enum
{
    T3_BOOT_WAIT = 0,
    T3_SETTLE,
    T3_PLUS_PUSH,
    T3_PLUS_RELAX,
    T3_PLUS_REST,
    T3_REVERSAL_DWELL,
    T3_MINUS_PUSH,
    T3_MINUS_RELAX,
    T3_MINUS_REST,
    T3_HOLD_MINUS
} task3_fine_state_t;

static app_key_t start_key;
static task3_fine_state_t fine_state = T3_BOOT_WAIT;
static uint16_t phase_elapsed_ms = 0;
static uint16_t display_elapsed_ms = 0;
static uint16_t servo_position = BOARD_BALANCE_SERVO_NEUTRAL;
static uint16_t servo_target = BOARD_BALANCE_SERVO_NEUTRAL;
static uint8_t plus_cycle_count = 0;
static uint8_t minus_cycle_count = 0;

static int fine_limit(int value, int minimum, int maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void fine_set_target(int position)
{
    servo_target = (uint16_t)fine_limit(position,
                                         BOARD_BALANCE_SERVO_SAFE_MIN,
                                         BOARD_BALANCE_SERVO_SAFE_MAX);
}

static void fine_update_servo(void)
{
    if (servo_position + T3_SERVO_STEP < servo_target)
    {
        servo_position += T3_SERVO_STEP;
    }
    else if (servo_target + T3_SERVO_STEP < servo_position)
    {
        servo_position -= T3_SERVO_STEP;
    }
    else
    {
        servo_position = servo_target;
    }

    scs_write_pos(BOARD_SCS_SERVO_ID, servo_position, 0, T3_SERVO_SPEED);
}

static void fine_enter(task3_fine_state_t state)
{
    fine_state = state;
    phase_elapsed_ms = 0;

    switch (state)
    {
        case T3_PLUS_PUSH:
            fine_set_target(BOARD_BALANCE_SERVO_NEUTRAL +
                            T3_PLUS_DIRECTION * T3_PLUS_PUSH_OFFSET);
            break;
        case T3_PLUS_RELAX:
            fine_set_target(BOARD_BALANCE_SERVO_NEUTRAL -
                            T3_PLUS_DIRECTION * T3_PLUS_RELAX_OFFSET);
            break;
        case T3_MINUS_PUSH:
            fine_set_target(BOARD_BALANCE_SERVO_NEUTRAL -
                            T3_PLUS_DIRECTION * T3_MINUS_PUSH_OFFSET);
            break;
        case T3_MINUS_RELAX:
            fine_set_target(BOARD_BALANCE_SERVO_NEUTRAL +
                            T3_PLUS_DIRECTION * T3_MINUS_RELAX_OFFSET);
            break;
        default:
            fine_set_target(BOARD_BALANCE_SERVO_NEUTRAL);
            break;
    }
}

static void fine_restart_route(void)
{
    plus_cycle_count = 0;
    minus_cycle_count = 0;
    fine_enter(T3_SETTLE);
}

static char *fine_state_text(void)
{
    switch (fine_state)
    {
        case T3_BOOT_WAIT:       return "T3 WAIT 5sec  ";
        case T3_SETTLE:          return "T3 CENTER HOLD ";
        case T3_PLUS_PUSH:
        case T3_PLUS_RELAX:
        case T3_PLUS_REST:       return "T3 GO +5cm    ";
        case T3_REVERSAL_DWELL:  return "T3 TURN BACK   ";
        case T3_MINUS_PUSH:
        case T3_MINUS_RELAX:
        case T3_MINUS_REST:      return "T3 GO -5cm    ";
        default:                 return "T3 HOLD -5cm  ";
    }
}

static void fine_show_oled(void)
{
    display_elapsed_ms += 10;
    if (display_elapsed_ms < 100) return;
    display_elapsed_ms = 0;

    OLED_ShowString(1, 1, fine_state_text());
    OLED_ShowString(2, 1, "C:");
    OLED_ShowNum(2, 3, servo_position, 4);
    OLED_ShowString(2, 9, "B:740");
    OLED_ShowString(3, 1, "P:");
    OLED_ShowNum(3, 3, plus_cycle_count, 2);
    OLED_ShowString(3, 7, "M:");
    OLED_ShowNum(3, 9, minus_cycle_count, 2);
    OLED_ShowString(3, 13, "    ");
    OLED_ShowString(4, 1, "PB1 RESTART     ");
}

void task3_asym_move_init(void)
{
    scs_init(BOARD_SCS_SERVO_ID);
    scs_torque_enable(BOARD_SCS_SERVO_ID, 1);
    app_key_init(&start_key, BOARD_START_KEY_PORT, BOARD_START_KEY_PIN,
                 BOARD_START_KEY_ACTIVE, 30);

    servo_position = BOARD_BALANCE_SERVO_NEUTRAL;
    servo_target = BOARD_BALANCE_SERVO_NEUTRAL;
    fine_enter(T3_BOOT_WAIT);
    scs_write_pos(BOARD_SCS_SERVO_ID, servo_position, 0, T3_SERVO_SPEED);
}

void task3_asym_move_update(uint16_t dt_ms)
{
    app_key_update(&start_key, dt_ms);
    if (app_key_take_pressed(&start_key))
    {
        fine_restart_route();
    }

    phase_elapsed_ms += dt_ms;
    if (fine_state == T3_BOOT_WAIT && phase_elapsed_ms >= T3_BOOT_WAIT_MS)
    {
        fine_restart_route();
    }
    else if (fine_state == T3_SETTLE && phase_elapsed_ms >= T3_SETTLE_MS)
    {
        fine_enter(T3_PLUS_PUSH);
    }
    else if (fine_state == T3_PLUS_PUSH && phase_elapsed_ms >= T3_PLUS_PUSH_MS)
    {
        fine_enter(T3_PLUS_RELAX);
    }
    else if (fine_state == T3_PLUS_RELAX && phase_elapsed_ms >= T3_PLUS_RELAX_MS)
    {
        fine_enter(T3_PLUS_REST);
    }
    else if (fine_state == T3_PLUS_REST && phase_elapsed_ms >= T3_PLUS_REST_MS)
    {
        plus_cycle_count++;
        if (plus_cycle_count >= T3_PLUS_CYCLES)
        {
            fine_enter(T3_REVERSAL_DWELL);
        }
        else
        {
            fine_enter(T3_PLUS_PUSH);
        }
    }
    else if (fine_state == T3_REVERSAL_DWELL &&
             phase_elapsed_ms >= T3_REVERSAL_DWELL_MS)
    {
        fine_enter(T3_MINUS_PUSH);
    }
    else if (fine_state == T3_MINUS_PUSH && phase_elapsed_ms >= T3_MINUS_PUSH_MS)
    {
        fine_enter(T3_MINUS_RELAX);
    }
    else if (fine_state == T3_MINUS_RELAX && phase_elapsed_ms >= T3_MINUS_RELAX_MS)
    {
        fine_enter(T3_MINUS_REST);
    }
    else if (fine_state == T3_MINUS_REST && phase_elapsed_ms >= T3_MINUS_REST_MS)
    {
        minus_cycle_count++;
        if (minus_cycle_count >= T3_MINUS_CYCLES)
        {
            fine_enter(T3_HOLD_MINUS);
        }
        else
        {
            fine_enter(T3_MINUS_PUSH);
        }
    }
    fine_update_servo();
    fine_show_oled();
}
