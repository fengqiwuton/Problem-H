#include "task3_openloop.h"
#include "app_board.h"
#include "app_key.h"
#include "openmv_uart.h"
#include "scs_servo.h"

/*
 * Fixed Task 3 route. Tune only this block during open-loop experiments.
 * SERVO_MAX must make the ball roll toward the +5 cm direction. If it makes
 * the ball roll toward -5 cm instead, swap SERVO_MAX and SERVO_MIN.
 *
 * The route is: settle at O -> accelerate to +5 cm -> reverse toward -5 cm
 * -> short opposite braking pulse -> hold the ball near -5 cm.
 */
#define T3_ROUTE_SERVO_MAX        720
#define T3_ROUTE_SERVO_MIN        620
#define T3_ROUTE_SERVO_BRAKE      715
#define T3_ROUTE_SERVO_HOLD       BOARD_BALANCE_SERVO_NEUTRAL

#define T3_ROUTE_SETTLE_MS        300
#define T3_ROUTE_MAX_MS          1000
#define T3_ROUTE_MIN_MS           550
#define T3_ROUTE_BRAKE_MS         100
#define T3_ROUTE_SERVO_SPEED      350
#define T3_ROUTE_SERVO_STEP          5

typedef enum
{
    T3_ROUTE_IDLE = 0,
    T3_ROUTE_SETTLE,
    T3_ROUTE_MAX,
    T3_ROUTE_MIN,
    T3_ROUTE_BRAKE,
    T3_ROUTE_HOLD
} task3_route_state_t;

static app_key_t start_key;
static task3_route_state_t route_state = T3_ROUTE_IDLE;
static uint16_t route_elapsed_ms = 0;
static uint16_t servo_position = BOARD_BALANCE_SERVO_NEUTRAL;
static uint16_t servo_target = BOARD_BALANCE_SERVO_NEUTRAL;
static uint16_t display_elapsed_ms = 0;
static int16_t ball_position_0p1mm = 0;
static uint8_t camera_valid = 0;

static int route_limit(int value, int minimum, int maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void route_set_servo(uint16_t position)
{
    servo_target = (uint16_t)route_limit(position,
                                           BOARD_BALANCE_SERVO_SAFE_MIN,
                                           BOARD_BALANCE_SERVO_SAFE_MAX);
}

/* The linkage changes by only five servo units on each 10 ms update instead
 * of jumping from one route point to the next. */
static void route_update_servo(void)
{
    if (servo_position + T3_ROUTE_SERVO_STEP < servo_target)
    {
        servo_position += T3_ROUTE_SERVO_STEP;
    }
    else if (servo_target + T3_ROUTE_SERVO_STEP < servo_position)
    {
        servo_position -= T3_ROUTE_SERVO_STEP;
    }
    else
    {
        servo_position = servo_target;
    }

    scs_write_pos(BOARD_SCS_SERVO_ID, servo_position, 0, T3_ROUTE_SERVO_SPEED);
}

static void route_enter(task3_route_state_t state)
{
    route_state = state;
    route_elapsed_ms = 0;

    switch (state)
    {
        case T3_ROUTE_SETTLE: route_set_servo(BOARD_BALANCE_SERVO_NEUTRAL); break;
        case T3_ROUTE_MAX:    route_set_servo(T3_ROUTE_SERVO_MAX); break;
        case T3_ROUTE_MIN:    route_set_servo(T3_ROUTE_SERVO_MIN); break;
        case T3_ROUTE_BRAKE:  route_set_servo(T3_ROUTE_SERVO_BRAKE); break;
        case T3_ROUTE_HOLD:   route_set_servo(T3_ROUTE_SERVO_HOLD); break;
        default:              route_set_servo(BOARD_BALANCE_SERVO_NEUTRAL); break;
    }
}

static void route_poll_camera(void)
{
    openmv_uart_frame_t frame;
    uint8_t received = 0;

    while (openmv_uart_read_ball(&frame))
    {
        received = 1;
    }

    if (received)
    {
        camera_valid = frame.valid;
        if (frame.valid)
        {
            ball_position_0p1mm = frame.position_0p1mm;
        }
    }
}

static char *route_state_text(void)
{
    switch (route_state)
    {
        case T3_ROUTE_IDLE:   return "IDLE PB1 START ";
        case T3_ROUTE_SETTLE: return "SETTLE O       ";
        case T3_ROUTE_MAX:    return "MAX TO +5cm   ";
        case T3_ROUTE_MIN:    return "MIN TO -5cm   ";
        case T3_ROUTE_BRAKE:  return "BRAKE -5cm    ";
        default:              return "HOLD -5cm     ";
    }
}

static void route_show_oled(void)
{
    display_elapsed_ms += 10;
    if (display_elapsed_ms < 100) return;
    display_elapsed_ms = 0;

    OLED_ShowString(1, 1, route_state_text());
    OLED_ShowString(2, 1, "S:");
    OLED_ShowNum(2, 3, servo_position, 3);
    OLED_ShowString(2, 8, "T:");
    OLED_ShowNum(2, 10, route_elapsed_ms, 4);
    OLED_ShowString(3, 1, "X:");
    OLED_ShowSignedNum(3, 3, ball_position_0p1mm / 10, 4);
    OLED_ShowString(3, 10, camera_valid ? "CAM OK" : "CAM NO");
    OLED_ShowString(4, 1, "HI:");
    OLED_ShowNum(4, 4, T3_ROUTE_SERVO_MAX, 3);
    OLED_ShowString(4, 8, "LO:");
    OLED_ShowNum(4, 11, T3_ROUTE_SERVO_MIN, 3);
}

void task3_openloop_init(void)
{
    scs_init(BOARD_SCS_SERVO_ID);
    scs_torque_enable(BOARD_SCS_SERVO_ID, 1);
    openmv_uart_init();
    app_key_init(&start_key, BOARD_START_KEY_PORT, BOARD_START_KEY_PIN,
                 BOARD_START_KEY_ACTIVE, 30);
    route_enter(T3_ROUTE_IDLE);
    scs_write_pos(BOARD_SCS_SERVO_ID, servo_position, 0, T3_ROUTE_SERVO_SPEED);
}

void task3_openloop_update(uint16_t dt_ms)
{
    route_poll_camera();
    app_key_update(&start_key, dt_ms);
    if (app_key_take_pressed(&start_key))
    {
        route_enter(T3_ROUTE_SETTLE);
    }

    if (route_state != T3_ROUTE_IDLE && route_state != T3_ROUTE_HOLD)
    {
        route_elapsed_ms += dt_ms;
        if (route_state == T3_ROUTE_SETTLE && route_elapsed_ms >= T3_ROUTE_SETTLE_MS)
        {
            route_enter(T3_ROUTE_MAX);
        }
        else if (route_state == T3_ROUTE_MAX && route_elapsed_ms >= T3_ROUTE_MAX_MS)
        {
            route_enter(T3_ROUTE_MIN);
        }
        else if (route_state == T3_ROUTE_MIN && route_elapsed_ms >= T3_ROUTE_MIN_MS)
        {
            route_enter(T3_ROUTE_BRAKE);
        }
        else if (route_state == T3_ROUTE_BRAKE && route_elapsed_ms >= T3_ROUTE_BRAKE_MS)
        {
            route_enter(T3_ROUTE_HOLD);
        }
    }

    route_update_servo();
    route_show_oled();
}
