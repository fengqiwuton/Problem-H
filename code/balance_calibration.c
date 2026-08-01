#include "balance_calibration.h"
#include "app_board.h"
#include "app_key.h"
#include "openmv_uart.h"
#include "scs_servo.h"

#define BAL_CAL_SERVO_SPEED           250
#define BAL_CAL_CAMERA_TIMEOUT_MS     150
#define BAL_CAL_CENTER_TOL_0P1MM       20
#define BAL_CAL_STABLE_VEL_0P1MM_S     80
#define BAL_CAL_STABLE_TIME_MS       1000

static app_key_t key_up;
static app_key_t key_down;
static uint16_t servo_position = BOARD_BALANCE_SERVO_NEUTRAL;
static uint16_t balance_position = BOARD_BALANCE_SERVO_NEUTRAL;
static int16_t ball_position_0p1mm = 0;
static int16_t ball_velocity_0p1mm_s = 0;
static int16_t last_ball_position_0p1mm = 0;
static uint16_t camera_age_ms = BAL_CAL_CAMERA_TIMEOUT_MS;
static uint16_t frame_elapsed_ms = 0;
static uint16_t stable_elapsed_ms = 0;
static uint16_t display_elapsed_ms = 0;
static uint8_t camera_valid = 0;
static uint8_t have_last_position = 0;
static uint8_t balance_found = 0;

static int balance_abs(int value)
{
    return value < 0 ? -value : value;
}

static int balance_limit(int value, int minimum, int maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void balance_set_servo(uint16_t position)
{
    servo_position = (uint16_t)balance_limit(position,
                                               BOARD_BALANCE_SERVO_SAFE_MIN,
                                               BOARD_BALANCE_SERVO_SAFE_MAX);
    scs_write_pos(BOARD_SCS_SERVO_ID, servo_position, 0, BAL_CAL_SERVO_SPEED);
}

static void balance_poll_camera(uint16_t dt_ms)
{
    openmv_uart_frame_t frame;
    uint8_t received = 0;
    int32_t raw_velocity;

    camera_age_ms += dt_ms;
    frame_elapsed_ms += dt_ms;

    while (openmv_uart_read_ball(&frame))
    {
        received = 1;
    }

    if (received && frame.valid)
    {
        if (frame_elapsed_ms > 250) frame_elapsed_ms = 250;
        if (!have_last_position)
        {
            last_ball_position_0p1mm = frame.position_0p1mm;
            ball_velocity_0p1mm_s = 0;
            have_last_position = 1;
        }
        else
        {
            raw_velocity = ((int32_t)frame.position_0p1mm - last_ball_position_0p1mm) *
                           1000 / frame_elapsed_ms;
            raw_velocity = balance_limit((int)raw_velocity, -3000, 3000);
            ball_velocity_0p1mm_s = (int16_t)((2 * (int32_t)ball_velocity_0p1mm_s + raw_velocity) / 3);
            last_ball_position_0p1mm = frame.position_0p1mm;
        }

        ball_position_0p1mm = frame.position_0p1mm;
        camera_age_ms = 0;
        frame_elapsed_ms = 0;
        camera_valid = 1;
    }
    else if (received)
    {
        camera_valid = 0; /* Explicit OpenMV "$L#" frame. */
    }

    if (camera_age_ms > BAL_CAL_CAMERA_TIMEOUT_MS)
    {
        camera_valid = 0;
    }
}

static void balance_update_found(uint16_t dt_ms)
{
    if (camera_valid &&
        balance_abs(ball_position_0p1mm) <= BAL_CAL_CENTER_TOL_0P1MM &&
        balance_abs(ball_velocity_0p1mm_s) <= BAL_CAL_STABLE_VEL_0P1MM_S)
    {
        stable_elapsed_ms += dt_ms;
        if (stable_elapsed_ms >= BAL_CAL_STABLE_TIME_MS)
        {
            balance_position = servo_position;
            balance_found = 1;
        }
    }
    else
    {
        stable_elapsed_ms = 0;
        balance_found = 0;
    }
}

static void balance_show_oled(void)
{
    display_elapsed_ms += 10;
    if (display_elapsed_ms < 100) return;
    display_elapsed_ms = 0;

    OLED_ShowString(1, 1, "CAL PB1+ PA6-  ");
    OLED_ShowString(2, 1, "R:590-740 S:");
    OLED_ShowNum(2, 13, servo_position, 3);
    /* X0/V0 are 0.1 mm and 0.1 mm/s respectively, so small drift is visible
     * during neutral-position calibration instead of being rounded away. */
    OLED_ShowString(3, 1, "X0:");
    OLED_ShowSignedNum(3, 4, ball_position_0p1mm, 4);
    OLED_ShowString(3, 10, "V0:");
    OLED_ShowSignedNum(3, 13, ball_velocity_0p1mm_s, 3);
    OLED_ShowString(4, 1, "BAL:");
    OLED_ShowNum(4, 5, balance_position, 3);
    OLED_ShowString(4, 10, balance_found ? "OK " : "ADJ");
    OLED_ShowString(4, 14, camera_valid ? "Y" : "N");
}

void balance_calibration_init(void)
{
    scs_init(BOARD_SCS_SERVO_ID);
    scs_torque_enable(BOARD_SCS_SERVO_ID, 1);
    openmv_uart_init();

    app_key_init(&key_up, BOARD_START_KEY_PORT, BOARD_START_KEY_PIN,
                 BOARD_START_KEY_ACTIVE, 30);
    app_key_init(&key_down, BOARD_TASK_KEY_PORT, BOARD_TASK_KEY_PIN,
                 BOARD_TASK_KEY_ACTIVE, 30);

    balance_set_servo(BOARD_BALANCE_SERVO_NEUTRAL);
}

void balance_calibration_update(uint16_t dt_ms)
{
    balance_poll_camera(dt_ms);
    app_key_update(&key_up, dt_ms);
    app_key_update(&key_down, dt_ms);

    if (app_key_take_pressed(&key_up))
    {
        balance_set_servo(servo_position + BOARD_BALANCE_SERVO_CAL_STEP);
        stable_elapsed_ms = 0;
        balance_found = 0;
    }
    if (app_key_take_pressed(&key_down))
    {
        balance_set_servo(servo_position - BOARD_BALANCE_SERVO_CAL_STEP);
        stable_elapsed_ms = 0;
        balance_found = 0;
    }

    balance_update_found(dt_ms);
    balance_show_oled();
}
