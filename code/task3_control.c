#include "task3_control.h"
#include "openmv_uart.h"
#include "scs_servo.h"
#include "app_board.h"

/* All position values in the controller are 0.1 mm. */
#define TASK3_TARGET_PLUS_0P1MM       500
#define TASK3_TARGET_MINUS_0P1MM     (-500)
#define TASK3_ENDPOINT_TOL_0P1MM       80
#define TASK3_STABLE_SPEED_0P1MM_S    300
#define TASK3_STABLE_TIME_MS          100
#define TASK3_FINAL_STABLE_TIME_MS    250
#define TASK3_CAMERA_TIMEOUT_MS       120
#define TASK3_PLUS_TIMEOUT_MS        1900
#define TASK3_TOTAL_TIMEOUT_MS       4800
#define TASK3_SERVO_SPEED             500
#define TASK3_SERVO_SAFE_MIN          BOARD_BALANCE_SERVO_SAFE_MIN
#define TASK3_SERVO_SAFE_MAX          BOARD_BALANCE_SERVO_SAFE_MAX
#define TASK3_SERVO_STEP_PER_UPDATE     6

/* Change from +5 cm to the return phase before momentum carries the ball
 * past the allowed +5 cm band. All values below use the controller's 0.1 mm
 * position unit. */
#define TASK3_REVERSAL_LOOKAHEAD_MS    220
#define TASK3_REVERSAL_MIN_X_0P1MM     350
#define TASK3_REVERSAL_TRIGGER_0P1MM   450
#define TASK3_REVERSAL_MIN_V_0P1MM_S   150

/* Initial gains: Kp = 1.20 servo units/mm, Kd = 0.24 servo units/(mm/s).
 * The smaller proportional action reduces the beam excursion, while the
 * stronger velocity term begins braking the ball before it reaches +5 cm.
 * TASK3_SERVO_SIGN must be changed to -1 if a positive position command makes
 * the ball move away from a positive target. */
#define TASK3_KP_X100                 120
#define TASK3_KD_X100                  24
#define TASK3_SERVO_SIGN                1

static task3_state_t task3_state = TASK3_WAIT_CAMERA;
static int16_t ball_pos_0p1mm = 0;
static int16_t ball_vel_0p1mm_s = 0;
static int16_t task3_target_0p1mm = 0;
static int16_t last_ball_pos_0p1mm = 0;
static uint16_t task3_command = BOARD_BALANCE_SERVO_NEUTRAL;
static uint16_t camera_age_ms = 0;
static uint16_t camera_frame_elapsed_ms = 0;
static uint16_t task_elapsed_ms = 0;
static uint16_t phase_elapsed_ms = 0;
static uint16_t stable_elapsed_ms = 0;
static uint16_t display_elapsed_ms = 0;
static uint8_t last_camera_sequence = 0;
static uint8_t have_camera_sequence = 0;
static uint8_t camera_valid = 0;
static uint8_t uart_frame_count = 0;

static int task3_abs(int value)
{
    return value < 0 ? -value : value;
}

static int task3_limit(int value, int min_value, int max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static void task3_set_servo(uint16_t command)
{
    command = (uint16_t)task3_limit(command, TASK3_SERVO_SAFE_MIN, TASK3_SERVO_SAFE_MAX);

    /* Do not step the water-pipe mechanism from one extreme command to the
     * other in a single 10 ms control period. */
    if (command > task3_command + TASK3_SERVO_STEP_PER_UPDATE)
    {
        task3_command += TASK3_SERVO_STEP_PER_UPDATE;
    }
    else if (task3_command > command + TASK3_SERVO_STEP_PER_UPDATE)
    {
        task3_command -= TASK3_SERVO_STEP_PER_UPDATE;
    }
    else
    {
        task3_command = command;
    }

    scs_write_pos(BOARD_SCS_SERVO_ID, task3_command, 0, TASK3_SERVO_SPEED);
}

static void task3_enter_state(task3_state_t new_state)
{
    task3_state = new_state;
    phase_elapsed_ms = 0;
    stable_elapsed_ms = 0;

    if (new_state == TASK3_GO_PLUS)
    {
        task3_target_0p1mm = TASK3_TARGET_PLUS_0P1MM;
    }
    else if (new_state == TASK3_GO_MINUS || new_state == TASK3_HOLD_MINUS)
    {
        task3_target_0p1mm = TASK3_TARGET_MINUS_0P1MM;
    }
    else
    {
        task3_target_0p1mm = 0;
    }

    if (new_state == TASK3_CAMERA_LOST || new_state == TASK3_TIMEOUT)
    {
        task3_set_servo(BOARD_BALANCE_SERVO_NEUTRAL);
    }
}

static void task3_accept_camera_frame(const openmv_uart_frame_t *frame, uint16_t dt_ms)
{
    int32_t raw_velocity;

    if (!frame->valid)
    {
        camera_valid = 0;
        return;
    }

    if (have_camera_sequence && frame->sequence == last_camera_sequence)
    {
        return;
    }

    if (!have_camera_sequence)
    {
        last_ball_pos_0p1mm = frame->position_0p1mm;
        ball_vel_0p1mm_s = 0;
        have_camera_sequence = 1;
    }
    else
    {
        raw_velocity = ((int32_t)frame->position_0p1mm - last_ball_pos_0p1mm) * 1000 / dt_ms;
        raw_velocity = task3_limit((int)raw_velocity, -4000, 4000);
        ball_vel_0p1mm_s = (int16_t)((3 * (int32_t)ball_vel_0p1mm_s + raw_velocity) / 4);
        last_ball_pos_0p1mm = frame->position_0p1mm;
    }

    ball_pos_0p1mm = frame->position_0p1mm;
    last_camera_sequence = frame->sequence;
    camera_valid = 1;
    camera_age_ms = 0;
    camera_frame_elapsed_ms = 0;
}

static void task3_poll_camera(uint16_t dt_ms)
{
    openmv_uart_frame_t frame;
    uint8_t received = 0;

    camera_age_ms += dt_ms;
    camera_frame_elapsed_ms += dt_ms;

    /* Keep the most recent complete UART frame. This avoids estimating an
     * unrealistic velocity when several queued frames are handled together. */
    while (openmv_uart_read_ball(&frame))
    {
        received = 1;
    }

    if (received)
    {
        if (camera_frame_elapsed_ms > 200)
        {
            camera_frame_elapsed_ms = 200;
        }
        task3_accept_camera_frame(&frame, camera_frame_elapsed_ms);
        if (uart_frame_count < 255)
        {
            uart_frame_count++;
        }
    }

    if (camera_age_ms > TASK3_CAMERA_TIMEOUT_MS)
    {
        camera_valid = 0;
    }
}

static void task3_run_pd(void)
{
    int error;
    int correction;
    int command;

    if (!camera_valid)
    {
        return;
    }

    error = task3_target_0p1mm - ball_pos_0p1mm;
    correction = (TASK3_KP_X100 * error - TASK3_KD_X100 * ball_vel_0p1mm_s) / 1000;
    command = BOARD_BALANCE_SERVO_NEUTRAL + TASK3_SERVO_SIGN * correction;
    task3_set_servo((uint16_t)task3_limit(command, TASK3_SERVO_SAFE_MIN, TASK3_SERVO_SAFE_MAX));
}

static uint8_t task3_is_stable(void)
{
    int error = task3_target_0p1mm - ball_pos_0p1mm;

    return task3_abs(error) <= TASK3_ENDPOINT_TOL_0P1MM &&
           task3_abs(ball_vel_0p1mm_s) <= TASK3_STABLE_SPEED_0P1MM_S;
}

static uint8_t task3_should_reverse_early(void)
{
    int32_t predicted_pos_0p1mm;

    if (ball_vel_0p1mm_s < TASK3_REVERSAL_MIN_V_0P1MM_S ||
        ball_pos_0p1mm < TASK3_REVERSAL_MIN_X_0P1MM)
    {
        return 0;
    }

    predicted_pos_0p1mm = (int32_t)ball_pos_0p1mm +
                            (int32_t)ball_vel_0p1mm_s * TASK3_REVERSAL_LOOKAHEAD_MS / 1000;
    return predicted_pos_0p1mm >= TASK3_REVERSAL_TRIGGER_0P1MM;
}

void task3_init(void)
{
    scs_init(BOARD_SCS_SERVO_ID);
    scs_torque_enable(BOARD_SCS_SERVO_ID, 1);
    openmv_uart_init();
    task3_set_servo(BOARD_BALANCE_SERVO_NEUTRAL);

    task3_state = TASK3_WAIT_CAMERA;
    ball_pos_0p1mm = 0;
    ball_vel_0p1mm_s = 0;
    task3_target_0p1mm = 0;
    last_ball_pos_0p1mm = 0;
    camera_age_ms = TASK3_CAMERA_TIMEOUT_MS;
    camera_frame_elapsed_ms = 0;
    task_elapsed_ms = 0;
    phase_elapsed_ms = 0;
    stable_elapsed_ms = 0;
    display_elapsed_ms = 0;
    have_camera_sequence = 0;
    camera_valid = 0;
    uart_frame_count = 0;
}

void task3_start(void)
{
    if (!camera_valid)
    {
        task3_enter_state(TASK3_WAIT_CAMERA);
        return;
    }

    task_elapsed_ms = 0;
    task3_enter_state(TASK3_GO_PLUS);
}

void task3_update(uint16_t dt_ms)
{
    task3_poll_camera(dt_ms);

    if (task3_state == TASK3_WAIT_CAMERA)
    {
        if (camera_valid)
        {
            task3_enter_state(TASK3_WAIT_START);
        }
        return;
    }

    if (task3_state == TASK3_WAIT_START)
    {
        task3_run_pd();
        return;
    }

    if (task3_state == TASK3_CAMERA_LOST || task3_state == TASK3_TIMEOUT)
    {
        return;
    }

    if (!camera_valid)
    {
        task3_enter_state(TASK3_CAMERA_LOST);
        return;
    }

    task_elapsed_ms += dt_ms;
    phase_elapsed_ms += dt_ms;
    task3_run_pd();

    if (task3_state == TASK3_GO_PLUS)
    {
        if (task3_should_reverse_early())
        {
            /* Start the braking/return command while the ball is still moving
             * toward +5 cm. Its inertial peak remains inside the tolerance. */
            task3_enter_state(TASK3_GO_MINUS);
        }
        else if (task3_is_stable())
        {
            stable_elapsed_ms += dt_ms;
            if (stable_elapsed_ms >= TASK3_STABLE_TIME_MS)
            {
                task3_enter_state(TASK3_GO_MINUS);
            }
        }
        else
        {
            stable_elapsed_ms = 0;
        }

        if (phase_elapsed_ms > TASK3_PLUS_TIMEOUT_MS)
        {
            task3_enter_state(TASK3_TIMEOUT);
        }
    }
    else if (task3_state == TASK3_GO_MINUS)
    {
        if (task3_is_stable())
        {
            stable_elapsed_ms += dt_ms;
            if (stable_elapsed_ms >= TASK3_FINAL_STABLE_TIME_MS)
            {
                task3_enter_state(TASK3_HOLD_MINUS);
            }
        }
        else
        {
            stable_elapsed_ms = 0;
        }
    }

    if (task_elapsed_ms > TASK3_TOTAL_TIMEOUT_MS && task3_state != TASK3_HOLD_MINUS)
    {
        task3_enter_state(TASK3_TIMEOUT);
    }
}

task3_state_t task3_get_state(void)
{
    return task3_state;
}

void task3_show_oled(void)
{
    int error = task3_target_0p1mm - ball_pos_0p1mm;
    char *state_text;

    display_elapsed_ms += 10;
    if (display_elapsed_ms < 100)
    {
        return;
    }
    display_elapsed_ms = 0;

    switch (task3_state)
    {
        case TASK3_WAIT_CAMERA: state_text = "T3 CAM WAIT   "; break;
        case TASK3_WAIT_START:  state_text = "T3 PB1 START  "; break;
        case TASK3_GO_PLUS:     state_text = "T3 GO +50mm   "; break;
        case TASK3_GO_MINUS:    state_text = "T3 GO -50mm   "; break;
        case TASK3_HOLD_MINUS:  state_text = "T3 HOLD -50mm "; break;
        case TASK3_CAMERA_LOST: state_text = "T3 CAM LOST   "; break;
        default:                state_text = "T3 TIMEOUT    "; break;
    }

    OLED_ShowString(1, 1, state_text);
    OLED_ShowString(2, 1, "X:");
    OLED_ShowSignedNum(2, 3, ball_pos_0p1mm / 10, 4);
    OLED_ShowString(2, 8, "E:");
    OLED_ShowSignedNum(2, 10, error / 10, 4);
    OLED_ShowString(3, 1, "V:");
    OLED_ShowSignedNum(3, 3, ball_vel_0p1mm_s / 10, 4);
    OLED_ShowString(3, 8, "S:");
    OLED_ShowNum(3, 10, task3_command, 4);
    OLED_ShowString(4, 1, "UART:");
    OLED_ShowNum(4, 6, uart_frame_count, 3);
    OLED_ShowString(4, 10, camera_valid ? "OK " : "NO ");
}
