#include "track_controller.h"
#include <string.h>

#define TRACK_KP_DEFAULT_X100       75
#define TRACK_KD_DEFAULT_X100       55
#define TRACK_TURN_MAX             140
#define TRACK_TURN_RISE_STEP        18
#define TRACK_TURN_FALL_STEP        24
#define TRACK_D_INPUT_MAX           80
#define TRACK_CURVE_ENTER_ERROR     45
#define TRACK_CURVE_EXIT_ERROR      28
#define TRACK_CURVE_ENTER_D         35
#define TRACK_CURVE_EXIT_D          12
#define TRACK_CURVE_ENTER_FRAMES     2
#define TRACK_CURVE_EXIT_FRAMES      6
#define TRACK_EXIT_HOLD_MS          100

#define TRACK_SPEED_STRAIGHT        210
#define TRACK_SPEED_CURVE_LIGHT     190
#define TRACK_SPEED_CURVE_MEDIUM    180
#define TRACK_SPEED_CURVE_TIGHT     165
#define TRACK_SPEED_CURVE_SHARP     145
#define TRACK_SPEED_DOWN_STEP        30
#define TRACK_SPEED_UP_STEP           8

static const int track_sensor_weight[8] =
    {-160, -105, -55, -18, 18, 55, 105, 160};

static int clamp_int(int value, int lower, int upper)
{
    if (value < lower)
        return lower;
    if (value > upper)
        return upper;
    return value;
}

static int abs_int(int value)
{
    return value < 0 ? -value : value;
}

static uint8_t count_active_sensors(uint8_t bits, int *weight_sum)
{
    uint8_t count = 0;
    uint8_t index;

    *weight_sum = 0;
    for (index = 0; index < 8; ++index)
    {
        if ((bits & ((uint8_t)1U << index)) != 0U)
        {
            *weight_sum += track_sensor_weight[index];
            ++count;
        }
    }
    return count;
}

static int limit_turn_change(int current, int target)
{
    if ((current > 0 && target < 0) || (current < 0 && target > 0))
    {
        if (current > 0)
            return current > TRACK_TURN_FALL_STEP ? current - TRACK_TURN_FALL_STEP : 0;
        return current < -TRACK_TURN_FALL_STEP ? current + TRACK_TURN_FALL_STEP : 0;
    }

    if (target > current)
    {
        int step = target - current;
        int limit = current < 0 && target <= 0 ?
                    TRACK_TURN_FALL_STEP : TRACK_TURN_RISE_STEP;
        return current + (step > limit ? limit : step);
    }

    if (target < current)
    {
        int step = current - target;
        int limit = current <= 0 && target < 0 ?
                    TRACK_TURN_RISE_STEP : TRACK_TURN_FALL_STEP;
        return current - (step > limit ? limit : step);
    }

    return current;
}

static int curve_speed_target(int absolute_error)
{
    if (absolute_error >= 140)
        return TRACK_SPEED_CURVE_SHARP;
    if (absolute_error >= 105)
        return TRACK_SPEED_CURVE_TIGHT;
    if (absolute_error >= 70)
        return TRACK_SPEED_CURVE_MEDIUM;
    return TRACK_SPEED_CURVE_LIGHT;
}

static int schedule_speed(Track_Controller_t *controller)
{
    int target = TRACK_SPEED_STRAIGHT;
    int current = controller->base_speed;

    if (controller->phase == TRACK_PHASE_CURVE)
        target = curve_speed_target(abs_int(controller->error));
    if (controller->exit_hold_ms != 0U && target > TRACK_SPEED_CURVE_LIGHT)
        target = TRACK_SPEED_CURVE_LIGHT;

    if (target < current)
    {
        current -= current - target > TRACK_SPEED_DOWN_STEP ?
                   TRACK_SPEED_DOWN_STEP : current - target;
    }
    else if (target > current)
    {
        current += target - current > TRACK_SPEED_UP_STEP ?
                   TRACK_SPEED_UP_STEP : target - current;
    }

    return current;
}

static void update_curve_phase(Track_Controller_t *controller)
{
    int absolute_error = abs_int(controller->error);
    int absolute_derivative = abs_int(controller->derivative);

    if (controller->phase == TRACK_PHASE_CURVE)
    {
        controller->curve_enter_frames = 0;
        if (absolute_error <= TRACK_CURVE_EXIT_ERROR &&
            absolute_derivative <= TRACK_CURVE_EXIT_D)
        {
            if (controller->curve_exit_frames < TRACK_CURVE_EXIT_FRAMES)
                ++controller->curve_exit_frames;
            if (controller->curve_exit_frames >= TRACK_CURVE_EXIT_FRAMES)
            {
                controller->phase = TRACK_PHASE_STRAIGHT;
                controller->curve_exit_frames = 0;
                controller->exit_hold_ms = TRACK_EXIT_HOLD_MS;
            }
        }
        else
        {
            controller->curve_exit_frames = 0;
        }
    }
    else
    {
        controller->curve_exit_frames = 0;
        if (absolute_error >= TRACK_CURVE_ENTER_ERROR ||
            absolute_derivative >= TRACK_CURVE_ENTER_D)
        {
            if (controller->curve_enter_frames < TRACK_CURVE_ENTER_FRAMES)
                ++controller->curve_enter_frames;
            if (controller->curve_enter_frames >= TRACK_CURVE_ENTER_FRAMES)
            {
                controller->phase = TRACK_PHASE_CURVE;
                controller->curve_enter_frames = 0;
            }
        }
        else
        {
            controller->curve_enter_frames = 0;
        }
    }
}

void track_controller_init(Track_Controller_t *controller)
{
    memset(controller, 0, sizeof(*controller));
    controller->gains.kp_x100 = TRACK_KP_DEFAULT_X100;
    controller->gains.kd_x100 = TRACK_KD_DEFAULT_X100;
    controller->base_speed = TRACK_SPEED_STRAIGHT;
    controller->phase = TRACK_PHASE_STRAIGHT;
}

void track_controller_reset(Track_Controller_t *controller)
{
    Track_Controller_Gains_t gains = controller->gains;

    track_controller_init(controller);
    controller->gains = gains;
}

void track_controller_set_gains(Track_Controller_t *controller, int kp_x100, int kd_x100)
{
    controller->gains.kp_x100 = kp_x100;
    controller->gains.kd_x100 = kd_x100;
}

Track_Controller_Gains_t track_controller_get_gains(const Track_Controller_t *controller)
{
    return controller->gains;
}

Track_Controller_Output_t track_controller_step(Track_Controller_t *controller,
                                                uint8_t bits,
                                                uint16_t dt_ms)
{
    Track_Controller_Output_t output;
    int sum;
    int delta;
    int target_turn;

    output.active_count = count_active_sensors(bits, &sum);
    if (output.active_count != 0U)
        controller->error = sum / output.active_count;

    delta = clamp_int(controller->error - controller->last_error,
                      -TRACK_D_INPUT_MAX, TRACK_D_INPUT_MAX);
    controller->derivative = (2 * controller->derivative + delta) / 3;
    target_turn = (controller->gains.kp_x100 * controller->error +
                   controller->gains.kd_x100 * controller->derivative) / 100;
    target_turn = clamp_int(target_turn, -TRACK_TURN_MAX, TRACK_TURN_MAX);
    controller->turn = limit_turn_change(controller->turn, target_turn);

    if (controller->error > 0)
        controller->last_direction = 1;
    else if (controller->error < 0)
        controller->last_direction = -1;

    update_curve_phase(controller);
    controller->base_speed = schedule_speed(controller);

    memset(&output, 0, sizeof(output));
    output.error = controller->error;
    output.derivative = controller->derivative;
    output.turn = controller->turn;
    output.base_speed = controller->base_speed;
    output.left_speed = output.base_speed + output.turn;
    output.right_speed = output.base_speed - output.turn;
    output.active_count = count_active_sensors(bits, &sum);
    output.phase = controller->phase;
    output.finish_detected = controller->finish_detected;
    output.stop_requested = controller->stop_requested;

    controller->last_error = controller->error;
    if (controller->exit_hold_ms != 0U)
    {
        controller->exit_hold_ms = controller->exit_hold_ms > dt_ms ?
                                   controller->exit_hold_ms - dt_ms : (uint16_t)0;
    }

    return output;
}

int track_controller_brake_speed(int speed)
{
    return -speed;
}
