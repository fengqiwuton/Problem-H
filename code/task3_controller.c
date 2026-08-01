#include "task3_controller.h"

#define T3_TARGET_PLUS                 500
#define T3_TARGET_MINUS              -500
#define T3_TARGET_TOL                  100
#define T3_CENTER_TOL                  100
#define T3_CENTER_SPEED_MAX            150
#define T3_FINAL_SPEED_MAX             200
#define T3_CENTER_STABLE_MS             300
#define T3_FINAL_STABLE_MS              300
#define T3_CAMERA_TIMEOUT_MS            150
#define T3_PLUS_TIMEOUT_MS             2200
#define T3_TOTAL_TIMEOUT_MS            4800
#define T3_POSITION_LIMIT              1250
#define T3_VELOCITY_LIMIT              5000
#define T3_JUMP_BASE                    120
#define T3_JUMP_PER_MS                    5

static int32_t t3_abs_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

static int32_t t3_clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static uint16_t t3_add_u16_sat(uint16_t value, uint16_t increment)
{
    uint32_t sum = (uint32_t)value + (uint32_t)increment;

    return sum > UINT16_MAX ? UINT16_MAX : (uint16_t)sum;
}

static void t3_clear_estimator(task3_controller_t *controller)
{
    controller->output.position_0p1mm = 0;
    controller->output.velocity_0p1mm_s = 0;
    controller->output.camera_age_ms = 0;
    controller->output.camera_valid = 0;
    controller->raw_position_0p1mm = 0;
    controller->previous_position_0p1mm = 0;
    controller->frame_elapsed_ms = 0;
    controller->center_stable_ms = 0;
    controller->final_stable_ms = 0;
    controller->have_position = 0;
}

static void t3_enter_state(task3_controller_t *controller, task3_state_t state)
{
    controller->output.state = state;
    controller->phase_elapsed_ms = 0;
    controller->center_stable_ms = 0;
    controller->final_stable_ms = 0;

    if (state == TASK3_GO_PLUS)
    {
        controller->output.target_0p1mm = T3_TARGET_PLUS;
        controller->output.start_pending = 0;
        controller->output.plus_reached = 0;
        controller->output.completed = 0;
    }
    else if (state == TASK3_GO_MINUS || state == TASK3_HOLD_MINUS)
    {
        controller->output.target_0p1mm = T3_TARGET_MINUS;
    }
    else
    {
        controller->output.target_0p1mm = 0;
    }
}

static void t3_reset_to_wait(task3_controller_t *controller)
{
    t3_clear_estimator(controller);
    controller->output.state = TASK3_WAIT_CAMERA;
    controller->output.target_0p1mm = 0;
    controller->output.center_ready = 0;
    controller->output.start_pending = 0;
    controller->output.plus_reached = 0;
    controller->output.completed = 0;
    controller->output.task_elapsed_ms = 0;
    controller->phase_elapsed_ms = 0;
}

static uint8_t t3_accept_frame(task3_controller_t *controller,
                               const task3_controller_input_t *input)
{
    int32_t new_raw;
    int32_t jump_limit;
    int32_t old_filtered;
    int32_t filtered;
    int32_t raw_velocity;
    uint16_t elapsed;

    if (input->has_frame == 0 || input->frame_valid == 0)
    {
        return 0;
    }

    new_raw = (int32_t)input->position_0p1mm;
    if (t3_abs_i32(new_raw) > T3_POSITION_LIMIT)
    {
        return 0;
    }

    if (controller->have_position != 0)
    {
        jump_limit = T3_JUMP_BASE +
                     T3_JUMP_PER_MS *
                     (controller->frame_elapsed_ms < 250U ?
                      controller->frame_elapsed_ms : 250U);
        if (t3_abs_i32(new_raw - (int32_t)controller->raw_position_0p1mm) >
            jump_limit)
        {
            return 0;
        }
    }

    old_filtered = (int32_t)controller->output.position_0p1mm;
    filtered = controller->have_position != 0 ?
               (old_filtered + new_raw) / 2 : new_raw;

    if (controller->have_position == 0)
    {
        controller->output.velocity_0p1mm_s = 0;
    }
    else
    {
        elapsed = controller->frame_elapsed_ms;
        if (elapsed == 0U)
        {
            elapsed = 1U;
        }
        raw_velocity = (filtered -
                        (int32_t)controller->previous_position_0p1mm) *
                       1000 / (int32_t)elapsed;
        raw_velocity = t3_clamp_i32(raw_velocity,
                                    -T3_VELOCITY_LIMIT,
                                    T3_VELOCITY_LIMIT);
        controller->output.velocity_0p1mm_s = (int16_t)(
            (3 * (int32_t)controller->output.velocity_0p1mm_s +
             raw_velocity) / 4);
    }

    controller->previous_position_0p1mm = (int16_t)filtered;
    controller->output.position_0p1mm = (int16_t)filtered;
    controller->raw_position_0p1mm = input->position_0p1mm;
    controller->have_position = 1;
    controller->output.camera_age_ms = 0;
    controller->output.camera_valid = 1;
    controller->frame_elapsed_ms = 0;
    return 1;
}

static void t3_step_servo(task3_controller_t *controller, int32_t desired)
{
    int32_t current = (int32_t)controller->output.servo_command;
    int32_t step = (int32_t)controller->config.servo_step;

    desired = t3_clamp_i32(desired,
                           (int32_t)controller->config.servo_min,
                           (int32_t)controller->config.servo_max);
    if (desired > current)
    {
        current += desired - current > step ? step : desired - current;
    }
    else if (desired < current)
    {
        current -= current - desired > step ? step : current - desired;
    }
    controller->output.servo_command = (uint16_t)current;
}

static void t3_update_servo(task3_controller_t *controller)
{
    int32_t error;
    int32_t correction;
    int32_t desired;

    if (controller->output.state == TASK3_WAIT_CAMERA ||
        controller->output.state == TASK3_CAMERA_LOST ||
        controller->output.state == TASK3_TIMEOUT ||
        controller->output.camera_valid == 0)
    {
        t3_step_servo(controller, (int32_t)controller->config.servo_neutral);
        return;
    }

    error = (int32_t)controller->output.target_0p1mm -
            (int32_t)controller->output.position_0p1mm;
    correction = ((int32_t)controller->config.kp_x100 * error -
                  (int32_t)controller->config.kd_x100 *
                  (int32_t)controller->output.velocity_0p1mm_s) / 1000;
    desired = (int32_t)controller->config.servo_neutral +
              (int32_t)controller->config.servo_sign * correction;
    t3_step_servo(controller, desired);
}

void task3_controller_init(task3_controller_t *controller,
                           const task3_controller_config_t *config)
{
    *controller = (task3_controller_t){0};
    controller->config = *config;
    controller->output.state = TASK3_WAIT_CAMERA;
    controller->output.servo_command = config->servo_neutral;
}

void task3_controller_request_start(task3_controller_t *controller)
{
    if (controller->output.state == TASK3_CAMERA_LOST ||
        controller->output.state == TASK3_TIMEOUT)
    {
        t3_reset_to_wait(controller);
        return;
    }

    if (controller->output.state == TASK3_GO_PLUS ||
        controller->output.state == TASK3_GO_MINUS ||
        controller->output.state == TASK3_HOLD_MINUS)
    {
        t3_reset_to_wait(controller);
        return;
    }

    controller->output.start_pending = 1;
}

void task3_controller_update(task3_controller_t *controller,
                             const task3_controller_input_t *input,
                             uint16_t dt_ms)
{
    uint8_t accepted;
    uint8_t center_stable;
    uint8_t final_stable;

    controller->output.camera_age_ms = t3_add_u16_sat(
        controller->output.camera_age_ms, dt_ms);
    controller->frame_elapsed_ms = t3_add_u16_sat(
        controller->frame_elapsed_ms, dt_ms);
    accepted = t3_accept_frame(controller, input);

    if (controller->output.camera_age_ms >= T3_CAMERA_TIMEOUT_MS)
    {
        controller->output.camera_valid = 0;
        controller->have_position = 0;
        controller->output.velocity_0p1mm_s = 0;
    }

    if (controller->output.state == TASK3_WAIT_CAMERA)
    {
        if (accepted != 0)
        {
            t3_enter_state(controller, TASK3_CENTER_READY);
        }
        t3_update_servo(controller);
        return;
    }

    if (controller->output.state == TASK3_CAMERA_LOST ||
        controller->output.state == TASK3_TIMEOUT)
    {
        t3_update_servo(controller);
        return;
    }

    if (controller->output.state == TASK3_GO_PLUS ||
        controller->output.state == TASK3_GO_MINUS)
    {
        controller->output.task_elapsed_ms = t3_add_u16_sat(
            controller->output.task_elapsed_ms, dt_ms);
        controller->phase_elapsed_ms = t3_add_u16_sat(
            controller->phase_elapsed_ms, dt_ms);

        if (controller->output.task_elapsed_ms >= T3_TOTAL_TIMEOUT_MS)
        {
            t3_enter_state(controller, TASK3_TIMEOUT);
            t3_update_servo(controller);
            return;
        }
    }

    if (controller->output.camera_valid == 0)
    {
        t3_enter_state(controller, TASK3_CAMERA_LOST);
        t3_update_servo(controller);
        return;
    }

    if (controller->output.state == TASK3_CENTER_READY)
    {
        center_stable = (uint8_t)(
            t3_abs_i32(controller->output.position_0p1mm) <=
            T3_CENTER_TOL &&
            t3_abs_i32(controller->output.velocity_0p1mm_s) <=
            T3_CENTER_SPEED_MAX);
        if (center_stable != 0)
        {
            controller->center_stable_ms = t3_add_u16_sat(
                controller->center_stable_ms, dt_ms);
            if (controller->center_stable_ms >= T3_CENTER_STABLE_MS)
            {
                controller->output.center_ready = 1;
            }
        }
        else
        {
            controller->center_stable_ms = 0;
            controller->output.center_ready = 0;
        }

        if (controller->output.start_pending != 0 &&
            controller->output.center_ready != 0)
        {
            controller->output.task_elapsed_ms = 0;
            t3_enter_state(controller, TASK3_GO_PLUS);
        }
        t3_update_servo(controller);
        return;
    }

    if (controller->output.state == TASK3_GO_PLUS)
    {
        if (controller->phase_elapsed_ms >= T3_PLUS_TIMEOUT_MS)
        {
            t3_enter_state(controller, TASK3_TIMEOUT);
        }
        else if (controller->raw_position_0p1mm >=
                     T3_TARGET_PLUS - T3_TARGET_TOL &&
                 controller->raw_position_0p1mm <=
                     T3_TARGET_PLUS + T3_TARGET_TOL)
        {
            controller->output.plus_reached = 1;
            t3_enter_state(controller, TASK3_GO_MINUS);
        }
    }
    else if (controller->output.state == TASK3_GO_MINUS)
    {
        final_stable = (uint8_t)(
            controller->output.position_0p1mm >=
                T3_TARGET_MINUS - T3_TARGET_TOL &&
            controller->output.position_0p1mm <=
                T3_TARGET_MINUS + T3_TARGET_TOL &&
            t3_abs_i32(controller->output.velocity_0p1mm_s) <=
                T3_FINAL_SPEED_MAX);
        if (final_stable != 0)
        {
            controller->final_stable_ms = t3_add_u16_sat(
                controller->final_stable_ms, dt_ms);
            if (controller->final_stable_ms >= T3_FINAL_STABLE_MS)
            {
                controller->output.completed = 1;
                t3_enter_state(controller, TASK3_HOLD_MINUS);
            }
        }
        else
        {
            controller->final_stable_ms = 0;
        }
    }

    t3_update_servo(controller);
}

task3_controller_output_t task3_controller_get_output(
    const task3_controller_t *controller)
{
    return controller->output;
}
