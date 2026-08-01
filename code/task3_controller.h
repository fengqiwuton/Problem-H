#ifndef __TASK3_CONTROLLER_H__
#define __TASK3_CONTROLLER_H__

#include <stdint.h>

typedef enum
{
    TASK3_WAIT_CAMERA = 0,
    TASK3_CENTER_READY,
    TASK3_GO_PLUS,
    TASK3_GO_MINUS,
    TASK3_HOLD_MINUS,
    TASK3_CAMERA_LOST,
    TASK3_TIMEOUT
} task3_state_t;

typedef struct
{
    uint16_t servo_neutral;
    uint16_t servo_min;
    uint16_t servo_max;
    uint16_t servo_step;
    int16_t kp_x100;
    int16_t kd_x100;
    int8_t servo_sign;
} task3_controller_config_t;

typedef struct
{
    uint8_t has_frame;
    uint8_t frame_valid;
    int16_t position_0p1mm;
} task3_controller_input_t;

typedef struct
{
    task3_state_t state;
    int16_t position_0p1mm;
    int16_t velocity_0p1mm_s;
    int16_t target_0p1mm;
    uint16_t servo_command;
    uint16_t task_elapsed_ms;
    uint16_t camera_age_ms;
    uint8_t camera_valid;
    uint8_t center_ready;
    uint8_t start_pending;
    uint8_t plus_reached;
    uint8_t completed;
} task3_controller_output_t;

typedef struct
{
    task3_controller_config_t config;
    task3_controller_output_t output;
    int16_t raw_position_0p1mm;
    int16_t previous_position_0p1mm;
    uint16_t frame_elapsed_ms;
    uint16_t center_stable_ms;
    uint16_t final_stable_ms;
    uint16_t phase_elapsed_ms;
    uint8_t have_position;
} task3_controller_t;

void task3_controller_init(task3_controller_t *controller,
                           const task3_controller_config_t *config);
void task3_controller_request_start(task3_controller_t *controller);
void task3_controller_update(task3_controller_t *controller,
                             const task3_controller_input_t *input,
                             uint16_t dt_ms);
task3_controller_output_t task3_controller_get_output(
    const task3_controller_t *controller);

#endif
