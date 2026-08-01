#include "task3_control.h"
#include "openmv_uart.h"
#include "scs_servo.h"
#include "app_board.h"

#define TASK3_SERVO_SPEED             500U
#define TASK3_SERVO_KEEPALIVE_MS      100U

static const task3_controller_config_t task3_controller_config = {
    BOARD_BALANCE_SERVO_NEUTRAL,
    BOARD_BALANCE_SERVO_SAFE_MIN,
    BOARD_BALANCE_SERVO_SAFE_MAX,
    6,
    120,
    24,
    1
};

static task3_controller_t task3_controller;
static uint16_t task3_last_servo_command;
static uint16_t task3_servo_keepalive_ms;
static uint8_t task3_servo_command_valid;

static char *task3_state_text(task3_state_t state)
{
    switch (state)
    {
        case TASK3_WAIT_CAMERA: return "T3 WAIT         ";
        case TASK3_CENTER_READY: return "T3 CENTER       ";
        case TASK3_GO_PLUS: return "T3 +50          ";
        case TASK3_GO_MINUS: return "T3 -50          ";
        case TASK3_HOLD_MINUS: return "T3 HOLD         ";
        case TASK3_CAMERA_LOST: return "T3 LOST         ";
        default: return "T3 TIME         ";
    }
}

static void task3_write_servo_if_due(
    const task3_controller_output_t *output, uint16_t dt_ms)
{
    if (task3_servo_command_valid == 0 ||
        output->servo_command != task3_last_servo_command)
    {
        scs_write_pos(BOARD_SCS_SERVO_ID, output->servo_command, 0,
                      TASK3_SERVO_SPEED);
        task3_last_servo_command = output->servo_command;
        task3_servo_keepalive_ms = 0;
        task3_servo_command_valid = 1;
    }
    else if ((uint32_t)task3_servo_keepalive_ms + dt_ms >=
             TASK3_SERVO_KEEPALIVE_MS)
    {
        scs_write_pos(BOARD_SCS_SERVO_ID, output->servo_command, 0,
                      TASK3_SERVO_SPEED);
        task3_servo_keepalive_ms = 0;
    }
    else
    {
        task3_servo_keepalive_ms += dt_ms;
    }
}

void task3_init(void)
{
    scs_init(BOARD_SCS_SERVO_ID);
    scs_torque_enable(BOARD_SCS_SERVO_ID, 1);
    openmv_uart_init();
    task3_controller_init(&task3_controller, &task3_controller_config);
    scs_write_pos(BOARD_SCS_SERVO_ID, BOARD_BALANCE_SERVO_NEUTRAL, 0,
                  TASK3_SERVO_SPEED);

    task3_last_servo_command = BOARD_BALANCE_SERVO_NEUTRAL;
    task3_servo_keepalive_ms = 0;
    task3_servo_command_valid = 1;
}

void task3_start(void)
{
    task3_controller_request_start(&task3_controller);
}

void task3_update(uint16_t dt_ms)
{
    task3_controller_input_t input = {0};
    task3_controller_output_t output;
    openmv_uart_frame_t frame;

    while (openmv_uart_read_ball(&frame) != 0)
    {
        input.has_frame = 1;
        input.frame_valid = frame.valid;
        input.position_0p1mm = frame.position_0p1mm;
    }

    task3_controller_update(&task3_controller, &input, dt_ms);
    output = task3_controller_get_output(&task3_controller);
    task3_write_servo_if_due(&output, dt_ms);
}

void task3_show_oled(void)
{
    task3_controller_output_t output = task3_controller_get_output(
        &task3_controller);
    uint16_t seconds = output.task_elapsed_ms / 1000U;
    uint16_t centiseconds = (output.task_elapsed_ms % 1000U) / 10U;

    OLED_ShowString(1, 1, task3_state_text(output.state));
    OLED_ShowString(2, 1, "X:");
    OLED_ShowSignedNum(2, 3, output.position_0p1mm / 10, 4);
    OLED_ShowString(2, 8, "T:");
    OLED_ShowSignedNum(2, 10, output.target_0p1mm / 10, 4);
    OLED_ShowString(3, 1, "V:");
    OLED_ShowSignedNum(3, 3, output.velocity_0p1mm_s / 10, 4);
    OLED_ShowString(3, 8, "S:");
    OLED_ShowNum(3, 10, output.servo_command, 4);
    OLED_ShowString(4, 1, output.camera_valid != 0 ? "C:Y t:" : "C:N t:");
    OLED_ShowNum(4, 7, seconds, 2);
    OLED_ShowString(4, 9, ".");
    OLED_ShowNum(4, 10, centiseconds, 2);
}

void task3_send_debug(void)
{
    task3_controller_output_t output = task3_controller_get_output(
        &task3_controller);

    app_uart_sendf(BOARD_UART_DEBUG,
        "T3,state=%u,t=%u,x=%d,v=%d,target=%d,servo=%u,cam=%u\r\n",
        (unsigned)output.state,
        (unsigned)output.task_elapsed_ms,
        (int)output.position_0p1mm,
        (int)output.velocity_0p1mm_s,
        (int)output.target_0p1mm,
        (unsigned)output.servo_command,
        (unsigned)output.camera_valid);
}

task3_state_t task3_get_state(void)
{
    return task3_controller_get_output(&task3_controller).state;
}

task3_controller_output_t task3_get_output(void)
{
    return task3_controller_get_output(&task3_controller);
}
