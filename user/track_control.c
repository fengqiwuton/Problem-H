#include "headfile.h"
#include "track_control.h"
#include <string.h>

#define TRACK_TRIM                 10
#define TRACK_KP_MIN                0
#define TRACK_KP_MAX              300
#define TRACK_KD_MIN                0
#define TRACK_KD_MAX              200
#define TRACK_FRAME_TIMEOUT_MS   1000
#define DRIVE_ACCEL_STEP           35
#define DRIVE_DECEL_STEP           80
#define TRACK_BRAKE_MS             40

#define TRACK_CENTER_BITS        0x18U
#define TRACK_ALIGN_MAX_ACTIVE      4
#define TRACK_UART_BUFFER_SIZE    100

static volatile uint8_t track_pending_bits = 0;
static volatile uint16_t track_d_sequence = 0;
static uint16_t track_consumed_sequence = 0;
static uint16_t track_frame_elapsed_ms = 0;
static Track_Controller_t track_controller;
static Track_Controller_Output_t track_output;

static volatile uint16_t track_frame_count = 0;
static volatile uint16_t track_a_frame_count = 0;
static uint8_t sensor_raw = 0xFFU;
static uint8_t sensor_bits = 0;
static uint8_t sensor_active_count = 0;
static int drive_left_now = 0;
static int drive_right_now = 0;
static uint8_t track_braking = 0;
static uint16_t track_brake_elapsed_ms = 0;
static int8_t track_preferred_dir = 0;

static uint8_t ir_rx_buf[TRACK_UART_BUFFER_SIZE];
static volatile uint16_t ir_data_analog[8] = {0};

static int limit_int(int value, int min_value, int max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

static int abs_int(int value)
{
    return value < 0 ? -value : value;
}

static int8_t normalize_dir(int8_t dir)
{
    if (dir > 0)
        return 1;
    if (dir < 0)
        return -1;
    return 0;
}

static uint16_t add_ms_saturated(uint16_t current, uint16_t dt_ms)
{
    if (dt_ms > (uint16_t)(0xFFFFU - current))
        return 0xFFFFU;
    return (uint16_t)(current + dt_ms);
}

static void increment_saturated(volatile uint16_t *counter)
{
    if (*counter < 0xFFFFU)
        ++(*counter);
}

static int move_toward(int current, int target, int step)
{
    if (target > current)
    {
        if (target - current > step)
            return current + step;
        return target;
    }
    if (target < current)
    {
        if (current - target > step)
            return current - step;
        return target;
    }
    return current;
}

static int ramp_speed(int current, int target)
{
    if (current == 0)
        return move_toward(current, target, DRIVE_ACCEL_STEP);

    if ((current > 0 && target < 0) || (current < 0 && target > 0))
        return move_toward(current, 0, DRIVE_DECEL_STEP);

    if (target == 0 || abs_int(target) < abs_int(current))
        return move_toward(current, target, DRIVE_DECEL_STEP);

    return move_toward(current, target, DRIVE_ACCEL_STEP);
}

static void send_drive_command(int left_speed, int right_speed)
{
    control_speed(-right_speed - TRACK_TRIM,
                  -right_speed - TRACK_TRIM,
                  -left_speed + TRACK_TRIM,
                  -left_speed + TRACK_TRIM);
}

static void clear_output(void)
{
    memset(&track_output, 0, sizeof(track_output));
    track_output.base_speed = track_controller.base_speed;
    track_output.phase = track_controller.phase;
}

void track_car_drive(int left_speed, int right_speed)
{
    if (track_braking != 0U)
        return;

    drive_left_now = ramp_speed(drive_left_now, left_speed);
    drive_right_now = ramp_speed(drive_right_now, right_speed);
    send_drive_command(drive_left_now, drive_right_now);
}

void track_car_request_stop(void)
{
    int brake_left;
    int brake_right;

    if (track_braking != 0U)
        return;

    brake_left = track_controller_brake_speed(drive_left_now);
    brake_right = track_controller_brake_speed(drive_right_now);
    track_braking = 1;
    track_brake_elapsed_ms = 0;
    send_drive_command(brake_left, brake_right);
}

void track_car_stop_update(uint16_t dt_ms)
{
    if (track_braking == 0U)
        return;

    track_brake_elapsed_ms = add_ms_saturated(track_brake_elapsed_ms, dt_ms);
    if (track_brake_elapsed_ms < TRACK_BRAKE_MS)
        return;

    control_speed(0, 0, 0, 0);
    drive_left_now = 0;
    drive_right_now = 0;
    track_braking = 0;
    track_brake_elapsed_ms = 0;
}

void track_car_stop_immediate(void)
{
    control_speed(0, 0, 0, 0);
    drive_left_now = 0;
    drive_right_now = 0;
    track_braking = 0;
    track_brake_elapsed_ms = 0;
}

uint8_t track_car_is_braking(void)
{
    return track_braking;
}

void track_control_request_data(void)
{
    uart_sendstr(UART_1, "$0,1,1#");
}

void track_control_init(void)
{
    uart_init(UART_1, 115200, 0);
    track_controller_init(&track_controller);
    clear_output();
    track_consumed_sequence = track_d_sequence;
    track_frame_elapsed_ms = 0;
    track_control_request_data();
}

void track_control_start(void)
{
    track_car_stop_immediate();
    track_controller_reset(&track_controller);
    clear_output();
    track_consumed_sequence = track_d_sequence;
    track_frame_elapsed_ms = 0;
    sensor_raw = 0xFFU;
    sensor_bits = 0;
    sensor_active_count = 0;
    track_preferred_dir = 0;
}

Track_PD_t track_pd_get(void)
{
    Track_Controller_Gains_t gains;
    Track_PD_t pd;

    gains = track_controller_get_gains(&track_controller);
    pd.kp_x100 = gains.kp_x100;
    pd.kd_x100 = gains.kd_x100;
    return pd;
}

void track_pd_set(int kp_x100, int kd_x100)
{
    track_controller_set_gains(&track_controller,
                               limit_int(kp_x100, TRACK_KP_MIN, TRACK_KP_MAX),
                               limit_int(kd_x100, TRACK_KD_MIN, TRACK_KD_MAX));
}

void track_pd_adjust(int kp_delta_x100, int kd_delta_x100)
{
    Track_Controller_Gains_t gains;

    gains = track_controller_get_gains(&track_controller);
    track_pd_set(gains.kp_x100 + kp_delta_x100,
                 gains.kd_x100 + kd_delta_x100);
}

static void publish_digital_frame(const uint8_t *package, uint8_t length)
{
    uint8_t bits = 0;
    uint8_t i;
    uint8_t value;
    uint8_t offset;

    for (i = 0; i < 8; ++i)
    {
        offset = (uint8_t)(6U + i * 5U);
        if (offset >= length)
            return;

        value = package[offset];
        if (value != '0' && value != '1')
            return;
        if (value == '0')
            bits |= (uint8_t)(1U << i);
    }

    track_pending_bits = bits;
    ++track_d_sequence;
    increment_saturated(&track_frame_count);
}

static void publish_analog_frame(const uint8_t *package, uint8_t length)
{
    uint8_t i;
    uint8_t index;
    uint32_t value;

    for (i = 0; (uint16_t)i + 3U < length; ++i)
    {
        if (package[i] == 'x' &&
            package[i + 2U] == ':' &&
            package[i + 1U] >= '1' &&
            package[i + 1U] <= '8')
        {
            index = (uint8_t)(package[i + 1U] - '1');
            value = 0;
            i = (uint8_t)(i + 3U);
            while (i < length && package[i] >= '0' && package[i] <= '9')
            {
                value = value * 10U + (uint32_t)(package[i] - '0');
                if (value > 0xFFFFU)
                    value = 0xFFFFU;
                ++i;
            }
            ir_data_analog[index] = (uint16_t)value;
        }
    }

    increment_saturated(&track_a_frame_count);
    increment_saturated(&track_frame_count);
}

static void deal_track_package(const uint8_t *package, uint8_t length)
{
    if (length < 2U)
        return;

    if (package[1] == 'D')
        publish_digital_frame(package, length);
    else if (package[1] == 'A')
        publish_analog_frame(package, length);
}

void track_uart_rx(uint8_t data)
{
    static uint8_t receiving = 0;
    static uint8_t length = 0;

    if (data == '$')
    {
        receiving = 1;
        length = 0;
        ir_rx_buf[length++] = data;
        return;
    }

    if (receiving == 0U)
        return;

    if (length >= TRACK_UART_BUFFER_SIZE)
    {
        receiving = 0;
        length = 0;
        return;
    }

    ir_rx_buf[length++] = data;
    if (data == '#')
    {
        deal_track_package(ir_rx_buf, length);
        receiving = 0;
        length = 0;
    }
}

void track_follow_update(uint16_t dt_ms)
{
    uint16_t sequence;
    uint8_t bits;

    track_frame_elapsed_ms = add_ms_saturated(track_frame_elapsed_ms, dt_ms);
    sequence = track_d_sequence;
    if (sequence == track_consumed_sequence)
    {
        if (track_frame_elapsed_ms > TRACK_FRAME_TIMEOUT_MS)
        {
            track_controller.stop_requested = 1;
            track_output.stop_requested = 1;
        }
        return;
    }

    do
    {
        sequence = track_d_sequence;
        bits = track_pending_bits;
    }
    while (sequence != track_d_sequence);

    track_consumed_sequence = sequence;
    sensor_bits = bits;
    sensor_raw = (uint8_t)~bits;
    track_output = track_controller_step(&track_controller,
                                         bits,
                                         track_frame_elapsed_ms);
    track_frame_elapsed_ms = 0;
    sensor_active_count = track_output.active_count;

    if (track_output.finish_detected == 0U &&
        track_output.stop_requested == 0U)
    {
        track_car_drive(track_output.left_speed, track_output.right_speed);
    }
}

int track_read_line_error(void)
{
    if (sensor_active_count == 0U)
        return 0;
    return track_output.error;
}

uint8_t track_read_active_count(void)
{
    return sensor_active_count;
}

void track_set_preferred_dir(int8_t dir)
{
    track_preferred_dir = normalize_dir(dir);
    if (track_preferred_dir != 0)
        track_controller.last_direction = track_preferred_dir;
}

int8_t track_get_trend_dir(void)
{
    if (track_controller.last_direction != 0)
        return track_controller.last_direction;
    return track_preferred_dir;
}

uint8_t track_center_ready(void)
{
    return sensor_active_count > 0U &&
           sensor_active_count <= TRACK_ALIGN_MAX_ACTIVE &&
           (sensor_bits & TRACK_CENTER_BITS) != 0U;
}

uint8_t track_align_to_line(int8_t preferred_dir)
{
    track_set_preferred_dir(preferred_dir);
    if (track_center_ready() != 0U)
        return 1;

    if (track_output.finish_detected == 0U &&
        track_output.stop_requested == 0U)
    {
        track_car_drive(track_output.left_speed, track_output.right_speed);
    }
    return 0;
}

uint8_t track_has_line(void)
{
    return sensor_active_count > 0U;
}

uint8_t track_line_lost(void)
{
    return track_controller.phase == TRACK_PHASE_LOST_STOP;
}

void track_reset_lost_count(void)
{
    track_controller.lost_ms = 0;
    if (track_controller.phase == TRACK_PHASE_RECOVERY_HOLD ||
        track_controller.phase == TRACK_PHASE_RECOVERY_SEARCH ||
        track_controller.phase == TRACK_PHASE_LOST_STOP)
    {
        track_controller.phase = TRACK_PHASE_STRAIGHT;
        track_controller.stop_requested = 0;
        track_output.phase = TRACK_PHASE_STRAIGHT;
        track_output.stop_requested = 0;
    }
}

Track_Info_t track_get_info(void)
{
    Track_Controller_Gains_t gains;
    Track_Info_t info;
    uint8_t i;

    gains = track_controller_get_gains(&track_controller);
    info.raw = sensor_raw;
    info.bits = sensor_bits;
    info.active_count = sensor_active_count;
    info.no_frame_ms = track_frame_elapsed_ms;
    info.frame_count = track_frame_count;
    info.d_frame_count = track_d_sequence;
    info.a_frame_count = track_a_frame_count;
    info.lost_ms = track_controller.lost_ms;
    info.error = track_output.error;
    info.derivative = track_output.derivative;
    info.turn = track_output.turn;
    info.base_speed = track_output.base_speed;
    info.left_speed = track_output.left_speed;
    info.right_speed = track_output.right_speed;
    info.kp_x100 = gains.kp_x100;
    info.kd_x100 = gains.kd_x100;
    info.phase = track_output.phase;
    info.finish_detected = track_output.finish_detected;
    info.stop_requested = track_output.stop_requested;
    info.braking = track_braking;
    for (i = 0; i < 8; ++i)
        info.analog[i] = ir_data_analog[i];

    return info;
}
