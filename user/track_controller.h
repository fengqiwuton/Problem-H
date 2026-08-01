#ifndef __TRACK_CONTROLLER_H__
#define __TRACK_CONTROLLER_H__

#include <stdint.h>

typedef enum
{
    TRACK_PHASE_STRAIGHT = 0,
    TRACK_PHASE_CURVE,
    TRACK_PHASE_RECOVERY_HOLD,
    TRACK_PHASE_RECOVERY_SEARCH,
    TRACK_PHASE_LOST_STOP
} Track_Controller_Phase_t;

typedef struct
{
    int kp_x100;
    int kd_x100;
} Track_Controller_Gains_t;

typedef struct
{
    int error;
    int last_error;
    int derivative;
    int turn;
    int base_speed;
    int8_t last_direction;
    uint8_t curve_enter_frames;
    uint8_t curve_exit_frames;
    uint8_t center_frames;
    uint8_t finish_frames;
    uint16_t lost_ms;
    uint16_t exit_hold_ms;
    Track_Controller_Gains_t gains;
    Track_Controller_Phase_t phase;
    uint8_t finish_detected;
    uint8_t stop_requested;
} Track_Controller_t;

typedef struct
{
    int error;
    int derivative;
    int turn;
    int base_speed;
    int left_speed;
    int right_speed;
    uint8_t active_count;
    Track_Controller_Phase_t phase;
    uint8_t finish_detected;
    uint8_t stop_requested;
} Track_Controller_Output_t;

typedef struct
{
    int motor_1;
    int motor_2;
    int motor_3;
    int motor_4;
} Track_Motor_Speeds_t;

void track_controller_init(Track_Controller_t *controller);
void track_controller_reset(Track_Controller_t *controller);
void track_controller_set_gains(Track_Controller_t *controller, int kp_x100, int kd_x100);
Track_Controller_Gains_t track_controller_get_gains(const Track_Controller_t *controller);
Track_Controller_Output_t track_controller_step(Track_Controller_t *controller,
                                                uint8_t bits,
                                                uint16_t dt_ms);
int track_controller_brake_speed(int speed);
Track_Motor_Speeds_t track_controller_map_motor_speeds(int left_speed,
                                                       int right_speed,
                                                       int trim);

#endif
