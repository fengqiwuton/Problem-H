#ifndef __BALL_BALANCE_H__
#define __BALL_BALANCE_H__

#include "headfile.h"

/* Ball position from camera, in 0.1mm units (e.g. 500 = +5.0cm, -500 = -5.0cm) */
extern int ball_position_raw;
/* Ball position valid flag (set by camera module when data is fresh) */
extern uint8_t ball_position_valid;

/* ── Single-loop PD: camera ball position → servo pulse ── */
#define BALANCE_KP        35    /* P gain: position error(mm) → pulse change(us) */
#define BALANCE_KD        15    /* D gain: error derivative → pulse change(us) */
#define BALANCE_MAX_DELTA  400  /* max servo pulse deviation from center */

/* Servo */
#define SERVO_CENTER_US    1500
#define SERVO_MIN_US        800
#define SERVO_MAX_US        2200

/* Task 3 constants */
#define TASK_3_MOVE_TIME_MS    2000
#define TASK_3_HOLD_TIME_MS    1500
#define TASK_3_TOTAL_MS        5000
#define TASK_3_TARGET_PLUS      500   /* +5.0cm */
#define TASK_3_TARGET_MINUS   (-500)  /* -5.0cm */
#define TASK_3_TOLERANCE        100   /* ±1.0cm */

void ball_balance_init(void);
void ball_balance_set_target(int target_raw);
void ball_balance_update(void);
void ball_balance_servo_center(void);
int  ball_balance_pos_error(void);

#endif
