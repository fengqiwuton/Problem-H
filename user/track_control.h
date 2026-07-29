#ifndef __TRACK_CONTROL_H__
#define __TRACK_CONTROL_H__

#include "stdint.h"

typedef struct
{
	/* Raw digital data from the module. */
	uint8_t raw;
	/* Active black-line bits after polarity conversion. */
	uint8_t bits;
	uint8_t active_count;
	uint8_t no_frame_count;
	uint8_t frame_count;
	uint8_t d_frame_count;
	uint8_t a_frame_count;
	uint8_t lost_count;
	int error;
	int turn;
	uint16_t analog[8];
} Track_Info_t;

typedef struct
{
	/* Kp * 100, for example 78 means 0.78. */
	int kp_x100;
	/* Kd * 100, for example 28 means 0.28. */
	int kd_x100;
} Track_PD_t;

/* UART tracking module */
void track_control_init(void);
void track_control_request_data(void);
void track_uart_rx(uint8_t data);

/* Read sensors and return weighted steering error.
 * Positive = steer right, negative = steer left.
 * Returns 0 when no line is detected - check track_read_active_count() > 0
 * to tell "centered" from "no line". */
int  track_read_line_error(void);
uint8_t track_read_active_count(void);
/* Line-following control */
void track_follow_update(void);
void track_car_drive(int left_speed, int right_speed);
void track_car_stop(void);
void track_set_preferred_dir(int8_t dir);
int8_t track_get_trend_dir(void);
uint8_t track_center_ready(void);
uint8_t track_align_to_line(int8_t preferred_dir);
uint8_t track_has_line(void);
uint8_t track_line_lost(void);
void track_reset_lost_count(void);
Track_Info_t track_get_info(void);

/* Runtime PD tuning. Values are stored as Kp/Kd * 100 to avoid float UI code. */
Track_PD_t track_pd_get(void);
void track_pd_set(int kp_x100, int kd_x100);
void track_pd_adjust(int kp_delta_x100, int kd_delta_x100);

#endif

