#include "headfile.h"
#include "track_control.h"

#define TRACK_BASE_SPEED     200
#define TRACK_EDGE_SPEED     150
#define TRACK_SEARCH_SPEED   170
#define TRACK_MAX_TURN       340
#define TRACK_MIN_EDGE_TURN  245
#define TRACK_KP_DEFAULT     100
#define TRACK_KD_DEFAULT     60
#define TRACK_KP_MIN         0
#define TRACK_KP_MAX         300
#define TRACK_KD_MIN         0
#define TRACK_KD_MAX         200
#define TRACK_LOST_STOP_CNT  120
#define TRACK_FRAME_TIMEOUT  100
#define DRIVE_RAMP_STEP      110
#define EDGE_CONFIRM_COUNT   2
#define TRACK_WIDE_COUNT     3
#define TRACK_WIDE_RECOVERY_COUNT 5
#define TRACK_WIDE_DEADBAND  80
#define TRACK_WIDE_MIN_TURN  200
#define TRACK_CENTER_BITS    0x18
#define TRACK_RECOVERY_ERROR 330
#define TRACK_RECOVERY_CENTER_COUNT 2
#define TRACK_SIDE_VOTE_LIMIT 5
#define TRACK_SIDE_VOTE_LOCK  2
#define TRACK_MIN_DRIVE_SPEED 25
#define TRACK_EDGE_REVERSE_SPEED -30
#define TRACK_RECOVERY_FORWARD 55
#define TRACK_RECOVERY_TURN   240
#define TRACK_SEARCH_INNER_SPEED -60
#define TRACK_SWEEP_PERIOD    25
#define TRACK_ALIGN_MAX_ACTIVE 4
#define TRACK_WEAVE_TURN      22
#define TRACK_WEAVE_PERIOD    16
#define TRACK_WEAVE_ERROR_LIMIT 120
#define TRACK_WEAVE_ACTIVE_MAX 2
#define TRACK_LOCK_FRAMES     25
#define TRACK_LOCK_BASE_SPEED 88
#define TRACK_LOCK_KP         160
#define TRACK_LOCK_KD         70
#define TRACK_TURN_SIGN       1
#define TRACK_WIDE_PAUSE_FRAMES 4

static uint8_t track_no_frame_count = 0;
static uint8_t sensor_raw = 0;
static uint8_t sensor_bits = 0;
static uint8_t sensor_active_count = 0;
static uint8_t track_frame_count = 0;
static uint8_t track_d_frame_count = 0;
static uint8_t track_a_frame_count = 0;
static uint8_t track_lost_count = 0;
static int track_error = 0;
static int last_track_error = 0;
static int track_turn = 0;
static int drive_left_now = 0;
static int drive_right_now = 0;
static uint8_t left_edge_count = 0;
static uint8_t right_edge_count = 0;
static int8_t track_recovery_dir = 0;
static uint8_t track_recovery_center_count = 0;
static int8_t track_preferred_dir = -1;
static int8_t track_side_vote = 0;
static int8_t track_side_trend = 0;
static int8_t track_sweep_dir = -1;
static uint8_t track_sweep_count = 0;
static int8_t track_weave_dir = 1;
static uint8_t track_weave_count = 0;
static uint8_t track_lock_count = 0;
static uint8_t track_wide_pause_count = 0;
static int track_kp_x100 = TRACK_KP_DEFAULT;
static int track_kd_x100 = TRACK_KD_DEFAULT;

static uint8_t ir_rx_buf[100];
static uint8_t ir_package[100];
static uint8_t ir_data_number[8] = {1, 1, 1, 1, 1, 1, 1, 1};
static uint16_t ir_data_analog[8] = {0};

static int limit_int(int value, int min_value, int max_value)
{
	if(value < min_value)
	{
		return min_value;
	}
	if(value > max_value)
	{
		return max_value;
	}
	return value;
}

static uint8_t count_bits(uint8_t value)
{
	uint8_t count = 0;

	while(value)
	{
		count += value & 0x01;
		value >>= 1;
	}
	return count;
}

static int8_t normalize_dir(int8_t dir)
{
	if(dir > 0)
	{
		return 1;
	}
	if(dir < 0)
	{
		return -1;
	}
	return 0;
}

static void update_side_trend(uint8_t bits)
{
	/* Relative side trend for line-entry alignment:
	 * left-side probes dominate -> keep searching left; right-side probes dominate -> keep searching right.
	 * This is only a fallback for uncertain line-entry and lost-line recovery.
	 */
	uint8_t left_count = count_bits(bits & 0x0F);
	uint8_t right_count = count_bits((uint8_t)((bits >> 4) & 0x0F));
	int8_t delta = 0;

	if(left_count > 0 && right_count == 0)
	{
		delta = -1;
	}
	else if(right_count > 0 && left_count == 0)
	{
		delta = 1;
	}
	else if(left_count > right_count + 1)
	{
		delta = -1;
	}
	else if(right_count > left_count + 1)
	{
		delta = 1;
	}

	if(delta > 0 && track_side_vote < TRACK_SIDE_VOTE_LIMIT)
	{
		track_side_vote++;
	}
	else if(delta < 0 && track_side_vote > -TRACK_SIDE_VOTE_LIMIT)
	{
		track_side_vote--;
	}
	else if(delta == 0)
	{
		if(track_side_vote > 0)
		{
			track_side_vote--;
		}
		else if(track_side_vote < 0)
		{
			track_side_vote++;
		}
	}

	if(track_side_vote >= TRACK_SIDE_VOTE_LOCK)
	{
		track_side_trend = 1;
	}
	else if(track_side_vote <= -TRACK_SIDE_VOTE_LOCK)
	{
		track_side_trend = -1;
	}
}

static int8_t choose_track_dir(int8_t fallback_dir)
{
	if(track_side_trend != 0)
	{
		return track_side_trend;
	}
	if(track_preferred_dir != 0)
	{
		return track_preferred_dir;
	}
	if(fallback_dir != 0)
	{
		return normalize_dir(fallback_dir);
	}
	if(last_track_error > TRACK_WIDE_DEADBAND)
	{
		return normalize_dir(TRACK_TURN_SIGN);
	}
	if(last_track_error < -TRACK_WIDE_DEADBAND)
	{
		return normalize_dir(-TRACK_TURN_SIGN);
	}
	return -1;
}

static int8_t choose_sweep_dir(void)
{
	if(track_side_trend != 0)
	{
		return track_side_trend;
	}
	if(track_preferred_dir != 0)
	{
		return track_preferred_dir;
	}
	if(last_track_error > TRACK_WIDE_DEADBAND)
	{
		return normalize_dir(TRACK_TURN_SIGN);
	}
	if(last_track_error < -TRACK_WIDE_DEADBAND)
	{
		return normalize_dir(-TRACK_TURN_SIGN);
	}

	if(track_sweep_count < TRACK_SWEEP_PERIOD)
	{
		track_sweep_count++;
	}
	else
	{
		track_sweep_count = 0;
		track_sweep_dir = -track_sweep_dir;
	}
	return track_sweep_dir;
}

static int8_t track_dir_from_error(int error)
{
	if(error > TRACK_WIDE_DEADBAND)
	{
		return normalize_dir(TRACK_TURN_SIGN);
	}
	if(error < -TRACK_WIDE_DEADBAND)
	{
		return normalize_dir(-TRACK_TURN_SIGN);
	}
	return 0;
}

static void keep_min_turn_for_error(int error, int min_turn)
{
	int8_t dir = track_dir_from_error(error);

	if(dir > 0 && track_turn < min_turn)
	{
		track_turn = min_turn;
	}
	else if(dir < 0 && track_turn > -min_turn)
	{
		track_turn = -min_turn;
	}
}

static int8_t choose_wide_recovery_dir(void)
{
	int8_t dir = track_dir_from_error(track_error);

	if(dir != 0)
	{
		return dir;
	}
	if(track_recovery_dir != 0)
	{
		return track_recovery_dir;
	}
	if(last_track_error > TRACK_WIDE_DEADBAND || last_track_error < -TRACK_WIDE_DEADBAND)
	{
		return track_dir_from_error(last_track_error);
	}
	return choose_track_dir(0);
}

static int track_weave_turn(void)
{
	if(track_weave_count < TRACK_WEAVE_PERIOD)
	{
		track_weave_count++;
	}
	else
	{
		track_weave_count = 0;
		track_weave_dir = -track_weave_dir;
	}
	return track_weave_dir * TRACK_WEAVE_TURN;
}

static void drive_seek_dir(int8_t dir)
{
	int outer_speed = TRACK_RECOVERY_FORWARD + TRACK_RECOVERY_TURN;

	if(dir < 0)
	{
		track_car_drive(TRACK_SEARCH_INNER_SPEED, outer_speed);
	}
	else
	{
		track_car_drive(outer_speed, TRACK_SEARCH_INNER_SPEED);
	}
}

static void track_quick_stop_keep_state(void)
{
	drive_left_now = 0;
	drive_right_now = 0;
	control_speed(0, 0, 0, 0);
}

/* Stop immediately and reset the speed ramp state. */
void track_car_stop(void)
{
	drive_left_now = 0;
	drive_right_now = 0;
	track_recovery_dir = 0;
	track_recovery_center_count = 0;
	track_lock_count = 0;
	track_wide_pause_count = 0;
	control_speed(0, 0, 0, 0);
}

/* Send differential speed to the motor board with a small ramp to reduce jerk. */
void track_car_drive(int left_speed, int right_speed)
{
	if(left_speed > drive_left_now + DRIVE_RAMP_STEP)
	{
		drive_left_now += DRIVE_RAMP_STEP;
	}
	else if(left_speed < drive_left_now - DRIVE_RAMP_STEP)
	{
		drive_left_now -= DRIVE_RAMP_STEP;
	}
	else
	{
		drive_left_now = left_speed;
	}

	if(right_speed > drive_right_now + DRIVE_RAMP_STEP)
	{
		drive_right_now += DRIVE_RAMP_STEP;
	}
	else if(right_speed < drive_right_now - DRIVE_RAMP_STEP)
	{
		drive_right_now -= DRIVE_RAMP_STEP;
	}
	else
	{
		drive_right_now = right_speed;
	}

	control_speed(drive_left_now, drive_left_now, drive_right_now, drive_right_now);
}

/* Ask the 8-channel tracking module to upload digital and analog data. */
void track_control_request_data(void)
{
	uart_sendstr(UART_1, "$0,1,1#");
}

void track_control_init(void)
{
	uart_init(UART_1, 115200, 0);
	track_control_request_data();
}

Track_PD_t track_pd_get(void)
{
	Track_PD_t pd;

	pd.kp_x100 = track_kp_x100;
	pd.kd_x100 = track_kd_x100;
	return pd;
}

void track_pd_set(int kp_x100, int kd_x100)
{
	track_kp_x100 = limit_int(kp_x100, TRACK_KP_MIN, TRACK_KP_MAX);
	track_kd_x100 = limit_int(kd_x100, TRACK_KD_MIN, TRACK_KD_MAX);
}

void track_pd_adjust(int kp_delta_x100, int kd_delta_x100)
{
	track_pd_set(track_kp_x100 + kp_delta_x100, track_kd_x100 + kd_delta_x100);
}

static void deal_track_package(void)
{
	uint8_t i;
	uint8_t index = 0;
	uint16_t value = 0;

	if(ir_package[1] == 'D')
	{
		for(i = 0; i < 8; i++)
		{
			if(ir_package[6 + i * 5] == '0' || ir_package[6 + i * 5] == '1')
			{
				ir_data_number[i] = ir_package[6 + i * 5] - '0';
			}
		}

		if(track_d_frame_count < 255)
		{
			track_d_frame_count++;
		}
	}
	else if(ir_package[1] == 'A')
	{
		for(i = 0; i < sizeof(ir_package); i++)
		{
			if(ir_package[i] == 'x' && ir_package[i + 2] == ':' && ir_package[i + 1] >= '1' && ir_package[i + 1] <= '8')
			{
				index = ir_package[i + 1] - '1';
				value = 0;
				i += 3;
				while(ir_package[i] >= '0' && ir_package[i] <= '9')
				{
					value = value * 10 + (ir_package[i] - '0');
					i++;
				}
				ir_data_analog[index] = value;
			}
		}

		if(track_a_frame_count < 255)
		{
			track_a_frame_count++;
		}
	}
	else
	{
		return;
	}

	if(track_frame_count < 255)
	{
		track_frame_count++;
	}
	track_no_frame_count = 0;
}

void track_uart_rx(uint8_t data)
{
	static uint8_t start = 0;
	static uint8_t step = 0;
	uint8_t i;

	if(data == '$')
	{
		start = 1;
		step = 0;
		ir_rx_buf[step++] = data;
		return;
	}

	if(start == 0)
	{
		return;
	}

	if(step >= sizeof(ir_rx_buf))
	{
		for(i = 0; i < sizeof(ir_rx_buf); i++)
		{
			ir_rx_buf[i] = 0;
		}
		start = 0;
		step = 0;
		return;
	}

	ir_rx_buf[step++] = data;
	if(data == '#')
	{
		for(i = 0; i < sizeof(ir_package); i++)
		{
			ir_package[i] = ir_rx_buf[i];
			ir_rx_buf[i] = 0;
		}
		start = 0;
		step = 0;
		deal_track_package();
		return;
	}

	if(step >= sizeof(ir_rx_buf))
	{
		for(i = 0; i < sizeof(ir_rx_buf); i++)
		{
			ir_rx_buf[i] = 0;
		}
		start = 0;
		step = 0;
	}
}

static uint8_t read_track_sensors(void)
{
	uint8_t bits = 0;
	uint8_t i;

	sensor_raw = 0;
	sensor_active_count = 0;
	for(i = 0; i < 8; i++)
	{
		if(ir_data_number[i])
		{
			sensor_raw |= (1 << i);
		}
		else
		{
			bits |= (1 << i);
			sensor_active_count++;
		}
	}

	return bits;
}

/* Convert active sensor positions into a signed line-position error.
 * The correction direction is applied later through TRACK_TURN_SIGN.
 */
static int calc_track_error(uint8_t bits)
{
	/* Absolute line position for normal tracking.
	 * Negative error means the line is on the left, positive means it is on the right.
	 * TRACK_TURN_SIGN maps that line position to the car's correction direction.
	 */
	static const int sensor_weight[8] = {-350, -250, -150, -50, 50, 150, 250, 350};
	int sum = 0;
	uint8_t i;
	uint8_t left_count;
	uint8_t right_count;

	if(sensor_active_count == 0)
	{
		return last_track_error;
	}

	if(sensor_active_count >= 7)
	{
		return last_track_error;
	}

	for(i = 0; i < 8; i++)
	{
		if(bits & (1 << i))
		{
			sum += sensor_weight[i];
		}
	}

	sum = sum / sensor_active_count;

	/* When the car enters the arc diagonally, several probes can see black at
	 * the same time. A plain average can become near zero and drive straight
	 * off the arc. Bias wide detections toward the side with more active probes,
	 * or keep the previous steering direction if the pattern is symmetrical.
	 */
	if(sensor_active_count >= TRACK_WIDE_COUNT && sum > -TRACK_WIDE_DEADBAND && sum < TRACK_WIDE_DEADBAND)
	{
		left_count = count_bits(bits & 0x0F);
		right_count = count_bits((uint8_t)((bits >> 4) & 0x0F));

		if(left_count > right_count)
		{
			sum = -TRACK_WIDE_MIN_TURN;
		}
		else if(right_count > left_count)
		{
			sum = TRACK_WIDE_MIN_TURN;
		}
		else if(last_track_error > TRACK_WIDE_DEADBAND)
		{
			sum = TRACK_WIDE_MIN_TURN;
		}
		else if(last_track_error < -TRACK_WIDE_DEADBAND)
		{
			sum = -TRACK_WIDE_MIN_TURN;
		}
	}

	return sum;
}

void track_follow_update(void)
{
	int base_speed;
	int left_speed;
	int right_speed;
	int kp;
	int kd;
	uint8_t wide_line;
	uint8_t lock_active;

	sensor_bits = read_track_sensors();
	update_side_trend(sensor_bits);
	track_error = calc_track_error(sensor_bits);
	wide_line = sensor_active_count >= TRACK_WIDE_COUNT;
	lock_active = track_lock_count > 0;
	if(sensor_bits & 0x03)
	{
		if(left_edge_count < 255)
		{
			left_edge_count++;
		}
		right_edge_count = 0;
	}
	else if(sensor_bits & 0xC0)
	{
		if(right_edge_count < 255)
		{
			right_edge_count++;
		}
		left_edge_count = 0;
	}
	else
	{
		left_edge_count = 0;
		right_edge_count = 0;
	}

	if(left_edge_count >= EDGE_CONFIRM_COUNT && track_error < 300)
	{
		track_error = -300;
	}
	else if(right_edge_count >= EDGE_CONFIRM_COUNT && track_error > -300)
	{
		track_error = 300;
	}
	else if(wide_line && track_error > TRACK_WIDE_DEADBAND)
	{
		track_error = TRACK_RECOVERY_ERROR;
	}
	else if(wide_line && track_error < -TRACK_WIDE_DEADBAND)
	{
		track_error = -TRACK_RECOVERY_ERROR;
	}

	if(track_recovery_dir != 0)
	{
		if(sensor_bits & TRACK_CENTER_BITS)
		{
			if(track_recovery_center_count < 255)
			{
				track_recovery_center_count++;
			}
			if(track_recovery_center_count >= TRACK_RECOVERY_CENTER_COUNT)
			{
				track_recovery_dir = 0;
				track_recovery_center_count = 0;
			}
		}
		else
		{
			track_recovery_center_count = 0;
		}
	}

	if(sensor_active_count == 0 && track_recovery_dir > 0)
	{
		track_error = TRACK_TURN_SIGN > 0 ? TRACK_RECOVERY_ERROR : -TRACK_RECOVERY_ERROR;
	}
	else if(sensor_active_count == 0 && track_recovery_dir < 0)
	{
		track_error = TRACK_TURN_SIGN > 0 ? -TRACK_RECOVERY_ERROR : TRACK_RECOVERY_ERROR;
	}

	if(wide_line || left_edge_count >= EDGE_CONFIRM_COUNT || right_edge_count >= EDGE_CONFIRM_COUNT || track_recovery_dir != 0)
	{
		track_error = (track_error * 9 + last_track_error) / 10;
	}
	else
	{
		track_error = (track_error * 4 + last_track_error) / 5;
	}
	if(track_no_frame_count < 255)
	{
		track_no_frame_count++;
	}

	if(track_no_frame_count > TRACK_FRAME_TIMEOUT)
	{
		track_car_stop();
		return;
	}

	if(sensor_active_count == 0)
	{
		left_edge_count = 0;
		right_edge_count = 0;
		track_recovery_center_count = 0;
		if(track_lost_count < 255)
		{
			track_lost_count++;
		}

		if(track_lost_count > TRACK_LOST_STOP_CNT)
		{
			track_recovery_dir = 0;
			track_car_stop();
			return;
		}

		if(track_recovery_dir != 0)
		{
			drive_seek_dir(track_recovery_dir);
		}
		else if(last_track_error > 0)
		{
			drive_seek_dir(track_dir_from_error(last_track_error));
		}
		else if(last_track_error < 0)
		{
			drive_seek_dir(track_dir_from_error(last_track_error));
		}
		else
		{
			drive_seek_dir(choose_sweep_dir());
		}
		return;
	}

	track_lost_count = 0;
	if(track_recovery_dir != 0 &&
	   (sensor_bits & TRACK_CENTER_BITS) != 0 &&
	   sensor_active_count <= TRACK_ALIGN_MAX_ACTIVE)
	{
		track_recovery_dir = 0;
		track_recovery_center_count = 0;
		track_wide_pause_count = 0;
		track_lock_count = TRACK_LOCK_FRAMES;
	}

	if(sensor_active_count >= TRACK_WIDE_RECOVERY_COUNT)
	{
		/* Too many probes on black means the sensor is pressing across the arc.
		 * Stop briefly, then keep a small-radius turn while the line is still visible.
		 */
		left_edge_count = 0;
		right_edge_count = 0;
		if(track_recovery_dir == 0)
		{
			track_recovery_dir = choose_wide_recovery_dir();
		}
		if(track_wide_pause_count < TRACK_WIDE_PAUSE_FRAMES)
		{
			track_wide_pause_count++;
			track_quick_stop_keep_state();
		}
		else
		{
			drive_seek_dir(track_recovery_dir);
		}
		last_track_error = track_error;
		return;
	}
	track_wide_pause_count = 0;

	kp = lock_active ? TRACK_LOCK_KP : track_kp_x100;
	kd = lock_active ? TRACK_LOCK_KD : track_kd_x100;
	track_turn = TRACK_TURN_SIGN * (kp * track_error + kd * (track_error - last_track_error)) / 100;
	track_turn = limit_int(track_turn, -TRACK_MAX_TURN, TRACK_MAX_TURN);

	{
		int abs_error = track_error >= 0 ? track_error : -track_error;

		/* Use continuous steering near the center, then increase authority only near the edge. */
		if(abs_error > 300)
		{
			keep_min_turn_for_error(track_error, TRACK_MIN_EDGE_TURN);
		}
		else if(abs_error > 140)
		{
			keep_min_turn_for_error(track_error, 120);
		}

		if(sensor_active_count >= TRACK_WIDE_COUNT)
		{
			keep_min_turn_for_error(track_error, TRACK_WIDE_MIN_TURN);
		}
		else if(sensor_active_count <= TRACK_WEAVE_ACTIVE_MAX &&
		        abs_error < TRACK_WEAVE_ERROR_LIMIT &&
		        track_recovery_dir == 0)
		{
			track_turn += track_weave_turn();
			track_turn = limit_int(track_turn, -TRACK_MAX_TURN, TRACK_MAX_TURN);
		}

		if(abs_error > 300)
		{
			base_speed = TRACK_EDGE_SPEED;
		}
		else if(lock_active)
		{
			base_speed = TRACK_LOCK_BASE_SPEED;
		}
		else
		{
			base_speed = TRACK_BASE_SPEED - abs_error / 8;
			base_speed = limit_int(base_speed, TRACK_EDGE_SPEED, TRACK_BASE_SPEED);
		}
	}

	if(track_error > 300 || track_error < -300 || sensor_active_count >= TRACK_WIDE_COUNT)
	{
		left_speed = limit_int(base_speed + track_turn, TRACK_EDGE_REVERSE_SPEED, base_speed + TRACK_MAX_TURN);
		right_speed = limit_int(base_speed - track_turn, TRACK_EDGE_REVERSE_SPEED, base_speed + TRACK_MAX_TURN);
	}
	else
	{
		left_speed = limit_int(base_speed + track_turn, TRACK_MIN_DRIVE_SPEED, base_speed + TRACK_MAX_TURN);
		right_speed = limit_int(base_speed - track_turn, TRACK_MIN_DRIVE_SPEED, base_speed + TRACK_MAX_TURN);
	}
	track_car_drive(left_speed, right_speed);
	last_track_error = track_error;
	if(track_lock_count > 0)
	{
		track_lock_count--;
	}
}

void track_set_preferred_dir(int8_t dir)
{
	track_preferred_dir = normalize_dir(dir);
}

int8_t track_get_trend_dir(void)
{
	return track_side_trend;
}

uint8_t track_center_ready(void)
{
	sensor_bits = read_track_sensors();
	update_side_trend(sensor_bits);
	track_error = calc_track_error(sensor_bits);

	if(sensor_active_count == 0)
	{
		return 0;
	}
	if(sensor_active_count > TRACK_ALIGN_MAX_ACTIVE)
	{
		return 0;
	}
	if((sensor_bits & TRACK_CENTER_BITS) == 0)
	{
		return 0;
	}
	return 1;
}

uint8_t track_align_to_line(int8_t preferred_dir)
{
	int8_t dir;

	sensor_bits = read_track_sensors();
	update_side_trend(sensor_bits);
	track_error = calc_track_error(sensor_bits);

	if(sensor_active_count > 0 &&
	   (sensor_bits & TRACK_CENTER_BITS) != 0 &&
	   sensor_active_count <= TRACK_ALIGN_MAX_ACTIVE)
	{
		last_track_error = track_error;
		track_recovery_dir = 0;
		track_recovery_center_count = 0;
		track_wide_pause_count = 0;
		track_lock_count = TRACK_LOCK_FRAMES;
		return 1;
	}

	if(sensor_active_count >= TRACK_WIDE_RECOVERY_COUNT)
	{
		dir = choose_wide_recovery_dir();
		track_recovery_dir = dir;
		track_recovery_center_count = 0;
		if(track_wide_pause_count < TRACK_WIDE_PAUSE_FRAMES)
		{
			track_wide_pause_count++;
			track_quick_stop_keep_state();
		}
		else
		{
			drive_seek_dir(dir);
		}
		return 0;
	}

	track_wide_pause_count = 0;
	dir = track_dir_from_error(track_error);
	if(dir == 0)
	{
		dir = choose_track_dir(preferred_dir);
	}
	track_recovery_dir = dir;
	track_recovery_center_count = 0;
	drive_seek_dir(dir);
	return 0;
}

void track_reset_lost_count(void)
{
	track_lost_count = 0;
	track_no_frame_count = 0;
}

uint8_t track_has_line(void)
{
	sensor_bits = read_track_sensors();
	update_side_trend(sensor_bits);
	return sensor_active_count > 0;
}

uint8_t track_line_lost(void)
{
	return track_lost_count > TRACK_LOST_STOP_CNT;
}

Track_Info_t track_get_info(void)
{
	Track_Info_t info;
	uint8_t i;

	info.raw = sensor_raw;
	info.bits = sensor_bits;
	info.active_count = sensor_active_count;
	info.no_frame_count = track_no_frame_count;
	info.frame_count = track_frame_count;
	info.d_frame_count = track_d_frame_count;
	info.a_frame_count = track_a_frame_count;
	info.lost_count = track_lost_count;
	info.error = track_error;
	info.turn = track_turn;
	for(i = 0; i < 8; i++)
	{
		info.analog[i] = ir_data_analog[i];
	}

	return info;
}

/* Read sensors and return the weighted line-position error.
 * Does NOT drive motors — safe to call from seek / gap-drive states. */
int track_read_line_error(void)
{
	sensor_bits = read_track_sensors();
	update_side_trend(sensor_bits);
	if(sensor_active_count == 0)
	{
		return 0;
	}
	return calc_track_error(sensor_bits);
}

uint8_t track_read_active_count(void)
{
	return sensor_active_count;
}
