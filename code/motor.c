#include "motor.h"

#define USART_MOTOR_TYPE       2
#define USART_MOTOR_PHASE      20
#define USART_MOTOR_PLUSE_LINE 13
#define USART_WHEEL_DIAMETER   48.0f
#define USART_MOTOR_DEADZONE   1600
#define USART_PWM_SCALE        20

uint8_t motorA_dir = 1;
uint8_t motorB_dir = 1;

int Encoder_count1 = 0;
int Encoder_count2 = 0;
int Encoder_Offset[4] = {0};
int Encoder_Now[4] = {0};
int speed_now;

static int16_t motorA_output = 0;
static int16_t motorB_output = 0;
static char motor_send_buf[50];

static void motor_uart_send_cmd(const char *cmd)
{
	uart_sendstr(UART_2, (char *)cmd);
}

static int16_t duty_to_speed(int duty, uint8_t dir)
{
	int32_t speed;

	if(duty < 0)
	{
		duty = -duty;
		dir = !dir;
	}

	speed = duty / USART_PWM_SCALE;
	if(speed > 1000)
	{
		speed = 1000;
	}

	return dir ? (int16_t)speed : (int16_t)-speed;
}

void motor_init(void)
{
	IIC_Motor_Init();
}

void IIC_Motor_Init(void)
{
	uart_init(UART_2, 115200, 1);
	USART2->CR1 &= ~(1 << 5);

	motor_uart_send_cmd("$upload:0,0,0#");
	delay_ms(10);
	control_speed(0, 0, 0, 0);
	delay_ms(100);

	Set_motor_type(USART_MOTOR_TYPE);
	delay_ms(100);
	Set_Pluse_Phase(USART_MOTOR_PHASE);
	delay_ms(100);
	Set_Pluse_line(USART_MOTOR_PLUSE_LINE);
	delay_ms(100);
	Set_Wheel_dis(USART_WHEEL_DIAMETER);
	delay_ms(100);
	Set_motor_deadzone(USART_MOTOR_DEADZONE);
	delay_ms(100);

	control_speed(0, 0, 0, 0);
}

void Set_motor_type(uint8_t data)
{
	sprintf(motor_send_buf, "$mtype:%d#", data);
	motor_uart_send_cmd(motor_send_buf);
}

void Set_motor_deadzone(uint16_t data)
{
	sprintf(motor_send_buf, "$deadzone:%d#", data);
	motor_uart_send_cmd(motor_send_buf);
}

void Set_Pluse_line(uint16_t data)
{
	sprintf(motor_send_buf, "$mline:%d#", data);
	motor_uart_send_cmd(motor_send_buf);
}

void Set_Pluse_Phase(uint16_t data)
{
	sprintf(motor_send_buf, "$mphase:%d#", data);
	motor_uart_send_cmd(motor_send_buf);
}

void Set_Wheel_dis(float data)
{
	sprintf(motor_send_buf, "$wdiameter:%.3f#", data);
	motor_uart_send_cmd(motor_send_buf);
}

void control_speed(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
	sprintf(motor_send_buf, "$spd:%d,%d,%d,%d#", m1, m2, m3, m4);
	motor_uart_send_cmd(motor_send_buf);
}

void control_pwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
	sprintf(motor_send_buf, "$pwm:%d,%d,%d,%d#", m1, m2, m3, m4);
	motor_uart_send_cmd(motor_send_buf);
}

void Read_10_Enconder(void)
{
}

void Read_ALL_Enconder(void)
{
}

void motorA_duty(int duty)
{
	motorA_output = duty_to_speed(duty, motorA_dir);
	control_speed(motorA_output, motorA_output, motorB_output, motorB_output);
}

void motorB_duty(int duty)
{
	motorB_output = duty_to_speed(duty, motorB_dir);
	control_speed(motorA_output, motorA_output, motorB_output, motorB_output);
}

void encoder_init(void)
{
}
