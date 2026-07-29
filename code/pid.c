#include "headfile.h"

pid_t motorA;
pid_t motorB;
pid_t angle;

void datavision_send()  // host waveform send
{
	uart_sendbyte(UART_1, 0x03);
	uart_sendbyte(UART_1, 0xfc);

	uart_sendbyte(UART_1, (uint8_t)motorA.target);
	uart_sendbyte(UART_1, (uint8_t)motorA.now);
//	uart_sendbyte(UART_1, (uint8_t)motorB.target);
//	uart_sendbyte(UART_1, (uint8_t)motorB.now);

	uart_sendbyte(UART_1, 0xfc);
	uart_sendbyte(UART_1, 0x03);
}

void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d)
{
	pid->pid_mode = mode;
	pid->p = p;
	pid->i = i;
	pid->d = d;
}

void motor_target_set(int spe1, int spe2)
{
	if(spe1 >= 0)
	{
		motorA_dir = 1;
		motorA.target = spe1;
	}
	else
	{
		motorA_dir = 0;
		motorA.target = -spe1;
	}

	if(spe2 >= 0)
	{
		motorB_dir = 1;
		motorB.target = spe2;
	}
	else
	{
		motorB_dir = 0;
		motorB.target = -spe2;
	}
}

void pid_control()
{
	track();

	if(motorA_dir){motorA.now = Encoder_count1;}else{motorA.now = -Encoder_count1;}
	if(motorB_dir){motorB.now = Encoder_count2;}else{motorB.now = -Encoder_count2;}
	Encoder_count1 = 0;
	Encoder_count2 = 0;

	pid_cal(&motorA);
	pid_cal(&motorB);
	pidout_limit(&motorA);
	pidout_limit(&motorB);
	motorA_duty(motorA.out);
	motorB_duty(motorB.out);

//	datavision_send();
}

void pid_cal(pid_t *pid)
{
	pid->error[0] = pid->target - pid->now;

	if(pid->pid_mode == DELTA_PID)
	{
		pid->pout = pid->p * (pid->error[0] - pid->error[1]);
		pid->iout = pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
		pid->out += pid->pout + pid->iout + pid->dout;
	}
	else if(pid->pid_mode == POSITION_PID)
	{
		pid->pout = pid->p * pid->error[0];
		pid->iout += pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - pid->error[1]);
		pid->out = pid->pout + pid->iout + pid->dout;
	}

	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
}

void pidout_limit(pid_t *pid)
{
	if(pid->out >= MAX_DUTY)
	{
		pid->out = MAX_DUTY;
	}
	if(pid->out <= 0)
	{
		pid->out = 0;
	}
}