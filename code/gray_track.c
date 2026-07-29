#include "headfile.h"

#define IR_TRACK_SPEED      120
#define IR_TRACK_TURN_KP    12
#define IR_TRACK_TURN_KI    0
#define IR_TRACK_TURN_KD    0
#define IR_TRACK_MAX_TURN   80

static int pid_output_IRR = 0;

static int limit_int(int value, int min, int max)
{
	if(value < min)
	{
		return min;
	}
	if(value > max)
	{
		return max;
	}
	return value;
}

void gray_init()
{
	gpio_init(GPIO_B, Pin_12, IU);   // D1
	gpio_init(GPIO_B, Pin_13, IU);   // D2
	gpio_init(GPIO_B, Pin_14, IU);   // D3
	gpio_init(GPIO_B, Pin_15, IU);   // D4
	gpio_init(GPIO_A, Pin_8, IU);    // D5
	gpio_init(GPIO_C, Pin_13, IU);   // D6
	gpio_init(GPIO_C, Pin_14, IU);   // D7
	gpio_init(GPIO_C, Pin_15, IU);   // D8
}

static int PID_IR_Calc(int8_t actual_value)
{
	static int8_t error_last = 0;
	static int IRTrack_Integral = 0;
	int error = actual_value;
	int IRTrackTurn;

	IRTrack_Integral += error;
	IRTrackTurn = error * IR_TRACK_TURN_KP
		+ IR_TRACK_TURN_KI * IRTrack_Integral
		+ IR_TRACK_TURN_KD * (error - error_last);
	error_last = error;

	return limit_int(IRTrackTurn, -IR_TRACK_MAX_TURN, IR_TRACK_MAX_TURN);
}

void track()                        // 1234 5678
{
	int8_t err = 0;
	u8 x1 = D1;
	u8 x2 = D2;
	u8 x3 = D3;
	u8 x4 = D4;
	u8 x5 = D5;
	u8 x6 = D6;
	u8 x7 = D7;
	u8 x8 = D8;

	if(x1 == 0 && x2 == 0 && x3 == 0 && x4 == 0 && x5 == 0 && x6 == 1 && x7 == 1 && x8 == 1)       // 0000 0111
	{
		err = -15;
	}
	else if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 0 && x5 == 0 && x6 == 0 && x7 == 0 && x8 == 0)  // 1110 0000
	{
		err = 15;
	}
	else if(x1 == 0 && x2 == 0 && x7 == 0 && x8 == 0)
	{
		err = 0;
	}
	else if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 0 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1)  // 1110 1111
	{
		err = -1;
	}
	else if(x1 == 1 && x2 == 1 && x3 == 0 && x4 == 0 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1)  // 1100 1111
	{
		err = -2;
	}
	else if(x1 == 1 && x2 == 0 && x3 == 0 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1)  // 1001 1111
	{
		err = -8;
	}
	else if(x1 == 0 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1)  // 0111 1111
	{
		err = -10;
	}
	else if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 0 && x6 == 1 && x7 == 1 && x8 == 1)  // 1111 0111
	{
		err = 1;
	}
	else if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 0 && x6 == 0 && x7 == 1 && x8 == 1)  // 1111 0011
	{
		err = 2;
	}
	else if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 0 && x7 == 0 && x8 == 1)  // 1111 1001
	{
		err = 8;
	}
	else if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 0)  // 1111 1110
	{
		err = 10;
	}
	else if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 0 && x5 == 0 && x6 == 1 && x7 == 1 && x8 == 1)  // straight
	{
		err = 0;
	}

	pid_output_IRR = PID_IR_Calc(err);
	motor_target_set(IR_TRACK_SPEED + pid_output_IRR, IR_TRACK_SPEED - pid_output_IRR);
}

void LineWalking(void)
{
	track();
}

unsigned char digtal(unsigned char channel) // 1-8 get digital value
{
	u8 value = 0;
	switch(channel) 
	{
		case 1:  
			if(gpio_get(GPIO_B, Pin_12) == 1) value = 1;
			else value = 0;  
			break;  
		case 2: 
			if(gpio_get(GPIO_B, Pin_13) == 1) value = 1;
			else value = 0;  
			break;  
		case 3: 
			if(gpio_get(GPIO_B, Pin_14) == 1) value = 1;
			else value = 0;  
			break;   
		case 4:  
			if(gpio_get(GPIO_B, Pin_15) == 1) value = 1;
			else value = 0;  
			break;   
		case 5:
			if(gpio_get(GPIO_A, Pin_8) == 1) value = 1;
			else value = 0;  
			break;
		case 6:  
			if(gpio_get(GPIO_C, Pin_13) == 1) value = 1;
			else value = 0;  
			break;  
		case 7: 
			if(gpio_get(GPIO_C, Pin_14) == 1) value = 1;
			else value = 0;  
			break;  
 		case 8: 
 			if(gpio_get(GPIO_C, Pin_15) == 1) value = 1;
 			else value = 0;  
 			break;   
	}
	return value; 
}