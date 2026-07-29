#ifndef __IRTRACK_I2C_H__
#define __IRTRACK_I2C_H__

#include "headfile.h"

#define TRACK_SENSOR_COUNT  8
#define IR_I2C_ADDR         0x12

extern uint8_t ir_data[TRACK_SENSOR_COUNT];

void irtrack_i2c_init(void);
void read_ir_sensors(void);
void LineWalking(void);
int  LineCheck(void);

/* OLED debug */
uint8_t track_get_bits(void);
uint8_t track_get_active(void);
int     track_get_error(void);
int     track_get_turn(void);

#endif
