#include "irtrack_i2c.h"

/* ── I2C read with STOP between write and read (no repeated start) ── */
static uint8_t IRI2C_ReadByte(uint8_t reg)
{
    uint8_t data;
    uint8_t ack;

    /* Step1: write register address */
    I2C_Start();
    I2C_SendByte((IR_I2C_ADDR << 1) | 0);
    ack = I2C_WaitAck();
    if (ack) { I2C_Stop(); return 0xFE; }  /* addr NAK */

    I2C_SendByte(reg);
    ack = I2C_WaitAck();
    I2C_Stop();                              /* STOP here, no repeated start */
    if (ack) { return 0xFD; }                /* reg NAK */

    delay_us(10);                            /* small gap */

    /* Step2: read data */
    I2C_Start();
    I2C_SendByte((IR_I2C_ADDR << 1) | 1);
    ack = I2C_WaitAck();
    if (ack) { I2C_Stop(); return 0xFC; }   /* read NAK */

    data = I2C_ReceiveByte();
    I2C_NotSendAck();
    I2C_Stop();
    return data;
}

/* ── Sensor data ── */
uint8_t ir_data[TRACK_SENSOR_COUNT] = {1,1,1,1,1,1,1,1};

/* ── Debug: last raw byte from I2C ── */
uint8_t g_raw_byte = 0;

/* ── Debug ── */
static uint8_t g_bits   = 0;
static uint8_t g_active = 0;
static int     g_err    = 0;
static int     g_turn   = 0;

/* ── Read sensors ── */
void read_ir_sensors(void)
{
    uint8_t buf, i;
    buf = IRI2C_ReadByte(0x30);
    g_raw_byte = buf;
    for (i = 0; i < 8; i++)
        ir_data[i] = (buf >> (7 - i)) & 0x01;
}

/* ── ir_data → bitmask (0=black → 1) ── */
static void read_track_bits(void)
{
    uint8_t i;
    g_bits = 0;
    g_active = 0;
    for (i = 0; i < 8; i++)
    {
        if (ir_data[i] == 0)
        {
            g_bits |= (1 << i);
            g_active++;
        }
    }
}

/* ── PID (matching TI tutorial PID_IR_Calc) ── */
#define IRR_TRUN_KP  500
#define IRR_TRUN_KI  0
#define IRR_TRUN_KD  0

static float PID_IR_Calc(int8_t actual_value)
{
    static int8_t error_last = 0;
    static float integral = 0;
    int8_t error = actual_value;

    integral += error;
    float turn = error * IRR_TRUN_KP
               + IRR_TRUN_KI * integral
               + IRR_TRUN_KD * (error - error_last);
    error_last = error;
    return turn;
}

/* ── I2C init ── */
void irtrack_i2c_init(void)
{
    I2C_Init();
}

/* ── LineWalking (matching TI tutorial) ── */
#define IRR_SPEED  300

void LineWalking(void)
{
    int8_t err = 0;
    static uint8_t x1,x2,x3,x4,x5,x6,x7,x8;

    read_ir_sensors();
    x1 = ir_data[0]; x2 = ir_data[1]; x3 = ir_data[2]; x4 = ir_data[3];
    x5 = ir_data[4]; x6 = ir_data[5]; x7 = ir_data[6]; x8 = ir_data[7];

    /* Right-angle / acute-angle detection (matching tutorial) */
    if (x1 == 0 && x2 == 0 && x3 == 0 && x4 == 0 && x5 == 0 && x6 == 1 && x7 == 1 && x8 == 1)
    {
        err = -15;
        delay_ms(100);
    }
    else if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 0 && x5 == 0 && x6 == 0 && x7 == 0 && x8 == 0)
    {
        err = 15;
        delay_ms(100);
    }
    /* 0000 0111 */
    else if (x1 == 0 && x2 == 0 && x3 == 0 && x4 == 0 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1)
        err = -14;
    /* 1110 0000 */
    else if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 0 && x6 == 0 && x7 == 0 && x8 == 0)
        err = 14;
    /* 1000 0000 */
    else if (x1 == 0 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1)
        err = -13;
    /* 0000 0001 */
    else if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 0)
        err = 13;
    /* 1100 0000 */
    else if (x1 == 0 && x2 == 0 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1)
        err = -12;
    /* 0000 0011 */
    else if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 0 && x8 == 0)
        err = 12;
    /* 1110 0000 */
    else if (x1 == 0 && x2 == 0 && x3 == 0 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1)
        err = -11;
    /* 0000 0111 */
    else if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 0 && x7 == 0 && x8 == 0)
        err = 11;
    /* 1111 0000 */
    else if (x1 == 0 && x2 == 0 && x3 == 0 && x4 == 0 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1)
        err = -10;
    /* 0000 1111 */
    else if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 0 && x6 == 0 && x7 == 0 && x8 == 0)
        err = 10;
    /* 0111 1000 */
    else if (x1 == 1 && x2 == 0 && x3 == 0 && x4 == 0 && x5 == 0 && x6 == 1 && x7 == 1 && x8 == 1)
        err = -9;
    /* 0001 1110 */
    else if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 0 && x5 == 0 && x6 == 0 && x7 == 0 && x8 == 1)
        err = 9;
    /* 0011 1100 */
    else if (x1 == 1 && x2 == 1 && x3 == 0 && x4 == 0 && x5 == 0 && x6 == 0 && x7 == 1 && x8 == 1)
        err = -8;
    /* 0011 1100 */
    else if (x1 == 1 && x2 == 1 && x3 == 0 && x4 == 0 && x5 == 0 && x6 == 0 && x7 == 1 && x8 == 1)
        err = 8;
    /* 0001 1110 */
    else if (x1 == 1 && x2 == 0 && x3 == 0 && x4 == 0 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1)
        err = -7;
    /* 0111 1000 */
    else if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 0 && x6 == 0 && x7 == 0 && x8 == 1)
        err = 7;
    /* 0000 1111 */
    else if (x1 == 1 && x2 == 1 && x3 == 0 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1)
        err = -6;
    /* 1111 0000 */
    else if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 0 && x7 == 1 && x8 == 1)
        err = 6;
    /* 0000 0111 */
    else if (x1 == 1 && x2 == 0 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1)
        err = -5;
    /* 1110 0000 */
    else if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 0 && x8 == 1)
        err = 5;
    /* 0000 0011 */
    else if (x1 == 1 && x2 == 0 && x3 == 0 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1)
        err = -4;
    /* 1100 0000 */
    else if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 0 && x7 == 0 && x8 == 1)
        err = 4;
    /* 0000 0001 */
    else if (x1 == 1 && x2 == 0 && x3 == 0 && x4 == 0 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1)
        err = -3;
    /* 1000 0000 */
    else if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 0 && x6 == 0 && x7 == 0 && x8 == 1)
        err = 3;
    /* 0000 0001 */
    else if (x1 == 1 && x2 == 0 && x3 == 0 && x4 == 0 && x5 == 0 && x6 == 1 && x7 == 1 && x8 == 1)
        err = -2;
    /* 1000 0000 */
    else if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 0 && x5 == 0 && x6 == 0 && x7 == 0 && x8 == 1)
        err = 2;
    /* 0000 0001 */
    else if (x1 == 1 && x2 == 0 && x3 == 0 && x4 == 0 && x5 == 0 && x6 == 0 && x7 == 1 && x8 == 1)
        err = -1;
    /* 1000 0000 */
    else if (x1 == 1 && x2 == 1 && x3 == 0 && x4 == 0 && x5 == 0 && x6 == 0 && x7 == 0 && x8 == 1)
        err = 1;

    int turn = (int)(PID_IR_Calc(err));

    g_err  = err;
    g_turn = turn;
    read_track_bits();

    /* Motion_Car_Control(IRR_SPEED, 0, turn):
     * left = speed + turn, right = speed - turn */
    int left_speed  = IRR_SPEED + turn;
    int right_speed = IRR_SPEED - turn;
    control_speed(left_speed, left_speed, right_speed, right_speed);
}

/* ── Line check ── */
int LineCheck(void)
{
    uint8_t i;
    read_ir_sensors();
    for (i = 0; i < TRACK_SENSOR_COUNT; i++)
        if (ir_data[i] == 0) return 1;
    return 0;
}

/* ── OLED getters ── */
uint8_t track_get_bits(void)   { return g_bits; }
uint8_t track_get_active(void) { return g_active; }
int     track_get_error(void)  { return g_err; }
int     track_get_turn(void)   { return g_turn; }
void track(void) { }
