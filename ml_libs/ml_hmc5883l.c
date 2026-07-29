#include "ml_hmc5883l.h"

#define MAG_TYPE_HMC5883L 0
#define MAG_TYPE_QMC5883L 1

int16_t hmc_x, hmc_y, hmc_z;
float yaw_hmc;
static uint8_t hmc_addr = HMC5883L_ADDR;
static uint8_t hmc_type = MAG_TYPE_HMC5883L;

static void mag_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t dat)
{
	I2C_Start();
	I2C_SendByte(dev_addr);
	I2C_WaitAck();
	I2C_SendByte(reg_addr);
	I2C_WaitAck();
	I2C_SendByte(dat);
	I2C_WaitAck();
	I2C_Stop();
}

static uint8_t mag_read(uint8_t dev_addr, uint8_t reg_addr)
{
	uint8_t dat;

	I2C_Start();
	I2C_SendByte(dev_addr);
	I2C_WaitAck();
	I2C_SendByte(reg_addr);
	I2C_WaitAck();
	I2C_Stop();

	I2C_Start();
	I2C_SendByte(dev_addr | 0x01);
	I2C_WaitAck();
	dat = I2C_ReceiveByte();
	I2C_NotSendAck();
	I2C_Stop();

	return dat;
}

void HMC5883L_Write(uint8_t addr, uint8_t dat)
{
	mag_write(hmc_addr, addr, dat);
}

uint8_t HMC5883L_Read(uint8_t addr)
{
	return mag_read(hmc_addr, addr);
}

void HMC5883L_Init(void)
{
	uint8_t id_a;
	uint8_t id_b;
	uint8_t id_c;

	hmc_addr = HMC5883L_ADDR;
	id_a = mag_read(HMC5883L_ADDR, HMC5883L_IRA);
	id_b = mag_read(HMC5883L_ADDR, HMC5883L_IRB);
	id_c = mag_read(HMC5883L_ADDR, HMC5883L_IRC);

	if((id_a == 'H') && (id_b == '4') && (id_c == '3'))
	{
		hmc_type = MAG_TYPE_HMC5883L;
		hmc_addr = HMC5883L_ADDR;
		mag_write(hmc_addr, HMC5883L_CRA, 0x78);
		mag_write(hmc_addr, HMC5883L_CRB, 0x20);
		mag_write(hmc_addr, HMC5883L_MR, 0x00);
	}
	else
	{
		hmc_type = MAG_TYPE_QMC5883L;
		hmc_addr = QMC5883L_ADDR;
		mag_write(hmc_addr, QMC5883L_SR, 0x01);
		mag_write(hmc_addr, QMC5883L_CR2, 0x00);
		mag_write(hmc_addr, QMC5883L_CR1, 0x1D);
	}
}

void HMC5883L_GetData(void)
{
	uint8_t data_h;
	uint8_t data_l;

	if(hmc_type == MAG_TYPE_HMC5883L)
	{
		data_h = mag_read(hmc_addr, HMC5883L_DOXMR);
		data_l = mag_read(hmc_addr, HMC5883L_DOXLR);
		hmc_x = data_l | (data_h << 8);

		data_h = mag_read(hmc_addr, HMC5883L_DOYMR);
		data_l = mag_read(hmc_addr, HMC5883L_DOYLR);
		hmc_y = data_l | (data_h << 8);

		data_h = mag_read(hmc_addr, HMC5883L_DOZMR);
		data_l = mag_read(hmc_addr, HMC5883L_DOZLR);
		hmc_z = data_l | (data_h << 8);
	}
	else
	{
		data_l = mag_read(hmc_addr, QMC5883L_X_LSB);
		data_h = mag_read(hmc_addr, QMC5883L_X_MSB);
		hmc_x = data_l | (data_h << 8);

		data_l = mag_read(hmc_addr, QMC5883L_Y_LSB);
		data_h = mag_read(hmc_addr, QMC5883L_Y_MSB);
		hmc_y = data_l | (data_h << 8);

		data_l = mag_read(hmc_addr, QMC5883L_Z_LSB);
		data_h = mag_read(hmc_addr, QMC5883L_Z_MSB);
		hmc_z = data_l | (data_h << 8);
	}
}
