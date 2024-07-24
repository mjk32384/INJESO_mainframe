/*
 * ak8963_i2c.h
 *
 * Created: 2023-06-27 오후 6:12:33
 *  Author: stu11
 */ 

#include <avr/io.h>

#define AK8963_I2C_ADDR		0x48

#define MAGNETO_XOUT_H		0x03
#define MAGNETO_XOUT_L		0x04
#define MAGNETO_YOUT_H		0x05
#define MAGNETO_YOUT_L		0x06
#define MAGNETO_ZOUT_H		0x07
#define MAGNETO_ZOUT_L		0x08


int AK8963I2CInit(int32_t spi_clk);
int AK8963I2CReadMAGNETO(int16_t mag[]);
int AK8963I2CReadMAGNETO_f(float mag_f[]);
