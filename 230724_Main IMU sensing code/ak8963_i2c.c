/*
 * ak8963_i2c.c
 *
 * Created: 2023-06-27 오후 6:09:11
 *  Author: stu11
 */ 
#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>

#include <stdio.h>

#include "i2c.h"
#include "ak8963_i2c.h"

static int write_a_byte(uint8_t addr, uint8_t data);
static int read_n_bytes(uint8_t addr, uint8_t *data, int n);

static float mag_scale = 1/6.66938;

int AK8963I2CReadMAGNETO(int16_t mag[]){
	int i;
	unsigned char buf[6];
	unsigned char *ptr_mag;
	
	if(read_n_bytes(MAGNETO_XOUT_H, buf, 6) < 0)
	return -1;
	
	
	ptr_mag  = (unsigned char *) mag;
	
	for(i=0; i<6; i+=2)
	{
		ptr_mag[i]   = buf[i+1];
		ptr_mag[i+1] = buf[i];

	}

	return 0;
	
}


int AK8963I2CReadMAGNETO_f(float mag_f[])
{
	int i;
	unsigned char buf[6];
	unsigned char *ptr_mag;
	int16_t  mag[3];
	
	if(read_n_bytes(MAGNETO_XOUT_H, buf, 6) < 0)
	return -1;
	
	ptr_mag  = (unsigned char *) mag;
	
	for(i=0; i<6; i+=2)
	{
		ptr_mag[i]   = buf[i+1];
		ptr_mag[i+1] = buf[i];
	}

	mag_f[0] = mag_scale  * ( mag[0]);
	mag_f[1] = mag_scale  * ( mag[1]);
	mag_f[2] = mag_scale  * ( mag[2]);
	
	return 0;
}


static int write_a_byte(uint8_t addr, uint8_t data)
{
	uint8_t	buffer[2];

	buffer[0] = addr;
	buffer[1] = data;
	
	return(i2c_write_n_bytes(AK8963_I2C_ADDR, buffer, 2, 1));
}


//=========================================-------===================
//	read n byte.
//
//  addr : register address.
//	data : pointer to buffer where receiving data are saved.
//	n    : number of data to be read
//===================================================================
//
static int read_n_bytes(uint8_t addr, uint8_t *data, int n)
{
	if(i2c_write_a_byte(AK8963_I2C_ADDR, addr | 0x80, 0)<0)		// send addr without STOP condition
	return -1;

	return(i2c_read_n_bytes(AK8963_I2C_ADDR, data, n));
}
