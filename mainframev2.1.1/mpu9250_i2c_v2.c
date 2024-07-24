/*
 * mpu9250_i2c.c
 *
 * Created: 2023-06-28 오후 11:01:32
 *  Author: stu11
 */ 

#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>

#include <stdio.h>

#include "i2c.h"
#include "mpu9250_i2c_v2.h"


static int write_a_byte(uint8_t addr, uint8_t data);
static int read_n_bytes(uint8_t addr, uint8_t *data, int n);

int MPU9250I2CInit(int32_t i2c_clk)
{
	i2c_init(i2c_clk);
	
	//-------------------------------------------------
	// Reset MPU6500
	//------------------------------------------------
	write_a_byte(PWR_MGMT_1, 1<<7);			// wake-up, reset and wait 100msec
	_delay_ms(10);_delay_ms(10);_delay_ms(10);_delay_ms(10);_delay_ms(10);
	_delay_ms(10);_delay_ms(10);_delay_ms(10);_delay_ms(10);_delay_ms(10);

	//-------------------------------------------------
	// Reset signal path for I2C Interface
	//------------------------------------------------
	write_a_byte(SIGNAL_PATH_RESET, 7);		// reset signal path
	_delay_ms(10);_delay_ms(10);_delay_ms(10);_delay_ms(10);_delay_ms(10);
	_delay_ms(10);_delay_ms(10);_delay_ms(10);_delay_ms(10);_delay_ms(10);
	write_a_byte(SIGNAL_PATH_RESET, 0);		// reset signal path
	_delay_ms(10);
	
	write_a_byte(USER_CTRL, 0x20);					// Enable I2C Master Mode
	write_a_byte(I2C_MST_CTRL, 0x0D);				// I2C configuration multi-master I2C 400khz
	_delay_ms(10);
	
	
	write_a_byte(I2C_SLV0_ADDR, 0x0c); // set the i2c slave address of ak8963(0x0c) write mode
	write_a_byte(I2C_SLV0_REG, 0x0B); // i2c slave 0 register address from where to begin data transfer
	write_a_byte(I2C_SLV0_DO, 0x01); // Reset Ak8963
	write_a_byte(I2C_SLV0_CTRL, 0x81); //enable i2c and transfer 1 byte
	_delay_ms(50);
	
	write_a_byte(I2C_SLV0_ADDR, 0x0c); // set the i2c slave address of ak8963(0x0c) write mode
	write_a_byte(I2C_SLV0_REG, 0x0A); // i2c slave 0 register address from where to begin data transfer
	write_a_byte(I2C_SLV0_DO, 0x10); // 16bit, power down
	write_a_byte(I2C_SLV0_CTRL, 0x81); //enable i2c and transfer 1 byte
	_delay_ms(50);
		
	write_a_byte(I2C_SLV0_ADDR, 0x0c); // set the i2c slave address of ak8963(0x0c) write mode
	write_a_byte(I2C_SLV0_REG, 0x0A); // i2c slave 0 register address from where to begin data transfer
	write_a_byte(I2C_SLV0_DO, 0x16); // 16bit,
	write_a_byte(I2C_SLV0_CTRL, 0x81); //enable i2c and transfer 1 byte
	_delay_ms(50);
	write_a_byte(I2C_SLV0_ADDR, 0x8c); //set the i2c slave address of ak8963(0x0c) read mode
	write_a_byte(I2C_SLV0_REG, 0x03); //i2c slave 0 register address from where to begin data transfer
	write_a_byte(I2C_SLV0_CTRL, 0x87); //enable i2c and read 6 byte
	_delay_ms(50);	
	
	
	return 0;
}

int8_t MPU9250I2CWhoAmI(void)
{
	int8_t me;

	if(read_n_bytes(WHO_AM_I, (uint8_t *) &me, 1) < 0)
	return -1;

	return me;
}

//--------------------------------------------
// Set accelerometer measurement range
//
//	range :	0 : +/- 2g				---- default
//			1 : +/- 4g
//			2 : +/- 8g
//			3 : +/- 16g
//--------------------------------------------
//
static float acc_scale  = 9.8/16384.;
static float gyro_scale = ((3.1415926/180.)/131.);

int MPU9250I2CSetAccRange(uint8_t range)
{
	if(write_a_byte(ACCEL_CONFIG, range <<3) <0)
	return -1;
	
	acc_scale = pow(2, range) * (9.8/16384.);

	return 0;
}

//--------------------------------------------
// Set gyro measurement range
//
//	range :	0 : +/- 250  deg/s		---- default
//			1 : +/- 500  deg/s
//			2 : +/- 1000 deg/s
//			3 : +/- 2000 deg/s
//--------------------------------------------

int MPU9250I2CSetGyroRange(uint8_t range)
{
	if(write_a_byte(GYRO_CONFIG, range<<3) < 0)
	return -1;
	
	gyro_scale = pow(2, range) * ((3.1415926/180.)/131.);

	return 0;
}

//-------------------------------------------------
// Read IMU data
//
// Output:	acc[] : specific force
//			gyro[]: angular velocity
//-------------------------------------------------
int MPU9250I2CReadIMU(int16_t acc[], int16_t gyro[])
{
	int i;
	unsigned char buf[14];
	unsigned char *ptr_acc, *ptr_gyro;
	
	if(read_n_bytes(ACCEL_XOUT_H, buf, 14) < 0)
	return -1;
	
	ptr_acc  = (unsigned char *) acc;
	ptr_gyro = (unsigned char *) gyro;
	
	for(i=0; i<6; i+=2)
	{
		ptr_acc[i]   = buf[i+1];
		ptr_acc[i+1] = buf[i];
		
		ptr_gyro[i]  = buf[i+8+1];
		ptr_gyro[i+1]= buf[i+8];
	}

	return 0;
}


//-------------------------------------------------
// Read IMU data
//
// Output:	acc_f[] : specific force(m/s/s)
//			gyro_f[]: angular velocity(rad/s)
//-------------------------------------------------

int MPU9250I2CReadIMU_f(float acc_f[], float gyro_f[])
{
	int i;
	unsigned char buf[14];
	unsigned char *ptr_acc, *ptr_gyro;
	int16_t  acc[3], gyro[3];
	
	if(read_n_bytes(ACCEL_XOUT_H, buf, 14) < 0)
	return -1;
	
	ptr_acc  = (unsigned char *) acc;
	ptr_gyro = (unsigned char *) gyro;
	
	for(i=0; i<6; i+=2)
	{
		ptr_acc[i]   = buf[i+1];
		ptr_acc[i+1] = buf[i];
		
		ptr_gyro[i]  = buf[i+1+8];
		ptr_gyro[i+1]= buf[i+8];
	}

	acc_f[0] = acc_scale  * ( acc[0]);
	acc_f[1] = acc_scale  * ( acc[1]);
	acc_f[2] = acc_scale  * ( acc[2]);
	
	gyro_f[0] = gyro_scale * ( gyro[0]);
	gyro_f[1] = gyro_scale * ( gyro[1]);
	gyro_f[2] = gyro_scale * ( gyro[2]);

	return 0;
}



int8_t AK8963I2CWhoAmI(void){
	int8_t me;
	write_a_byte(I2C_SLV0_ADDR, 0x0c | 0x80); //set the i2c slave address of ak8963(0x0c)
	write_a_byte(I2C_SLV0_REG, 0x00); //i2c slave 0 register address from where to begin data transfer
	write_a_byte(I2C_SLV0_CTRL, 0x81); //enable i2c and transfer 1 byte
	_delay_ms(50);
	
	if(read_n_bytes(EXT_SENS_DATA_00, (uint8_t *) &me, 1) < 0)
		return -1;
	
	return me;
}


int AK8963I2CReadMAGNETO(int16_t mag[]){
	unsigned char buf[6];
	unsigned char *ptr_mag;
	int i;
	write_a_byte(I2C_SLV0_ADDR, 0x8c); //set the i2c slave address of ak8963(0x0c) read mode
	write_a_byte(I2C_SLV0_REG, 0x03); //i2c slave 0 register address from where to begin data transfer
	write_a_byte(I2C_SLV0_CTRL, 0x87); //enable i2c and read 6 byte
	_delay_ms(50);	

	if(read_n_bytes(EXT_SENS_DATA_00, buf, 6) < 0)
		return -1;
	
	ptr_mag  = (unsigned char *) mag;
	
	for(i=0; i<6; i+=2)
	{
		ptr_mag[i]   = buf[i];
		ptr_mag[i+1] = buf[i+1];
	}

	return 0;

}
int AK8963I2CReadMAGNETO_2(int16_t mag[]){
	unsigned char buf[6];
	unsigned char *ptr_mag;
	int i;

	if(read_n_bytes(EXT_SENS_DATA_00, buf, 6) < 0)
	return -1;
	
	ptr_mag  = (unsigned char *) mag;
	
	for(i=0; i<6; i+=2)
	{
		ptr_mag[i]   = buf[i];
		ptr_mag[i+1] = buf[i+1];
	}

	return 0;

}


/*
int AK8963I2CReadMAGNETO_f(float mag[]){
	int i;
	unsigned char buf[8];
	unsigned char *ptr_mag;
	
	if(ak8963_read_n_bytes(0x02, buf, 8 < 0)
	return -1;
	
	ptr_mag  = (unsigned char *) mag;
	
	for(i=0; i<6; i+=2)
	{
		ptr_mag[i]   = buf[i+1];
		ptr_mag[i+1] = buf[i+2];
		
	}

	return 0;
}
*/




//=========================================-------===================
//	write single byte.
//
//  addr : register address
//	data : data to be written
//
//===================================================================
//
static int write_a_byte(uint8_t addr, uint8_t data)
{
	uint8_t	buffer[2];

	buffer[0] = addr;
	buffer[1] = data;
	
	return(i2c_write_n_bytes(MPU9250_I2C_ADDR, buffer, 2, 1));
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
	if(i2c_write_a_byte(MPU9250_I2C_ADDR, addr | 0x80, 0)<0)		// send addr without STOP condition
	return -1;

	return(i2c_read_n_bytes(MPU9250_I2C_ADDR, data, n));
}


