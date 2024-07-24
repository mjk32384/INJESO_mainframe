﻿/*
 * mpu9250_i2c_v2.h
 *
 * Created: 2023-06-28 오후 11:02:11
 *  Author: stu11
 * Use	  : mpu9250 + ak8963 master mode code
 */ 

#ifndef __MPU9250_I2C_v2_H__
#define __MPU9250_I2C_v2_H__

#include <avr/io.h>

//-------------------------------
// MPU9250 address of I2C
//-------------------------------
#define MPU9250_I2C_ADDR	0x68		// if AD0 = 0
#define AK8963_I2C_ADDR		0x0C
//#define MPU9250_I2C_ADDR	0x69		// if AD0 = 1

//-------------------------------
// Accelerometer Range
//-------------------------------
//
#define ACC_RANGE_2G		0		// +/- 2g range
#define ACC_RANGE_4G		1		// +/- 4g range
#define ACC_RANGE_8G		2		// +/- 8g range
#define ACC_RANGE_16G		3		// +/- 16g range

//-------------------------------
// Gyro Range
//-------------------------------
#define GYRO_RANGE_250		0		// +/- 250  deg/sec range
#define GYRO_RANGE_500		1		// +/- 500  deg/sec range
#define GYRO_RANGE_1000		2		// +/- 1000 deg/sec range
#define GYRO_RANGE_2000		3		// +/- 2000 deg/sec range

//-------------------------------
// LPF bandwidth
//-------------------------------
#define LPF_5Hz				6		// 5Hz   bandwidth
#define LPF_10Hz			5		// 10Hz  bandwidth
#define LPF_20Hz			4		// 20Hz  bandwidth
#define LPF_40Hz			3		// 40Hz  bandwidth
#define LPF_100Hz			2		// 100Hz bandwidth
#define LPF_180Hz			1		// 180Hz bandwidth
#define LPF_260Hz			0		// 260Hz bandwidth

//-------------------------------
// Register Address
//-------------------------------

#define SELF_TEST_X			0x0D
#define SELF_TEST_Y			0x0E
#define SELF_TEST_Z			0x0F
#define SELF_TEST_A			0x10
#define SMPLRT_DIV			0x19
#define CONFIG				0x1A
#define GYRO_CONFIG			0x1B
#define ACCEL_CONFIG		0x1C
#define FF_THR				0x1D
#define FF_DUR				0x1E
#define MOT_THR				0x1F
#define MOT_DUR				0x20
#define ZRMOT_THR			0x21
#define ZRMOT_DUR			0x22
#define FIFO_EN				0x23
#define I2C_MST_CTRL		0x24
#define I2C_SLV0_ADDR		0x25
#define I2C_SLV0_REG		0x26
#define I2C_SLV0_CTRL		0x27
#define I2C_SLV1_ADDR		0x28
#define I2C_SLV1_REG		0x29
#define I2C_SLV1_CTRL		0x2A
#define I2C_SLV2_ADDR		0x2B
#define I2C_SLV2_REG		0x2C
#define I2C_SLV2_CTRL		0x2D
#define I2C_SLV3_ADDR		0x2E
#define I2C_SLV3_REG		0x2F
#define I2C_SLV3_CTRL		0x30
#define I2C_SLV4_ADDR		0x31
#define I2C_SLV4_REG		0x32
#define I2C_SLV4_DO			0x33
#define I2C_SLV4_CTRL		0x34
#define I2C_SLV4_DI			0x35
#define I2C_MST_STATUS		0x36
#define INT_PIN_CFG			0x37
#define INT_ENABLE			0x38
#define INT_STATUS			0x3A
#define ACCEL_XOUT_H		0x3B
#define ACCEL_XOUT_L		0x3C
#define ACCEL_YOUT_H		0x3D
#define ACCEL_YOUT_L		0x3E
#define ACCEL_ZOUT_H		0x3F
#define ACCEL_ZOUT_L		0x40
#define TEMP_OUT_H			0x41
#define TEMP_OUT_L			0x42
#define GYRO_XOUT_H			0x43
#define GYRO_XOUT_L			0x44
#define GYRO_YOUT_H			0x45
#define GYRO_YOUT_L			0x46
#define GYRO_ZOUT_H			0x47
#define GYRO_ZOUT_L			0x48
#define EXT_SENS_DATA_00	0x49
#define EXT_SENS_DATA_01	0x4A
#define EXT_SENS_DATA_02	0x4B
#define EXT_SENS_DATA_03	0x4C
#define EXT_SENS_DATA_04	0x4D
#define EXT_SENS_DATA_05	0x4E
#define EXT_SENS_DATA_06	0x4F
#define EXT_SENS_DATA_07	0x50
#define EXT_SENS_DATA_08	0x51
#define EXT_SENS_DATA_09	0x52
#define EXT_SENS_DATA_10	0x53
#define EXT_SENS_DATA_11	0x54
#define EXT_SENS_DATA_12	0x55
#define EXT_SENS_DATA_13	0x56
#define EXT_SENS_DATA_14	0x57
#define EXT_SENS_DATA_15	0x58
#define EXT_SENS_DATA_16	0x59
#define EXT_SENS_DATA_17	0x5A
#define EXT_SENS_DATA_18	0x5B
#define EXT_SENS_DATA_19	0x5C
#define EXT_SENS_DATA_20	0x5D
#define EXT_SENS_DATA_21	0x5E
#define EXT_SENS_DATA_22	0x5F
#define EXT_SENS_DATA_23	0x60
#define MOT_DETECT_STATUS	0x61
#define I2C_SLV0_DO			0x63
#define I2C_SLV1_DO			0x64
#define I2C_SLV2_DO			0x65
#define I2C_SLV3_DO			0x66
#define I2C_MST_DELAY_CTRL	0x67
#define SIGNAL_PATH_RESET	0x68
#define MOT_DETECT_CTRL		0x69
#define USER_CTRL			0x6A
#define PWR_MGMT_1			0x6B
#define PWR_MGMT_2			0x6C
#define FIFO_COUNTH			0x72
#define FIFO_COUNTL			0x73
#define FIFO_R_W			0x74
#define WHO_AM_I			0x75

int MPU9250I2CInit(int32_t spi_clk);
int8_t MPU9250I2CWhoAmI(void);
int MPU9250I2CSetAccRange(uint8_t range);
int MPU9250I2CSetGyroRange(uint8_t range);
int MPU9250I2CReadIMU(int16_t acc[], int16_t gyro[]);
int MPU9250I2CReadIMU_f(float acc_f[], float gyro_f[]);

int8_t AK8963I2CWhoAmI(void);
int AK8963I2CReadMAGNETO(int16_t mag[]);
int AK8963I2CReadSTATUS(unsigned char status[]);

#endif	