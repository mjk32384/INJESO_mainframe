//===================================================
// i2c.c
//
// I2C(TWI) bus interface
//===================================================

#ifndef __I2C_H__
#define __I2C_H__

//============================================
// Initialize I2C bus
//
//	input:
//		hz - data transfer rate in Hz
//============================================
//
void i2c_init(int32_t hz);

//=========================================-------===================
//	write a byte on I2C.
//
//  i2c_addr	: I2C address.
//	data		: data to write
//	with_stop	: 0 - do not send stop at the end, 1 - send stop at the end
//
//	return :
//		0		- success
//	   negative	- fail to write
//===================================================================
//
int i2c_write_a_byte(uint8_t i2c_addr, uint8_t buffer, int with_stop);

//=========================================-------===================
//	write n bytes on I2C.
//
//  i2c_addr : I2C address.
//	buffer   : pointer to buffer
//	n		 : number of bytes to be send
//
//	return :
//		0		- success
//	   negative	- fail to write
//===================================================================
//
int i2c_write_n_bytes(uint8_t i2c_addr, uint8_t *buffer, int n, int with_stop);

//====================================================================
//	read n bytes on I2C.
//
//  i2c_addr : I2C address.
//	buffer   : pointer to buffer which stores data read
//	n		 : number of bytes to be read
//
//	return :
//		0		- success
//	   negative	- fail to write
//===================================================================
//
int i2c_read_n_bytes(uint8_t i2c_addr, uint8_t *data, int n);

#endif /* __I2C_H__ */