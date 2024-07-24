//===================================================
// i2c.c
//
// I2C(TWI) bus interface
//===================================================

#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>

#include "i2c.h"

//============================================
// Initialize I2C bus
//
//	input:
//		hz - data transfer rate in Hz
//============================================
void i2c_init(int32_t hz)
{
	TWSR = 0x00;
	//TWSR = 1<<TWPS0;		// 비트율 프리스케일러값을 1로 한다.

	// 비트율 설정
	TWBR = 0x0c;
	//TWBR = (uint8_t) ((F_CPU/hz - 16L)/(2L*1L));
}

#define TIME_OUT_CNT	10

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
int i2c_write_a_byte(uint8_t i2c_addr, uint8_t data, int with_stop)
{
	int time_out = TIME_OUT_CNT;
	 
	while(1)
	{
		TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);	// send START condition
		
		while(!(TWCR & (1<<TWINT))); 				// wait until START condition is sent.
		if((TWSR & TW_STATUS_MASK) != TW_START)		// check if START condition is sent normally.
			return -1;

		TWDR = (i2c_addr << 1);						// SLA_W : (i2c_addr << 1)
		TWCR = (1<<TWINT) | (1<<TWEN);				// send SLA_W.
		while(!(TWCR & (1<<TWINT)));				// wait until SLA_W is sent.
		
		if((TWSR & TW_STATUS_MASK) != TW_MT_SLA_ACK)	// check if slave returns acknowledgment
		{
			TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN); // send STOP condition.
			while(TWCR & (1<<TWSTO));					// wait until STOP condition is sent.

			time_out--;
			if(time_out <= 0) return -2;
			_delay_us(10);
		}
		else
		{
			break;
		}
	}

	//--------------------------------------------------------
	// write i-th data to slave
	//--------------------------------------------------------
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);			// send data
	while(!(TWCR & (1<<TWINT)));			// wait until data is sent.

	// check if ACK is received from slave after sending data
	if((TWSR & TW_STATUS_MASK) != TW_MT_DATA_ACK)
		return -3;
	
	//----------------------------------
	// send STOP condition
	//----------------------------------

	if(with_stop)
	{
		TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
		while((TWCR & (1<<TWSTO)));					// wait until STOP condition is sent and release bus
	}

	return 0;
}

//=========================================-------===================
//	write n bytes on I2C.
//
//  i2c_addr : I2C address.
//	buffer   : pointer to buffer
//	n		 : number of bytes to be send
//	with_stop	: 0 - do not send stop at the end, 1 - send stop at the end
//
//	return :
//		0		- success
//	   negative	- fail to write
//===================================================================
//
int i2c_write_n_bytes(uint8_t i2c_addr, uint8_t *buffer, int n, int with_stop)
{
	int i;
	int time_out = TIME_OUT_CNT;

	while(1)
	{
		TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);	// send START condition
		
		while(!(TWCR & (1<<TWINT))); 				// wait until START condition is sent.
		if((TWSR & TW_STATUS_MASK) != TW_START)		// check if START condition is sent normally.
			return -1;

		TWDR = (i2c_addr << 1);						// SLA_W : (i2c_addr << 1)				
		TWCR = (1<<TWINT) | (1<<TWEN);				// send SLA_W.
		while(!(TWCR & (1<<TWINT)));				// wait until SLA_W is sent.
		
		if((TWSR & TW_STATUS_MASK) != TW_MT_SLA_ACK)	// check if slave returns acknowledgment
		{
			TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN); // send STOP condition.
			while(TWCR & (1<<TWSTO));					// wait until STOP condition is sent.

			time_out--;
			if(time_out <= 0) return -2;
			_delay_us(10);
		}
		else
		{
			break;
		}
	}

	for(i=0; i<n; i++)
	{
		//--------------------------------------------------------
		// write i-th data to slave
		//--------------------------------------------------------
		TWDR = buffer[i];
		TWCR = (1<<TWINT) | (1<<TWEN);			// send data
		while(!(TWCR & (1<<TWINT)));			// wait until data is sent.

		// check if ACK is received from slave after sending data
		if((TWSR & TW_STATUS_MASK) != TW_MT_DATA_ACK)
			return -3;
	}
	
	//----------------------------------
	// send STOP condition
	//----------------------------------
	if(with_stop)
	{
		TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
		while((TWCR & (1<<TWSTO)));				// wait until STOP condition is sent.
	}

	return 0;
}

//===================================================================
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
int i2c_read_n_bytes(uint8_t i2c_addr, uint8_t *data, int n)
{
	int i;
	int time_out = TIME_OUT_CNT;

	while(1)
	{
		TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);	// send START or REPEATED START condition
		
		while(!(TWCR & (1<<TWINT))); 				// wait until START condition is sent.
		if(!((TWSR & TW_STATUS_MASK) == TW_START ||		// check if START condition is sent normally.
		     (TWSR & TW_STATUS_MASK) == TW_REP_START))
			return -1;

		TWDR = (i2c_addr << 1) | 0x01;				// SLA_R : (i2c_addr << 1) | 0x01
		TWCR = (1<<TWINT) | (1<<TWEN);				// send SLA_R.
		while(!(TWCR & (1<<TWINT)));				// wait until SLA_R is sent.
		
		if((TWSR & TW_STATUS_MASK) != TW_MR_SLA_ACK)
		{
			TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN); // send STOP condition.
			while(TWCR & (1<<TWSTO));					// wait until STOP condition is sent.

			time_out--;
			if(time_out <= 0) return -2;
			_delay_us(10);
		}
		else
		{
			break;
		}
	}

	//----------------------
	// read data
	//----------------------
	for(i=0; i<n-1; i++)
	{
		// send ACK after receiving data
		TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);	// if receiving data, send ACK.
		while(!(TWCR & (1<<TWINT)));				// wait until receiving data.
		
		if((TWSR & TW_STATUS_MASK) != TW_MR_DATA_ACK)
			return -6;
		
		data[i] = TWDR;								// read data.
	}

	// if read the last byte, send NACK.

	TWCR = (1<<TWINT) | (1<<TWEN);					// after receiving data, do end NACK
	while(!(TWCR & (1<<TWINT)));					// wait until receiving data.
	
	if((TWSR & TW_STATUS_MASK) != TW_MR_DATA_NACK)
		return -7;
	
	data[i] = TWDR;									// read the last data

	//-----------------------------
	// send STOP condition.
	//-----------------------------

	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
	while(TWCR & (1<<TWSTO));						// wait until STOP condition is sent
	
	return 0;
}