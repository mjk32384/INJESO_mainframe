/*
 * UART1.h
 *
 * Created: 2023-07-27 오후 4:11:55
 *  Author: PC
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <string.h>

#ifndef UART1_H_
#define UART1_H_


void UART1_init(void);
void UART1_transmit(char data);
unsigned char UART1_receive(void);
void UART1_print16b(int16_t no);


#endif /* UART1_H_ */