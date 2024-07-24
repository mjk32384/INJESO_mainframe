/*
 * UART1.c
 *
 * Created: 2023-07-27 오후 4:10:44
 *  Author: PC
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include "UART1.h"

void UART1_init(void)
{
	UBRR1H = 0x00;                     //9,600 보율로 설정
	UBRR1L = 16;
	UCSR1A |= _BV(U2X1);               //2배속 모드
	// 비동기, 8비트 데이터, 패리티 없음, 1비트 정지 비트 모드
	UCSR1C |= 0x06;
	
	UCSR1B |= _BV(RXEN1);               //송수신 가능
	UCSR1B |= _BV(TXEN1);
}

void UART1_transmit(char data)
{
	while( !(UCSR1A & (1 << UDRE1)) );       //송신 가능 대기
	UDR1 = data;                     //데이터 전송
}

unsigned char UART1_receive(void)
{
	while( !(UCSR1A & (1<<RXC1)) );         //데이터 수신 대기
	return UDR1;
}

void UART1_print16b(int16_t no)
{
	char numStr[6]="0";
	int mino=0;
	int index=0;
	
	int i=0;
	if(no>0)
	{
		for(i=0;no!=0;i++)
		{
			numStr[i]=no%10+48;
			no=no/10;
			
		}
		numStr[i]='\0';
		index=i-1;
	}if(no<0)
	{
		mino=0-no;
		for(i=0;mino!=0;i++)
		{
			numStr[i]=mino%10+48;
			mino=mino/10;
		}
		numStr[i]='\0';
		index=i-1;
		UART1_transmit('-');
	}
	for(int j=index;j>=0;j--)
	{
		UART1_transmit(numStr[j]);
	}
	return;
}

