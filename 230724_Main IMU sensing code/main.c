/*
 * 230724_Main IMU sensing code.c
 *
 * Created: 2023-07-24 오후 9:44:19
 * Author : 정우진 wj Jeong
 */ 



//---------------------------------------------------
// 
// 

// MPU9250
// timer 이용한 제어
// lpf 와 회전 제어 

//---------------------------------------------------


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "mpu9250_i2c_v2.h"

//손수 제작 uart1 코드
void UART1_init(void);
void UART1_transmit(char data);
unsigned char UART1_receive(void);
void UART1_print16b(int16_t no);
int n_enter=0;		// 인터럽트 횟수(전역변수)
int n_enter2=0;
int index1 = 0;
float gyroxx = 0; // mpu9250는 2번 센서로 값은 xx, 행렬은 gyro2[]
float gyroyy = 0;
float gyrozz = 0;
float gyrosumxx=0;
float gyrosumyy=0;
float gyrosumzz=0;
float avgxx = 0.025;
float avgyy = -0.063;
float avgzz = 0.008;
float thetax = 0;
float thetay = 0;
float thetaz = 0;

float accelxx = 0;
float accelyy = 0;
float accelzz = 0;
float avgax = 0.67278;
float avgay = 0.60033;
float avgaz = -1.8743;

float acc2_f[3], gyro2_f[3];
int16_t mag[3];

float alpha = 0.666; //filter 
float f_gx_last =0;
float f_gy_last =0;
float f_gz_last =0;
float f_gx_now  =0;
float f_gy_now	=0;
float f_gz_now  =0;

float f_ax_last =0;
float f_ay_last =0;
float f_az_last =0;
float f_ax_now =0;
float f_ay_now =0;
float f_az_now =0;


ISR(TIMER0_OVF_vect) 				// 타이머0 오버플로 인터럽트 서비스루틴
{

	TCNT0 = 6;					    // 16msec 후에 인터럽트발생
	n_enter++;						// 인터럽트 횟수 증가
	MPU9250I2CReadIMU_f(acc2_f,gyro2_f);
	gyroxx=-(gyro2_f[0]-avgxx);
	gyroyy=-(gyro2_f[1]-avgyy);
	gyrozz=-(gyro2_f[2]-avgzz);
	
	f_gx_now=gyroxx-alpha*(gyroxx-f_gx_last);
	f_gy_now=gyroyy-alpha*(gyroyy-f_gy_last);
	f_gz_now=gyrozz-alpha*(gyrozz-f_gz_last);
	f_gx_last=f_gx_now;
	f_gy_last=f_gy_now;
	f_gz_last=f_gz_now;	
	
	
	/*			
	UART1_print16b((int16_t)(gyroxx*1000));
	UART1_transmit('\t');
	UART1_print16b((int16_t)(gyroyy*1000));
	UART1_transmit('\t');
	UART1_print16b((int16_t)(gyrozz*1000));
	UART1_transmit('\t');
	UART1_transmit('\t');	
	UART1_print16b((int16_t)(f_gx_now*1000));
	UART1_transmit('\t');
	UART1_print16b((int16_t)(f_gy_now*1000));
	UART1_transmit('\t');
	UART1_print16b((int16_t)(f_gz_now*1000));
	UART1_transmit('\n');	*/
	
	accelxx = acc2_f[0]-avgax;
	accelyy = acc2_f[1]-avgay;
	accelzz = acc2_f[2]-avgaz;		
	f_ax_now=accelxx-alpha*(accelxx-f_ax_last);
	f_ay_now=accelyy-alpha*(accelyy-f_ay_last);
	f_az_now=accelzz-alpha*(accelzz-f_az_last);
	f_ax_last=f_ax_now;
	f_ay_last=f_ay_now;
	f_az_last=f_az_now;
	
	if(n_enter>=25){
		UART1_print16b((int16_t)(f_ax_now*1000));
		UART1_transmit('\t');
		UART1_print16b((int16_t)(f_ay_now*1000));
		UART1_transmit('\t');
		UART1_print16b((int16_t)(f_az_now*1000));
		UART1_transmit('\n');	
		n_enter=0;
	}
//	AK8963I2CReadMAGNETO(mag);
//	UART1_print16b(mag[0]);
//	UART1_transmit('\t');
//	UART1_print16b(mag[1]);
//	UART1_transmit('\t');
//	UART1_print16b(mag[2]);
//	UART1_transmit('\t');
//	UART1_print16b((int16_t)n_enter2);
//	UART1_transmit('\n');
	
	 
}
ISR(TIMER2_OVF_vect)
{

}


int main(void)
{

	_delay_ms(1000);
	UART1_init();
	MPU9250I2CInit(400000);		
	_delay_ms(1000);		
	
	TCCR0 = 0x00;
	//TCCR2 = 0x68;				        // 표준모드, 타이머 정지
	TCCR2 = 0x00;						// 함수 소요시간 계산
	TCNT0 = 6;
	TCNT2 = 6;					        // 타이머 초기 값 설정
	//OCR2 = 250;
	
	//DDRB |= (1<<DDB7);
	// 인터럽트 설정
	TIMSK = (1<<TOIE0) || (1<<TOIE2);	// 타이머0,2 오버플로 인터럽트 허용

    int i = 0;

	for( i = 0 ; i<2050 ; i++ )
	{
		
		MPU9250I2CReadIMU_f(acc2_f,gyro2_f);
		gyrosumxx += gyro2_f[0];
		gyrosumyy += gyro2_f[1];
		gyrosumzz += gyro2_f[2];
		_delay_ms(10);
		
	}
    avgxx = gyrosumxx / 2050.0;
    avgyy = gyrosumyy / 2050.0;
    avgzz = gyrosumzz / 2050.0;
	UART1_print16b((int16_t)(avgxx*1000));	
	UART1_transmit('\t');
	UART1_print16b((int16_t)(avgyy*1000));
	UART1_transmit('\t');
	UART1_print16b((int16_t)(avgzz*1000));
	UART1_transmit('\n');	
	UART1_transmit('\n');
	AK8963I2CReadMAGNETO(mag);
	UART1_print16b(mag[0]);
	UART1_transmit('\t');
	UART1_print16b(mag[1]);
	UART1_transmit('\t');
	UART1_print16b(mag[2]);
	UART1_transmit('\t');		
	_delay_ms(250);
	TCCR0 |= 0x07;
	TCCR2 |= 0x03;						//1ms(16000tic) , duty 64
	sei();							    // 전역 인터럽트 허용

	UART1_print16b((int16_t)n_enter);
	UART1_transmit('\n');	
	AK8963I2CReadMAGNETO(mag);
	UART1_print16b(mag[0]);
	UART1_transmit('\t');
	UART1_print16b(mag[1]);
	UART1_transmit('\t');
	UART1_print16b(mag[2]);
	UART1_transmit('\t');
	UART1_print16b((int16_t)n_enter);
	UART1_transmit('\n');	
	while(1);
}

void UART1_init(void)
{
	UBRR1H = 0x00;                     //9,600 보율로 설정
	UBRR1L = 207;
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
