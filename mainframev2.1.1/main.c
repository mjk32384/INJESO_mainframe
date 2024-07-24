/*
 * mainframev2.1.1.c
 *
 * Created: 2023-09-08 오후 8:27:17
 * Author : 정우진 wj Jeong
 */ 
//---------------------------------------------------
// MPU9250 센서의 방향이 다른 점 유의하세요. 


//---------------------------------------------------


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include <avr/eeprom.h>
#include "mpu9250_i2c_v2.h"
#include "Quaternion.h"
#include "UART1.h"
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
//float avgxx = 0.02525;//1번
//float avgyy = -0.06337;
//float avgzz = -0.00780;
float avgxx = -0.01707;//2번
float avgyy = -0.00570;
float avgzz = -0.00997;
float yaw = 0;
float pitch = 0;
float roll = 0;
float roll0 =0;
float roll1 =0;
float thetaz = 0;

float accelxx = 0;
float accelyy = 0;
float accelzz = 0;
//float avgax = 0.67278;//1번
//float avgay = 0.60033;
//float avgaz = -1.8743;
float avgax = 0.67278;//2번 보드
float avgay = 0.60033;
float avgaz = -1.8743;
float magxx = 0;
float magyy = 0;
float magzz = 0;
float avgmx = 0;
float avgmy = 0;
float avgmz = 0;
float magsumx =0;
float magsumy =0;
float magsumz =0;
float magmeanx=0;
float magmeany=0;
float magmeanz=0;
float acc2_f[3], gyro2_f[3];
int16_t mag[3];

float alpha = 0.66; //filter 
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

Quaternion orientation;
Quaternion delta;
Quaternion omega;
Quaternion q1;
Quaternion q2;
Quaternion orientation2;

double eulang[3];
double orient_temp[3];
double orient_temp2[3];
double orient_x_init[3] = {1, 0, 0};
double orient_x[3];
double orient_y_init[3] = {0, 1, 0};
double orient_y[3];
double orient_z_init[3] = {0, 0, 1};
double orient_z[3];

uint16_t v0 = 0;
uint16_t v1 = 0;
uint16_t v2 = 0;
int k = 0;

ISR(TIMER0_OVF_vect) 				// 타이머0 오버플로 인터럽트 서비스루틴
{

	TCNT0 = 6;	//131				// 16msec 후에 인터럽트발생
	n_enter++;						// 인터럽트 횟수 증가
	MPU9250I2CReadIMU_f(acc2_f,gyro2_f);
	gyroxx=-(gyro2_f[0]-avgxx);		//orient change1
	gyroyy=-(gyro2_f[1]-avgyy);
	gyrozz=-(gyro2_f[2]-avgzz);
	
	//filter
	f_gx_now=gyroxx-alpha*(gyroxx-f_gx_last);
	f_gy_now=gyroyy-alpha*(gyroyy-f_gy_last);
	f_gz_now=gyrozz-alpha*(gyrozz-f_gz_last);
	Quaternion_set(0, -f_gx_now, -f_gy_now, -f_gz_now, &omega);
	Quaternion_multiply(&orientation, &omega, &delta);
	Quaternion_ratio(&delta, 0.5 * 16 / 1000, &delta);
	Quaternion_addition(&orientation, &delta, &orientation);
	Quaternion_normalize(&orientation, &orientation);
	Quaternion_toEulerZYX2(&orientation,eulang);
	f_gx_last=f_gx_now;
	f_gy_last=f_gy_now;
	f_gz_last=f_gz_now;
	
	//orient not change	
	accelxx = acc2_f[0]-avgax;
	accelyy = acc2_f[1]-avgay;
	accelzz = acc2_f[2]-avgaz;	
	//filter	
	f_ax_now=accelxx-alpha*(accelxx-f_ax_last);
	f_ay_now=accelyy-alpha*(accelyy-f_ay_last);
	f_az_now=accelzz-alpha*(accelzz-f_az_last);
	f_ax_last=f_ax_now;
	f_ay_last=f_ay_now;
	f_az_last=f_az_now;
	
	AK8963I2CReadMAGNETO_2(mag);
	magyy = mag[0]-avgmy;			//orient change2
	magxx = mag[1]-avgmx;			//orient change2
	magzz =-(mag[2]-avgmz);
	orient_temp[0]=magxx;
	orient_temp[1]=magyy;
	orient_temp[2]=magzz;
	yaw=atan2(-f_ax_now,+f_ay_now)*57.296 + eulang[2]*57.295;
	pitch=atan2(sqrt(f_ax_now*f_ax_now+f_ay_now*f_ay_now),f_az_now)*57.295;
	roll= roll0 - (atan2(magyy,magxx)* 57.295);
	
	Quaternion_set(cos(pitch/2.0/57.295), sin(pitch/2.0/57.295)*cos(atan2(-f_ax_now,+f_ay_now)), sin(pitch/2.0/57.295)*sin(atan2(-f_ax_now,+f_ay_now)), 0, &q1);
	Quaternion_rotate(&q1, orient_temp, orient_temp2);
	roll1=roll0/57.295 - atan2(orient_temp2[1],orient_temp2[0]);//개선 필요. ss-cc/cs+sc
	Quaternion_set(cos(roll1/2.0), 0, 0, sin(roll1/2.0), &q2);
	Quaternion_multiply(&q2, &q1, &orientation2);
	Quaternion_normalize(&orientation2, &orientation2);
	if(n_enter>=25){
			

		v0 = f_gz_last * 10000;
		v1 = orientation.v[1] * 10000;
		v2 = orientation.v[2] * 10000;
		
		
		if(k<250)
		{
		UART1_transmit('\t');
		UART1_print16b((uint16_t) (orientation.w * 10000));
		UART1_transmit('\t');
		UART1_print16b((uint16_t) v0);
		UART1_transmit('\t');
		UART1_print16b((uint16_t) v1);
		UART1_transmit('\t');
		UART1_print16b((uint16_t) v2);
		UART1_transmit('\t');
		UART1_transmit('\t');
		eeprom_write_word( 6*k, v0>>8);
	    eeprom_write_word( 6*k+1, v0&0xff);
		eeprom_write_word( 6*k+2, v1>>8);
		eeprom_write_word( 6*k+3, v1&0xff);
		eeprom_write_word( 6*k+4, v2>>8);
		eeprom_write_word( 6*k+5, v2&0xff);
		k++;
		}
	    _delay_ms(10);
		Quaternion_toEulerZYX2(&orientation2,eulang);
		
		
		UART1_print16b((uint16_t) (f_ax_now*10000));
 		UART1_transmit('\t');
		UART1_print16b((uint16_t) (f_ay_now*10000));
 		UART1_transmit('\t');		
		UART1_print16b((uint16_t) (f_az_now*10000));
 		UART1_transmit('\t');		
		UART1_print16b((uint16_t) magxx);
		UART1_transmit('\t');
		UART1_print16b((uint16_t) magyy);
		UART1_transmit('\t');
		UART1_print16b((uint16_t) magzz);
		UART1_transmit('\t');	
		UART1_transmit('\n');			 
/*
 		if(eulang[1]<0.1) UART1_print16b(0);
 		else UART1_print16b((uint16_t) (eulang[0] * 180/3.141592));
 		UART1_transmit('\t');
 		UART1_print16b((uint16_t) (eulang[1] * 180/3.141592));
 		UART1_transmit('\t');
 		UART1_print16b((uint16_t) (eulang[2] * 180/3.141592));
 		UART1_transmit('\t');		
		UART1_print16b((uint16_t) (yaw));
		UART1_transmit('\t');
 		UART1_print16b((uint16_t) (pitch));	
		UART1_transmit('\t');
    	UART1_print16b((uint16_t) (roll));					
		UART1_transmit('\n');
*/
		OCR2 += f_gz_now * 100;

		n_enter=0;

	}
}

ISR(TIMER2_OVF_vect) 
{
	TCNT2 = 6;
}

int main(void)
{

	_delay_ms(1000);
	UART1_init();
//	UART1_transmit('A');
	MPU9250I2CInit(400000);
			
	_delay_ms(1000);		
	
	TCCR0 = 0x00;
	TCNT0 = 6;//131
	TCNT2 = 6;					        // 타이머 초기 값 설정
	
	TCCR2 = 0x68;				        // 표준모드, 타이머 정지
	OCR2 = 00;
	DDRB |= (1<<DDB7);// 인터럽트 설정
	TIMSK = (1<<TOIE0); 	// 타이머0 오버플로 인터럽트 허용
	TCCR0 |= 0x07;
	TCCR2 |= 0x03;						//1ms(16000tic) , duty 64 ?32

    int i = 0;
	MPU9250I2CReadIMU_f(acc2_f,gyro2_f);
	AK8963I2CReadMAGNETO(mag);
	UART1_print16b(mag[0]);
	UART1_transmit('\t');
	UART1_print16b(mag[1]);
	UART1_transmit('\t');
	UART1_print16b(mag[2]);
	UART1_transmit('\n');
	_delay_ms(10);

	for( i = 0 ; i<100 ; i++ )
	{
		
		MPU9250I2CReadIMU_f(acc2_f,gyro2_f);
		gyrosumxx += gyro2_f[0];
		gyrosumyy += gyro2_f[1];
		gyrosumzz += gyro2_f[2];
		AK8963I2CReadMAGNETO_2(mag);
		magsumx	+= mag[0]-avgmx;
		magsumy	+= mag[1]-avgmy;
		magsumz	+= mag[2]-avgmz;
		_delay_ms(10);
		
	}
    avgxx = avgxx* 0.5 + 0.5* gyrosumxx / 1000.0;
    avgyy = avgyy* 0.5 + 0.5* gyrosumyy / 1000.0;
    avgzz = avgzz* 0.7 + 0.3* gyrosumzz / 1000.0 ;
	magmeanx = magsumx/100.0; // x and y have to be inversed 
	magmeany = magsumy/100.0; //
	magmeanz = magsumz/100.0;
	roll0 = atan2(magmeanx,magmeany)*57.295; //so inverse
	UART1_print16b((int16_t)(avgxx*100000));	
	UART1_transmit('\t');
	UART1_print16b((int16_t)(avgyy*100000));
	UART1_transmit('\t');
	UART1_print16b((int16_t)(avgzz*100000));
	UART1_transmit('\t');
	UART1_transmit('\t');	
	UART1_print16b((int16_t)roll0);
	UART1_transmit('\t');
	UART1_transmit('\n');
	
	_delay_ms(1000);
	Quaternion_setIdentity(&orientation); 
	sei();							    // 전역 인터럽트 허용

	UART1_transmit('\n');
	_delay_ms(400);	
	
	//for(int j = 0 ; j<25 ; j++){
		//OCR2 = 10*j + 10;
		//_delay_ms(1000);
		//
	//}
	OCR2 = 100;
	_delay_ms(10000);
}

