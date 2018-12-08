#ifndef __REMOTE_TASK
#define __REMOTE_TASK

#include "stm32f4xx.h"

typedef __packed struct		//Ò£¿ØÆ÷
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
}Remote;

typedef __packed struct		//Êó±ê
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;	

typedef	__packed struct		//¼üÅÌ
{
	uint16_t v;
}Key;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctl_t;

short int Get_Mode_Data(void);
short int Get_ch0_Data(void);
short int Get_ch1_Data(void);
short int Get_ch2_Data(void);
short int Get_ch3_Data(void);
short int Get_Mouse_X_Data(void);
short int Get_Mouse_Y_Data(void);
void RC_Ctl_t_Init(void);
void Print_RC_Data(void);
void RemoteDataProcess(volatile unsigned char *sbus_rx_buffer);


#endif



