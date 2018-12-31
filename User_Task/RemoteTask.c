#include "RemoteTask.h"
#include "stdio.h"		//sprintf
#include "LCD.h"		//LCD_ShowString()

RC_Ctl_t RC_Ctl;

#define STOP   2	//��ң����S2����ʱֹͣ

/*------���ղ�����ң��������------*/

short int Get_Mode_Data(void)	//����ģʽ����
{
	return RC_Ctl.rc.s2;
}


short int Get_ch0_Data(void)	//ͨ�����ݷ���
{
	return RC_Ctl.rc.ch0;
}


short int Get_ch1_Data(void)
{
	return RC_Ctl.rc.ch1;
}


short int Get_ch2_Data(void)
{
	return RC_Ctl.rc.ch2;
}


short int Get_ch3_Data(void)
{
	return RC_Ctl.rc.ch3;
}


short int Get_Mouse_X_Data(void)
{
	return RC_Ctl.mouse.x;
}


short int Get_Mouse_Y_Data(void)
{
	return RC_Ctl.mouse.y;
}
	

void RC_Ctl_t_Init(void)	//��ң�����������г�ʼ��
{
	RC_Ctl.rc.ch0 = 1024;		//ch0��ch1Ϊ��ҡ��
	RC_Ctl.rc.ch1 = 1024;	
	RC_Ctl.rc.ch2 = 1024;		//ch2��ch3Ϊ��ҡ��
	RC_Ctl.rc.ch3 = 1024;
	RC_Ctl.rc.s1 = 0;			//�����Ƹ�
	RC_Ctl.rc.s2 = STOP;		//�����Ƹ�		S2ѡ�����ģʽ
	RC_Ctl.mouse.press_l = 0;		//������
	RC_Ctl.mouse.press_r = 0;		//����Ҽ�
	RC_Ctl.mouse.last_press_l = 0;	
	RC_Ctl.mouse.last_press_r = 0;
	RC_Ctl.mouse.x = 0;	//�����X����ƶ��ٶ�
	RC_Ctl.mouse.y = 0;	//�����Y����ƶ��ٶ�
	RC_Ctl.mouse.z = 0;	//�����Z����ƶ��ٶ�
	RC_Ctl.key.v = 0;
}

void RemoteDataProcess(volatile unsigned char *sbus_rx_buffer)
{
	RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff; //!< Channel 0
	RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1
	RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | //!< Channel 2
	(sbus_rx_buffer[4] << 10)) & 0x07ff;
	RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
	RC_Ctl.rc.s1 = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2; //!< Switch left
	RC_Ctl.rc.s2 = ((sbus_rx_buffer[5] >> 4)& 0x0003); //!< Switch right9 / 9
	RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8); //!< Mouse X axis
	RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8); //!< Mouse Y axis
	RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8); //!< Mouse Z axis
	RC_Ctl.mouse.press_l = sbus_rx_buffer[12]; //!< Mouse Left Is Press ?
	RC_Ctl.mouse.press_r = sbus_rx_buffer[13]; //!< Mouse Right Is Press ?
	RC_Ctl.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8); //!< KeyBoard value
}






