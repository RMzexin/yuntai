#include "RemoteTask.h"
#include "stdio.h"		//sprintf
#include "LCD.h"		//LCD_ShowString()

RC_Ctl_t RC_Ctl;

#define STOP   2	//当遥控器S2在下时停止

/*------接收并处理遥控器数据------*/

short int Get_Mode_Data(void)	//控制模式反馈
{
	return RC_Ctl.rc.s2;
}


short int Get_ch0_Data(void)	//通道数据反馈
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
	

void RC_Ctl_t_Init(void)	//对遥控器按键进行初始化
{
	RC_Ctl.rc.ch0 = 1024;		//ch0和ch1为右摇杆
	RC_Ctl.rc.ch1 = 1024;	
	RC_Ctl.rc.ch2 = 1024;		//ch2和ch3为左摇杆
	RC_Ctl.rc.ch3 = 1024;
	RC_Ctl.rc.s1 = 0;			//左上推杆
	RC_Ctl.rc.s2 = STOP;		//右上推杆		S2选择控制模式
	RC_Ctl.mouse.press_l = 0;		//鼠标左键
	RC_Ctl.mouse.press_r = 0;		//鼠标右键
	RC_Ctl.mouse.last_press_l = 0;	
	RC_Ctl.mouse.last_press_r = 0;
	RC_Ctl.mouse.x = 0;	//鼠标在X轴的移动速度
	RC_Ctl.mouse.y = 0;	//鼠标在Y轴的移动速度
	RC_Ctl.mouse.z = 0;	//鼠标在Z轴的移动速度
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






