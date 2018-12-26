#include "sprintf.h"
#include "RemoteTask.h"
#include "PIDcontrolTask.h"

unsigned char DataScope_OutPut_Buffer[42] = {0};

extern RC_Ctl_t RC_Ctl;

extern Gimbal_Ref_t gimbal_ref;				//云台数据
extern mpu9250_t mpu9250;

extern pid_t CM1pid;
extern pid_t CM2pid;
extern pid_t CM3pid;
extern pid_t CM4pid;
extern pid_t PLUCKSpid;
extern pid_t PLUCKPpid;

extern PID_value_t PID_pitchPosition_value;  //pitch位置环理想值 实际值
extern PID_value_t PID_yawPosition_value;    //yaw位置环理想值 实际值
extern PID_value_t PID_pluckPosition_value;    //拨弹位置环理想值 实际值
extern PID_value_t PID_pitchSpeed_value;     //pitch速度环理想值 实际值
extern PID_value_t PID_yawSpeed_value;       //yaw速度环理想值 实际值
extern PID_value_t PID_pluckSpeed_value;       //拨弹速度环理想值 实际值
extern PID_value_t PID_CM1_value;
extern PID_value_t PID_CM2_value;
extern PID_value_t PID_CM3_value;
extern PID_value_t PID_CM4_value;


extern PID_Angle_Speed_t PID_Angle_Speed;
extern PID_Angle_Speed_t PID_Angle_Speed;



void Print_RC_Data(void)
{
	char str[100];
		
		sprintf(str, "RC_Ctl.rc.ch0: %5d", RC_Ctl.rc.ch0);
		LCD_ShowString(30,150, 200, 16, 16, (u8 *)str);	
		sprintf(str, "RC_Ctl.rc.ch1: %5d", RC_Ctl.rc.ch1);
		LCD_ShowString(30,170, 200, 16, 16, (u8 *)str);
		sprintf(str, "RC_Ctl.rc.ch2: %5d", RC_Ctl.rc.ch2);
		LCD_ShowString(30,190, 200, 16, 16, (u8 *)str);
		sprintf(str, "RC_Ctl.rc.ch3: %5d", RC_Ctl.rc.ch3);
		LCD_ShowString(30,210, 200, 16, 16, (u8 *)str);
}

void Print_PID_Data(void)
{                                                                             
//	char str[100];

	  OS_ERR err;
	
    unsigned char i;         
    unsigned char Send_Count=5;

      DataScope_Get_Channel_Data(PID_yawPosition_value.ideal, 1 );
			DataScope_Get_Channel_Data(GMYawEncoder.ecd_angle, 2 );
			DataScope_Get_Channel_Data(PID_pluckSpeed_value.ideal, 3 );
			DataScope_Get_Channel_Data(PID_pluckSpeed_value.actual, 4 );
			DataScope_Get_Channel_Data(PLUCKSpid.pidout, 5 );
//			DataScope_Get_Channel_Data(YAWSpid.pidout, 6 );
//			DataScope_Get_Channel_Data(CM3pid.Voltage, 7 );
//			DataScope_Get_Channel_Data(CM4pid.Voltage, 8 );
//			DataScope_Get_Channel_Data(PID_CM1_out, 9 );
//			DataScope_Get_Channel_Data(PID_CM2_out, 10 );
          Send_Count = DataScope_Data_Generate(5); 
          for( i = 0 ; i < Send_Count; i++) 
          {
             while((USART3->SR&0X40)==0);  
            USART3->DR = DataScope_OutPut_Buffer[i]; 
          }
					OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_PERIODIC,&err); 
//     printf("A%6.3f\r\nB%6.3f\r\nC%6.3f\r\nD%6.3f\r\n",(float)GMYawEncoder.ecd_angle,(float)mpu9250.stcGyroZ,(float)PID_yawSpeed_value.actual,(float)PID_yawSpeed_value.ideal);
//    OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_PERIODIC,&err); 
//		sprintf(str, "PID_CM1_out: %6.3f",PID_CM1_out);
//		LCD_ShowString(30, 50,500,30, 16, (u8 *)str);	
//		sprintf(str, "CM1PID.D_Voltage: %6.3f",CM1PID.D_Voltage);
//		LCD_ShowString(30, 70,500,30, 16, (u8 *)str);	
//		sprintf(str, "PID_CM1_out: %6.3f",PID_CM1_out);
//		LCD_ShowString(30, 90,500,30, 16, (u8 *)str);	
//		sprintf(str, "PID_CM1_out: %6.3f",PID_CM1_out);
//		LCD_ShowString(30, 110,500,30, 16, (u8 *)str);	
}

void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //??float???
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}

void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
	if ( (Channel > 10) || (Channel == 0) ) return; 
  else
  {
     switch (Channel)
		{
      case 1:  Float2Byte(&Data,DataScope_OutPut_Buffer,1); break;
      case 2:  Float2Byte(&Data,DataScope_OutPut_Buffer,5); break;
		  case 3:  Float2Byte(&Data,DataScope_OutPut_Buffer,9); break;
		  case 4:  Float2Byte(&Data,DataScope_OutPut_Buffer,13); break;
		  case 5:  Float2Byte(&Data,DataScope_OutPut_Buffer,17); break;
		  case 6:  Float2Byte(&Data,DataScope_OutPut_Buffer,21); break;
		  case 7:  Float2Byte(&Data,DataScope_OutPut_Buffer,25); break;
		  case 8:  Float2Byte(&Data,DataScope_OutPut_Buffer,29); break;
		  case 9:  Float2Byte(&Data,DataScope_OutPut_Buffer,33); break;
		  case 10: Float2Byte(&Data,DataScope_OutPut_Buffer,37); break;
		}
  }	 
}

unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
	if ( (Channel_Number > 10) || (Channel_Number == 0) ) { return 0; }
  else
  {	
	 DataScope_OutPut_Buffer[0] = '$';
		
	 switch(Channel_Number)   
   { 
		 case 1:   DataScope_OutPut_Buffer[5]  =  5; return  6; break;   
		 case 2:   DataScope_OutPut_Buffer[9]  =  9; return 10; break;
		 case 3:   DataScope_OutPut_Buffer[13] = 13; return 14; break;
		 case 4:   DataScope_OutPut_Buffer[17] = 17; return 18; break;
		 case 5:   DataScope_OutPut_Buffer[21] = 21; return 22; break; 
		 case 6:   DataScope_OutPut_Buffer[25] = 25; return 26; break;
		 case 7:   DataScope_OutPut_Buffer[29] = 29; return 30; break;
		 case 8:   DataScope_OutPut_Buffer[33] = 33; return 34; break;
		 case 9:   DataScope_OutPut_Buffer[37] = 37; return 38; break;
     case 10:  DataScope_OutPut_Buffer[41] = 41; return 42; break;
   }	 
  }
	return 0;
}








