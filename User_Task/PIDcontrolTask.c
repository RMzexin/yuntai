#include "PIDcontrolTask.h"
#include "RemoteTask.h"
#include <math.h>
RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;
RampGen_t   GMYawRamp = RAMP_GEN_DAFAULT;

Gimbal_Ref_t gimbal_ref;				//云台数据

PID_t Position_pitchPID;        //pitch位置环PID计算参数
PID_t Position_yawPID;          //yaw位置环PID计算参数
PID_t Speed_pitchPID;           //pitch速度环PID计算参数
PID_t Speed_yawPID;             //yaw速度环PID计算参数

PID_value_t PID_CM1_value; 
PID_value_t PID_CM2_value;
PID_value_t PID_CM3_value;
PID_value_t PID_CM4_value;

pid_t CM1pid={100,6.3,5,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,16000,-16000};
pid_t CM2pid={100,6.3,5,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,16000,-16000}; 
pid_t CM3pid={100,6.3,5,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,16000,-16000}; 
pid_t CM4pid={100,6.3,5,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,16000,-16000}; 

PID_value_t PID_pitchPosition_value;  //pitch位置环理想值 实际值
PID_value_t PID_yawPosition_value;    //yaw位置环理想值 实际值
PID_value_t PID_pitchSpeed_value;     //pitch速度环理想值 实际值
PID_value_t PID_yawSpeed_value;       //yaw速度环理想值 实际值

PID_Angle_Speed_t PID_Angle_Speed;



float PID_pitch_Position_out;
float PID_yaw_Position_out;
float PID_pitch_Speed_out;
float PID_yaw_Speed_out;


void Chassis_And_Gimbal_Data_Init(void)
{
	gimbal_ref.pitch_angle_dynamic_ref = +GMPitchEncoder.ecd_angle;		//让云台稳定在上电的位置
  gimbal_ref.yaw_angle_dynamic_ref = +GMYawEncoder.ecd_angle;
	
	PID_yawPosition_value.ideal = gimbal_ref.yaw_angle_dynamic_ref;
	PID_yawPosition_value.actual =GMYawEncoder.ecd_angle;
	PID_pitchPosition_value.ideal = gimbal_ref.pitch_angle_dynamic_ref;
	PID_pitchPosition_value.actual =GMPitchEncoder.ecd_angle;
	
	
  GMPitchRamp.SetScale(&GMPitchRamp, PREPARE_TIME_TICK_MS);
	GMYawRamp.SetScale(&GMYawRamp, PREPARE_TIME_TICK_MS);
	GMPitchRamp.ResetCounter(&GMPitchRamp);
	GMYawRamp.ResetCounter(&GMYawRamp);
}

void PID_Init(void) //PID各参数初始化为0
{
	Position_pitchPID.Ideal_Now=0.00;
	Position_pitchPID.D_Voltage=0.00;
	Position_pitchPID.Ideal_Next=0.00;
	Position_pitchPID.Ideal_Last=0.00;
	
	Position_yawPID.Ideal_Now=0.00;
	Position_yawPID.D_Voltage=0.00;
	Position_yawPID.Ideal_Next=0.00;
	Position_yawPID.Ideal_Last=0.00;
	
	
	Speed_pitchPID.Ideal_Now=0.00;
	Speed_pitchPID.D_Voltage=0.00;
	Speed_pitchPID.Ideal_Next=0.00;
	Speed_pitchPID.Ideal_Last=0.00;
	
	Speed_pitchPID.Ideal_Now=0.00;
	Speed_pitchPID.D_Voltage=0.00;
	Speed_pitchPID.Ideal_Next=0.00;
	Speed_pitchPID.Ideal_Last=0.00;
	

}

                    
 void Set_Gimbal_Motor_Output(void)	 //can 输出
{	
// 	Set_Gimbal_Current(CAN1, (int16_t)PID_yaw_Speed_out, (int16_t)PID_pitch_Speed_out);	
	Set_CM_Speed(CAN1,(int16_t)CM1pid.pidout,(int16_t)CM2pid.pidout,(int16_t)CM3pid.pidout,(int16_t)CM4pid.pidout);
}


  void Gimbal_RC_Mode(void)          //遥控值写入
{
	
	  gimbal_ref.pitch_angle_dynamic_ref -=((Get_ch3_Data()-1024)*STICK_TO_PITCH_ANGLE_INC_FACT) ;
    gimbal_ref.yaw_angle_dynamic_ref   -=((Get_ch2_Data()-1024)*STICK_TO_YAW_ANGLE_INC_FACT);
		gimbal_ref.left_right_ref           =((Get_ch0_Data()-1024)*STICK_TO_FORWARD_BACK_ANGLE_INC_FACT) ;
    gimbal_ref.forward_back_ref         =((Get_ch1_Data()-1024)*STICK_TO_LEFT_RIGHT_ANGLE_INC_FACT);
	
	  PID_CM1_value.ideal = -gimbal_ref.forward_back_ref*0.075f - gimbal_ref.left_right_ref*0.075f ;
	  PID_CM2_value.ideal = gimbal_ref.forward_back_ref*0.075f - gimbal_ref.left_right_ref*0.075f;
	  PID_CM3_value.ideal = -gimbal_ref.forward_back_ref*0.075f - gimbal_ref.left_right_ref*0.075f;
	  PID_CM4_value.ideal = gimbal_ref.forward_back_ref*0.075f - gimbal_ref.left_right_ref*0.075f;

	/* not used to control, just as a flag */ 
    gimbal_ref.pitch_speed_ref =(Get_ch3_Data()-1024);    //speed_ref仅做输入量判断用
    gimbal_ref.yaw_speed_ref   =(Get_ch2_Data()-1024);
	

//	  PID_calculate_chassis_self();
}

void PID_calculate_position_self(void)//云台位置环PID算法
{ 
	static unsigned short index = 0;
	static unsigned short index_1 = 0;
  PID_pitchPosition_value.ideal = gimbal_ref.pitch_angle_dynamic_ref;
	PID_pitchPosition_value.actual =GMPitchEncoder.ecd_angle;
  Position_pitchPID.Ideal_Last=Position_pitchPID.Ideal_Now;	
	Position_pitchPID.Ideal_Now=PID_pitchPosition_value.ideal-PID_pitchPosition_value.actual;

    if(Position_pitchPID.D_Voltage>Position_pitch_D_Voltage_MAX)
       {
         index=0.0;
				 if(Position_pitchPID.Ideal_Now<0)
					Position_pitchPID.D_Voltage+=Position_pitchPID.Ideal_Now; 
       }
			 else if(Position_pitchPID.D_Voltage<Position_pitch_D_Voltage_MIN)
       {
         index=0.0;
				 if(Position_pitchPID.Ideal_Now>0)
					Position_pitchPID.D_Voltage+=Position_pitchPID.Ideal_Now; 
       }
			 else if(fabs(Position_pitchPID.D_Voltage)<Position_pitch_D_Voltage_MAX-(Position_pitch_D_Voltage_MAX/4))
		   {
         index=1.0;
				 Position_pitchPID.D_Voltage+=Position_pitchPID.Ideal_Now;
			 }
			 else
			{
       index=(Position_pitch_D_Voltage_MAX-fabs(Position_pitchPID.D_Voltage))/(Position_pitch_D_Voltage_MAX/4);
			Position_pitchPID.D_Voltage+=Position_pitchPID.Ideal_Now;
			}

		
	PID_pitch_Position_out=-(PID_P_Position_KP*Position_pitchPID.Ideal_Now+
	                       index*PID_P_Position_KI*Position_pitchPID.D_Voltage+
	                       PID_P_Position_KD*(Position_pitchPID.Ideal_Now-Position_pitchPID.Ideal_Last));
	
	
	
  PID_yawPosition_value.ideal = gimbal_ref.yaw_angle_dynamic_ref;
	PID_yawPosition_value.actual =GMYawEncoder.ecd_angle;
  Position_yawPID.Ideal_Last=Position_yawPID.Ideal_Now;	
	Position_yawPID.Ideal_Now=PID_yawPosition_value.ideal-PID_yawPosition_value.actual;
			if(Position_yawPID.D_Voltage>Position_yaw_D_Voltage_MAX)
       {
         index_1=0.0;
				 if(Position_yawPID.Ideal_Now<0)
					Position_yawPID.D_Voltage+=Position_yawPID.Ideal_Now; 
       }
			 else if(Position_yawPID.D_Voltage<Position_yaw_D_Voltage_MIN)
       {
         index_1=0.0;
				 if(Position_yawPID.Ideal_Now>0)
					Position_yawPID.D_Voltage+=Position_yawPID.Ideal_Now; 
       }
		else if(fabs(Position_yawPID.D_Voltage)<Position_yaw_D_Voltage_MAX-(Position_yaw_D_Voltage_MAX/4))
		   {
         index_1=1.0;
				 Position_yawPID.D_Voltage+=Position_yawPID.Ideal_Now;
			 }
		else
			{
       index_1=(Position_yaw_D_Voltage_MAX-fabs(Position_yawPID.D_Voltage))/(Position_yaw_D_Voltage_MAX/4);
			Position_yawPID.D_Voltage+=Position_yawPID.Ideal_Now;
			}
			
	PID_yaw_Position_out=-(PID_Y_Position_KP*Position_yawPID.Ideal_Now+
	                     index_1*PID_Y_Position_KI*Position_yawPID.D_Voltage+
	                     PID_Y_Position_KD*(Position_yawPID.Ideal_Now-Position_yawPID.Ideal_Last));	

			
}

void YUNTAI_SPEED()
{
	Yuntai_angle_speed(&PID_pitchSpeed_value.actual,&PID_yawSpeed_value.actual);
}

void PID_calculate_speed_self(void)//云台速度环PID算法
{
	static unsigned short index = 0;
	static unsigned short index_1 = 0;			
	PID_pitchSpeed_value.ideal =PID_pitch_Position_out;
  Speed_pitchPID.Ideal_Last=Speed_pitchPID.Ideal_Now;	
	Speed_pitchPID.Ideal_Now=PID_pitchSpeed_value.ideal+PID_pitchSpeed_value.actual;
    if(Speed_pitchPID.D_Voltage>Speed_pitch_D_Voltage_MAX)
       {
         index=1.0;
				 if(Speed_pitchPID.Ideal_Now<0)
					Speed_pitchPID.D_Voltage+=Speed_pitchPID.Ideal_Now; 
       }
			 else if(Speed_pitchPID.D_Voltage<Speed_pitch_D_Voltage_MIN)
       {
         index=1.0;
				 if(Speed_pitchPID.Ideal_Now>0)
					Speed_pitchPID.D_Voltage+=Speed_pitchPID.Ideal_Now; 
       }
		else if(fabs(Speed_pitchPID.D_Voltage)<Speed_pitch_D_Voltage_MAX-(Speed_pitch_D_Voltage_MAX/4))
		   {
         index=1.0;
				 Speed_pitchPID.D_Voltage+=Speed_pitchPID.Ideal_Now;
			 }
		else
			{
				index=1.0;
//       index=(Speed_pitch_D_Voltage_MAX-fabs(Speed_pitchPID.D_Voltage))/(Speed_pitch_D_Voltage_MAX/4);
			Speed_pitchPID.D_Voltage+=Speed_pitchPID.Ideal_Now;
			}

		
	PID_pitch_Speed_out=PID_P_Speed_KP*Speed_pitchPID.Ideal_Now+
	                    index*PID_P_Speed_KI*Speed_pitchPID.D_Voltage+
	                    PID_P_Speed_KD*(Speed_pitchPID.Ideal_Now-Speed_pitchPID.Ideal_Last);
	
			
			
	PID_yawSpeed_value.ideal =PID_yaw_Position_out;
  Speed_yawPID.Ideal_Last=Speed_yawPID.Ideal_Now;	
	Speed_yawPID.Ideal_Now=PID_yawSpeed_value.ideal+PID_yawSpeed_value.actual;
    if(Speed_yawPID.D_Voltage>Speed_yaw_D_Voltage_MAX)
       {
         index_1=1.0;
				 if(Speed_yawPID.Ideal_Now<0)
					Speed_yawPID.D_Voltage+=Speed_yawPID.Ideal_Now; 
       }
			 else if(Speed_yawPID.D_Voltage<Speed_yaw_D_Voltage_MIN)
       {
         index_1=1.0;
				 if(Speed_yawPID.Ideal_Now>0)
					Speed_yawPID.D_Voltage+=Speed_yawPID.Ideal_Now; 
       }
		else if(fabs(Speed_yawPID.D_Voltage)<Speed_yaw_D_Voltage_MAX-(Speed_yaw_D_Voltage_MAX/4))
		   {
         index_1=1.0;
				 Speed_yawPID.D_Voltage+=Speed_yawPID.Ideal_Now;
			 }
		else
			{
				index_1=1.0;
//       index_1=(Speed_yaw_D_Voltage_MAX-fabs(Speed_yawPID.D_Voltage))/(Speed_yaw_D_Voltage_MAX/4);
			 Speed_yawPID.D_Voltage+=Speed_yawPID.Ideal_Now;
			}

		
	PID_yaw_Speed_out=PID_Y_Speed_KP*Speed_yawPID.Ideal_Now+
	                   index_1*PID_Y_Speed_KI*Speed_yawPID.D_Voltage+
	                   PID_Y_Speed_KD*(Speed_yawPID.Ideal_Now-Speed_yawPID.Ideal_Last);

			Set_Gimbal_Motor_Output();	
}




void PID_calculate_chassis_self(void)//底盘PID算法
{
  pid_calculate(&CM1pid,CM1Encoder.rotor_speed,-PID_CM1_value.ideal);	
	pid_calculate(&CM2pid,CM2Encoder.rotor_speed,-PID_CM2_value.ideal);	
	pid_calculate(&CM3pid,CM3Encoder.rotor_speed,-PID_CM3_value.ideal);	
	pid_calculate(&CM4pid,CM4Encoder.rotor_speed,-PID_CM4_value.ideal);	
	Set_Gimbal_Motor_Output();
	

}

