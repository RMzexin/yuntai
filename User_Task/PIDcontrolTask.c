#include "PIDcontrolTask.h"
#include "RemoteTask.h"
#include <math.h>
RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;
RampGen_t   GMYawRamp = RAMP_GEN_DAFAULT;
RampGen_t GMPluckRamp = RAMP_GEN_DAFAULT;
extern RC_Ctl_t RC_Ctl;

Gimbal_Ref_t gimbal_ref;				//云台数据

PID_t Position_pitchPID;        //pitch位置环PID计算参数
PID_t Position_yawPID;          //yaw位置环PID计算参数
PID_t Position_pluckPID;          //拨弹轮位置环PID计算参数
PID_t Speed_pitchPID;           //pitch速度环PID计算参数
PID_t Speed_yawPID;             //yaw速度环PID计算参数
PID_t Speed_pluckPID;             //拨弹轮速度环PID计算参数

PID_value_t PID_pitchPosition_value;  //pitch位置环理想值 实际值
PID_value_t PID_yawPosition_value;    //yaw位置环理想值 实际值
PID_value_t PID_pluckPosition_value;    //拨弹轮位置环理想值 实际值
PID_value_t PID_pitchSpeed_value;     //pitch速度环理想值 实际值
PID_value_t PID_yawSpeed_value;       //yaw速度环理想值 实际值
PID_value_t PID_pluckSpeed_value;       //拨弹轮速度环理想值 实际值
PID_value_t PID_CM1_value; 
PID_value_t PID_CM2_value;
PID_value_t PID_CM3_value;
PID_value_t PID_CM4_value;

pid_t CM1pid={100,3.3,5,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,16000,-16000};
pid_t CM2pid={100,3.3,5,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,16000,-16000}; 
pid_t CM3pid={100,3.3,5,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,16000,-16000}; 
pid_t CM4pid={100,3.3,5,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,16000,-16000}; 
pid_t   YAWPpid={3.4,0,0,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,5000,-5000};
pid_t PITCHPpid={4.1,0,0,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,5000,-5000}; 
pid_t   YAWSpid={21.75,0,0,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,5000,-5000}; 
pid_t PITCHSpid={21.75,0,0,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,5000,-5000}; 
pid_t PLUCKPpid={1.2,0,0,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,5000,-5000}; 
pid_t PLUCKSpid={1.4,0.12,0,0,0,0,0,0,0,0,0,0,0,1000,-1000,0,0,0,0,10000,-10000}; 

PID_Angle_Speed_t PID_Angle_Speed;

void Chassis_And_Gimbal_Data_Init(void)
{
	gimbal_ref.pitch_angle_dynamic_ref = +GMPitchEncoder.ecd_angle;		//让云台稳定在上电的位置
  gimbal_ref.yaw_angle_dynamic_ref   = +GMYawEncoder.ecd_angle;
	gimbal_ref.pluck_angle_dynamic_ref = +GMPluckEncoder.ecd_angle;
	
	PID_yawPosition_value.ideal    = gimbal_ref.yaw_angle_dynamic_ref;
	PID_yawPosition_value.actual   = GMYawEncoder.ecd_angle;
	PID_pitchPosition_value.ideal  = gimbal_ref.pitch_angle_dynamic_ref;
	PID_pitchPosition_value.actual = GMPitchEncoder.ecd_angle;
	PID_pluckPosition_value.ideal  = gimbal_ref.pluck_angle_dynamic_ref;
	PID_pluckPosition_value.actual = GMPluckEncoder.ecd_angle;
}

void RAMP_INIT()
{
	 GMPitchRamp.SetScale(&GMPitchRamp, PREPARE_TIME_TICK_MS);
	GMYawRamp.  SetScale(&GMYawRamp,   PREPARE_TIME_TICK_MS);
	GMPluckRamp.SetScale(&GMPluckRamp, PREPARE_TIME_TICK_MS);
	GMPitchRamp.ResetCounter(&GMPitchRamp);
	GMYawRamp.  ResetCounter(&GMYawRamp);
	GMPluckRamp.ResetCounter(&GMPluckRamp);
}


                   
void Set_Gimbal_Motor_Output(void)	 //can 输出
{	
 	//Set_Gimbal_Current(CAN1, (int16_t)YAWSpid.pidout, (int16_t)PITCHSpid.pidout, (int16_t)PLUCKSpid.pidout);
  Set_Gimbal_Current(CAN1, (int16_t)0, (int16_t)0, (int16_t)0);	
	Set_CM_Speed(CAN1,(int16_t)CM1pid.pidout,(int16_t)CM2pid.pidout,(int16_t)CM3pid.pidout,(int16_t)CM4pid.pidout);
}


void Gimbal_RC_Mode(void)          //遥控值写入
{
	  mpu9250_assignment();// 陀螺仪赋值
	  gimbal_ref.pitch_angle_dynamic_ref -=((Get_ch3_Data()-1024)*STICK_TO_PITCH_ANGLE_INC_FACT) ;
    gimbal_ref.yaw_angle_dynamic_ref   -=((Get_ch2_Data()-1024)*STICK_TO_YAW_ANGLE_INC_FACT);
	  gimbal_ref.pluck_angle_dynamic_ref +=PLUCK();
		gimbal_ref.left_right_ref           =((Get_ch0_Data()-1024)*STICK_TO_FORWARD_BACK_ANGLE_INC_FACT) ;
    gimbal_ref.forward_back_ref         =((Get_ch1_Data()-1024)*STICK_TO_LEFT_RIGHT_ANGLE_INC_FACT);
	  
	
	  PID_CM1_value.ideal = -gimbal_ref.forward_back_ref*0.075f - gimbal_ref.left_right_ref*0.075f ;
	  PID_CM2_value.ideal = gimbal_ref.forward_back_ref*0.075f - gimbal_ref.left_right_ref*0.075f;
	  PID_CM3_value.ideal = -gimbal_ref.forward_back_ref*0.075f - gimbal_ref.left_right_ref*0.075f;
	  PID_CM4_value.ideal = gimbal_ref.forward_back_ref*0.075f - gimbal_ref.left_right_ref*0.075f;
	  PID_pitchPosition_value.ideal = gimbal_ref.pitch_angle_dynamic_ref;
	  PID_yawPosition_value.ideal   = gimbal_ref.yaw_angle_dynamic_ref;
	  PID_pluckPosition_value.ideal = gimbal_ref.pluck_angle_dynamic_ref;
	

	/* not used to control, just as a flag */ 
    gimbal_ref.pitch_speed_ref =(Get_ch3_Data()-1024);    //speed_ref仅做输入量判断用
    gimbal_ref.yaw_speed_ref   =(Get_ch2_Data()-1024);	
}
float  PLUCK()
{
	static Bool first = true;
	if(RC_Ctl.rc.s1==3&&first)
	{
		first = false;
		return 360.0*PLUCK_RATIO/7.0;
	}
	else if(RC_Ctl.rc.s1==1&&first==false)
	{
	  first = true;
		GMPluckRamp.ResetCounter(&GMPluckRamp);
		return 0;
	}
	else
	{
		GMPluckRamp.ResetCounter(&GMPluckRamp);
		return 0;
	}
}


void PID_calculate_position_self(void)//云台位置环PID算法
{ 
	pid_calculate(&PITCHPpid,GMPitchEncoder.ecd_angle,PID_pitchPosition_value.ideal);	
	pid_calculate(&YAWPpid,  GMYawEncoder.ecd_angle,  PID_yawPosition_value.ideal);	
	pid_calculate(&PLUCKPpid,GMPluckEncoder.ecd_angle,PID_pluckPosition_value.ideal);	
  PID_calculate_speed_self();	
}

void YUNTAI_SPEED()
{
	Yuntai_angle_speed(&PID_pitchSpeed_value.actual,&PID_yawSpeed_value.actual,&PID_pluckSpeed_value.actual);
}

void PID_calculate_speed_self(void)//云台速度环PID算法
{			
	PID_pitchSpeed_value.ideal =PITCHPpid.pidout;
	PID_yawSpeed_value.ideal   =YAWPpid.pidout;
	PID_pluckSpeed_value.ideal =PLUCKPpid.pidout;
	pid_calculate(&PITCHSpid,-PID_pitchSpeed_value.actual,-PID_pitchSpeed_value.ideal);
	pid_calculate(&YAWSpid,  -PID_yawSpeed_value.actual,  -PID_yawSpeed_value.ideal);
  pid_calculate(&PLUCKSpid,PID_pluckSpeed_value.actual ,PID_pluckSpeed_value.ideal);	
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

