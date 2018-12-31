#ifndef _PID_CONTROL_TAKE_H_
#define _PID_CONTROL_TAKE_H_
#include "main.h"
#include "stm32f4xx.h"
#include "RemoteTask.h"	
#include "PIDcontrolTask.h"
#include "CanBusTask.h"
#include "Ramp.h"

typedef struct
{
    float pitch_angle_dynamic_ref;
    float yaw_angle_dynamic_ref;
   	float pluck_angle_dynamic_ref;
    float pitch_angle_static_ref;
    float yaw_angle_static_ref;
	  float pluck_angle_static_ref;
    float pitch_speed_ref;
    float yaw_speed_ref;
	  float pluck_speed_ref;
	
	  float forward_back_ref;
    float left_right_ref;	
}Gimbal_Ref_t;				//云台数据（期望值）结构体


typedef struct
{
	float Ideal_Now;
	float Ideal_Next;
	float Ideal_Last;
	float D_Now;
	float D_Last;
	float D_Voltage;	
}PID_t;


typedef struct
{
	float ideal;
	float actual;
}PID_value_t;

typedef struct
{
	float Pitch_Speed_NOW;
	float Pitch_Speed_LAST;
	
	float Yaw_Speed_NOW;
	float Yaw_Speed_LAST;
	
	float Pluck_Speed_NOW;
	float Pluck_Speed_LAST;
}PID_Angle_Speed_t;


#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\


#define PREPARE_TIME_TICK_MS 4000      //prapare time in ms
#define PLUCK_RATIO          36        //拨弹轮减速比


//#define PID_P_Position_KP 4.032f
//#define PID_P_Position_KI 0.06f
//#define PID_P_Position_KD 0.0f

//#define PID_Y_Position_KP 3.665f
//#define PID_Y_Position_KI 0.000f
//#define PID_Y_Position_KD 0.00f


//#define PID_P_Speed_KP 72.75f
//#define PID_P_Speed_KI 0.00f
//#define PID_P_Speed_KD 6.5f

//#define PID_Y_Speed_KP 81.75f
//#define PID_Y_Speed_KI 0.145f
//#define PID_Y_Speed_KD 0.00f

#define PID_P_Position_KP 3.532f
#define PID_P_Position_KI 0.0015f
#define PID_P_Position_KD 0.0f

#define PID_Y_Position_KP 3.665f
#define PID_Y_Position_KI 0.000f
#define PID_Y_Position_KD 0.00f


#define PID_P_Speed_KP 12.75f
#define PID_P_Speed_KI 0.102f
#define PID_P_Speed_KD 6.5f

#define PID_Y_Speed_KP 11.75f
#define PID_Y_Speed_KI 0.145f
#define PID_Y_Speed_KD 0.00f


#define PID_CM1_KP 280.665f
#define PID_CM1_KI 6.000f
#define PID_CM1_KD 10.00f

#define PID_CM2_KP 280.665f
#define PID_CM2_KI 6.000f
#define PID_CM2_KD 10.00f

#define PID_CM3_KP 280.665f
#define PID_CM3_KI 6.000f
#define PID_CM3_KD 10.00f

#define PID_CM4_KP 280.665f
#define PID_CM4_KI 6.000f
#define PID_CM4_KD 10.00f



#define D_Value_P_Voltage_MAX   500
#define D_Value_Y_Voltage_MAX   500

//云台灵敏度
#define STICK_TO_PITCH_ANGLE_INC_FACT       0.0028f   
#define STICK_TO_YAW_ANGLE_INC_FACT         0.0028f

#define STICK_TO_FORWARD_BACK_ANGLE_INC_FACT       3.68f   
#define STICK_TO_LEFT_RIGHT_ANGLE_INC_FACT         3.68f



void Chassis_And_Gimbal_Data_Init(void);
void RAMP_INIT(void);
void PID_Init(void);
void Set_Gimbal_Motor_Output(void);
void Gimbal_RC_Mode(void);

void PID_calculate_position_self(void);
void YUNTAI_SPEED(void);
void PID_calculate_speed_self(void);
void PID_calculate_chassis_self(void);

void Value_Voltage_Init(void);
void Print_PID_Data(void);

#endif


