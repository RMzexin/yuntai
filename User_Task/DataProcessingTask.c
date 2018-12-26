#include "DataProcessingTask.h"

extern PID_Angle_Speed_t PID_Angle_Speed;

void Yuntai_angle_speed(float *pitch_speed , float *yaw_speed, float *pluck_speed)
{
	
	static unsigned short index = 0;
	static Bool first = true;
	static float pitch_angle_buf[10] = {0};
	static float yaw_angle_buf[10] = {0};
	static float pluck_angle_buf[10] = {0};
	static double biggest_P =0;
	static double smallest_P =0;
	static double biggest_Y =0;
	static double smallest_Y =0;
	static double biggest_L =0;
	static double smallest_L =0;
	
	if(first == true)
	{
		
	  first = false;
	  for(int i=0;i<10;i++)
		{
			pitch_angle_buf[i]=GMPitchEncoder.ecd_angle;
			yaw_angle_buf[i]  =GMYawEncoder.ecd_angle;
			pluck_angle_buf[i]=GMPluckEncoder.ecd_angle;
		}
	}
	pitch_angle_buf[index]=GMPitchEncoder.ecd_angle;
	yaw_angle_buf[index]  =GMYawEncoder.ecd_angle;
	pluck_angle_buf[index]=GMPluckEncoder.ecd_angle;
	biggest_P  = pitch_angle_buf[0];
	smallest_P = pitch_angle_buf[0];
	biggest_Y  = yaw_angle_buf[0];
	smallest_Y = yaw_angle_buf[0];
	biggest_L  = pluck_angle_buf[0];
	smallest_L = pluck_angle_buf[0];
	
	if(pitch_angle_buf[index]>biggest_P)
	{
		biggest_P = pitch_angle_buf[index];
	}
	if(pitch_angle_buf[index]<smallest_P )
	{
		smallest_P = pitch_angle_buf[index];
	}
	
	if(yaw_angle_buf[index]>biggest_Y)
	{
		biggest_Y = yaw_angle_buf[index];
	}
	if(yaw_angle_buf[index]<smallest_Y )
	{
		smallest_Y = yaw_angle_buf[index];
	}
	
	if(pluck_angle_buf[index]>biggest_L)
	{
		biggest_L = pluck_angle_buf[index];
	}
	if(pluck_angle_buf[index]<smallest_L )
	{
		smallest_L = pluck_angle_buf[index];
	}
	
	index++;
	if(index>=10)
	{
		PID_Angle_Speed.Pitch_Speed_LAST=PID_Angle_Speed.Pitch_Speed_NOW;
		PID_Angle_Speed.Yaw_Speed_LAST  =PID_Angle_Speed.Yaw_Speed_NOW;
		PID_Angle_Speed.Pluck_Speed_LAST=PID_Angle_Speed.Pluck_Speed_NOW;
		for(int i=0;i<10-1;i++)
		{
			PID_Angle_Speed.Pitch_Speed_NOW+=pitch_angle_buf[i];
			PID_Angle_Speed.Yaw_Speed_NOW  +=yaw_angle_buf[i];
			PID_Angle_Speed.Pluck_Speed_NOW+=pluck_angle_buf[i];
		}
		PID_Angle_Speed.Pitch_Speed_NOW=(PID_Angle_Speed.Pitch_Speed_NOW-biggest_P-smallest_P)/8;
		PID_Angle_Speed.Yaw_Speed_NOW  =(PID_Angle_Speed.Yaw_Speed_NOW  -biggest_Y-smallest_Y)/8;
		PID_Angle_Speed.Pluck_Speed_NOW=(PID_Angle_Speed.Pluck_Speed_NOW-biggest_L-smallest_L)/8;
		
		*pitch_speed=((PID_Angle_Speed.Pitch_Speed_NOW)-(PID_Angle_Speed.Pitch_Speed_LAST))/0.1f;
		*yaw_speed  =((PID_Angle_Speed.Yaw_Speed_NOW)  -(PID_Angle_Speed.Yaw_Speed_LAST))/0.1f;
		*pluck_speed=((PID_Angle_Speed.Pluck_Speed_NOW)-(PID_Angle_Speed.Pluck_Speed_LAST))/0.1f;
		
		index=0,smallest_P=0,biggest_P=0,smallest_Y=0,biggest_Y=0,smallest_L=0,biggest_L=0;
		
	}
}	


