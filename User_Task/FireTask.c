#include "FireTask.h"
extern RC_Ctl_t RC_Ctl;

//配置摩擦轮电调信号
void  FireSetSingle()          
{
     TIM_SetCompare1(TIM4,1000);
}


//电调开启信号
void  FireOpen()            
{
   	TIM_SetCompare1(TIM4,1500);
}

void  Firefast()            
{
    TIM_SetCompare1(TIM4,1800);
}


float  PLUCK()
{
	static Bool first = true;
	if(RC_Ctl.rc.s1==3&&first)
	{
		first = false;
		return 360.0*PLUCK_RATIO/9.0;
	}
	else if(RC_Ctl.rc.s1==1&&first==false)
	{
	  first = true;
		return 0;
	}
	else
	{
		return 0;
	}
}
//拨弹
void  SHORT()
{
	if(RC_Ctl.rc.s2==3)
	{
		FireOpen();
	}
	else if(RC_Ctl.rc.s2==1)
	{
	  FireSetSingle();
	}
	else if(RC_Ctl.rc.s2==2)
	{
		Firefast();
	}
}
	


