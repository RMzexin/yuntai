#include "FireTask.h"
extern RC_Ctl_t RC_Ctl;

//����Ħ���ֵ���ź�
void  FireSetSingle()          
{
     TIM_SetCompare1(TIM4,1000);
}


//��������ź�
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
//����
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
	


