#include "pid.h"

float pid_calculate(pid_t* pid, float get, float set)
{
	static unsigned short index = 0;
	pid->err[LAST]= pid->err[NOW];
	pid->get[NOW] = get;
  pid->set[NOW] = set;
  pid->err[NOW] = set - get;	
	
	if(pid->Voltage>pid->Voltage_max)
       {
         index=0.0;
				 if(pid->err[NOW]<0)
					pid->Voltage+=pid->err[NOW]; 
       }
			 else if(pid->Voltage<pid->Voltage_min)
       {
         index=0.0;
				 if(pid->err[NOW]>0)
					pid->Voltage+=pid->err[NOW];
       }
			 else if(fabs(pid->Voltage)<pid->Voltage_max-(pid->Voltage_max/4))
		   {
         index=1.0;
				 pid->Voltage+=pid->err[NOW];
			 }
			 else
			{
				index=(pid->Voltage_max-fabs(pid->Voltage))/(pid->Voltage_max/4);
				pid->Voltage+=pid->err[NOW];
			}
			

			pid->pout=pid->p*pid->err[NOW];
			pid->iout=index*pid->i*pid->Voltage;
			pid->dout=pid->d*(pid->err[NOW]-pid->err[LAST]);
			pid->pidout=pid->pout + pid->iout + pid->dout;
			VAL_LIMIT(pid->pidout,pid->MinOutput,pid->MaxOutput);
			return pid->pidout;	
		}

		
