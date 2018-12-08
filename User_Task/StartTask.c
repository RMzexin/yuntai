#include "StartTask.h"


//任务优先级
#define TASK1_TASK_PRIO		6
//任务堆栈大小	
#define TASK1_STK_SIZE 		256
//任务控制块
 OS_TCB Task1_TaskTCB;
 //任务堆栈	
 CPU_STK TASK1_TASK_STK[TASK1_STK_SIZE];
 
 
 //任务优先级
#define SPRINTF_TASK_PRIO		9
//任务堆栈大小	
#define SPRINTF_STK_SIZE 		128
//任务控制块
 OS_TCB Sprintf_TaskTCB;
 //任务堆栈	
 CPU_STK SPRINTF_TASK_STK[TASK1_STK_SIZE];
 
  OS_TMR 	tmr1;		//定时器1
 


void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//统计任务                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();	
#endif

#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //当使用时间片轮转的时候
	 //使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif
  //创建定时器1
  OSTmrCreate((OS_TMR		*)&tmr1,		//定时器1
                (CPU_CHAR	*)"tmr1",		//定时器名字
                (OS_TICK	 )0,			//20*10=200ms
                (OS_TICK	 )1,          //1*10=10ms
                (OS_OPT		 )OS_OPT_TMR_PERIODIC, //周期模式
                (OS_TMR_CALLBACK_PTR)tmr1_callback,//定时器1回调函数
                (void	    *)0,			//参数为0
                (OS_ERR	    *)&err);		//返回的错误码			
	OS_CRITICAL_ENTER();	//进入临界区							
	//创建TASK1任务
	OSTaskCreate((OS_TCB 	* )&Task1_TaskTCB,		
				 (CPU_CHAR	* )"Task1 task", 		
                 (OS_TASK_PTR )task1_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )TASK1_TASK_PRIO,     
                 (CPU_STK   * )&TASK1_TASK_STK[0],	
                 (CPU_STK_SIZE)TASK1_STK_SIZE/10,	
                 (CPU_STK_SIZE)TASK1_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,  					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);
	//创建sprintf任务							 
//OSTaskCreate((OS_TCB 	* )&Sprintf_TaskTCB,		
//				 (CPU_CHAR	* )"printf task", 		
//                 (OS_TASK_PTR )sprintf_task, 			
//                 (void		* )0,					
//                 (OS_PRIO	  )SPRINTF_TASK_PRIO,     
//                 (CPU_STK   * )&SPRINTF_TASK_STK[0],	
//                 (CPU_STK_SIZE)SPRINTF_STK_SIZE/10,	
//                 (CPU_STK_SIZE)SPRINTF_STK_SIZE,		
//                 (OS_MSG_QTY  )0,					
//                 (OS_TICK	  )2,  					
//                 (void   	* )0,					
//                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//                 (OS_ERR 	* )&err);
	OS_CRITICAL_EXIT();	//退出临界区
	OSTaskDel((OS_TCB*)0,&err);	//删除start_task任务自身
}

void task1_task(void *p_arg)
{
	
	OS_ERR err;
	OSTmrStart(&tmr1,&err);
	  while(1)
		{
			if(Get_ch0_Data()<364||Get_ch1_Data()<364||Get_ch2_Data()<364|Get_ch3_Data()<364)
		{
			RC_Init();
		}
		Print_PID_Data();
//		Print_RC_Data();
		}
}

void tmr1_callback(void *p_tmr, void *p_arg)
{

//	  PID_calculate_chassis_self();
}
	



