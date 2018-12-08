#include "StartTask.h"


//�������ȼ�
#define TASK1_TASK_PRIO		6
//�����ջ��С	
#define TASK1_STK_SIZE 		256
//������ƿ�
 OS_TCB Task1_TaskTCB;
 //�����ջ	
 CPU_STK TASK1_TASK_STK[TASK1_STK_SIZE];
 
 
 //�������ȼ�
#define SPRINTF_TASK_PRIO		9
//�����ջ��С	
#define SPRINTF_STK_SIZE 		128
//������ƿ�
 OS_TCB Sprintf_TaskTCB;
 //�����ջ	
 CPU_STK SPRINTF_TASK_STK[TASK1_STK_SIZE];
 
  OS_TMR 	tmr1;		//��ʱ��1
 


void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//ͳ������                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//���ʹ���˲����жϹر�ʱ��
    CPU_IntDisMeasMaxCurReset();	
#endif

#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //��ʹ��ʱ��Ƭ��ת��ʱ��
	 //ʹ��ʱ��Ƭ��ת���ȹ���,ʱ��Ƭ����Ϊ1��ϵͳʱ�ӽ��ģ���1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif
  //������ʱ��1
  OSTmrCreate((OS_TMR		*)&tmr1,		//��ʱ��1
                (CPU_CHAR	*)"tmr1",		//��ʱ������
                (OS_TICK	 )0,			//20*10=200ms
                (OS_TICK	 )1,          //1*10=10ms
                (OS_OPT		 )OS_OPT_TMR_PERIODIC, //����ģʽ
                (OS_TMR_CALLBACK_PTR)tmr1_callback,//��ʱ��1�ص�����
                (void	    *)0,			//����Ϊ0
                (OS_ERR	    *)&err);		//���صĴ�����			
	OS_CRITICAL_ENTER();	//�����ٽ���							
	//����TASK1����
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
	//����sprintf����							 
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
	OS_CRITICAL_EXIT();	//�˳��ٽ���
	OSTaskDel((OS_TCB*)0,&err);	//ɾ��start_task��������
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
	



