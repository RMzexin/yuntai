#ifndef __STARTTASK_H__
#define __STARTTASK_H__
#include "main.h"


void start_task(void *p_arg);
void task1_task(void *p_arg);
void sprintf_task(void *p_arg);

void tmr1_callback(void *p_tmr, void *p_arg); 	//定时器1回调函数

#endif

