#ifndef _SPRINTF_H_
#define _SPRINTF_H_

#include "main.h"

extern unsigned char DataScope_OutPut_Buffer[42];	   //待发送帧数据缓存区


void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    // 写通道数据至 待发送帧数据缓存区

unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // 发送帧数据生成函数 

void Print_RC_Data(void);
void Print_PID_Data(void);

#endif


