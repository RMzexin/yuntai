#ifndef _SPRINTF_H_
#define _SPRINTF_H_

#include "main.h"

extern unsigned char DataScope_OutPut_Buffer[42];	   //������֡���ݻ�����


void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    // дͨ�������� ������֡���ݻ�����

unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // ����֡�������ɺ��� 

void Print_RC_Data(void);
void Print_PID_Data(void);

#endif


