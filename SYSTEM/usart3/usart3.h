#ifndef USART3_H
#define USART3_H

#include "main.h"

#define USART3_REC_LEN  			200 

extern u8  USART3_RX_BUF[USART3_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART3_RX_STA;         		//����״̬���	

void Initial_UART3(unsigned long baudrate);

#endif
