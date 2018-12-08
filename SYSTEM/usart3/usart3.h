#ifndef USART3_H
#define USART3_H

#include "main.h"

#define USART3_REC_LEN  			200 

extern u8  USART3_RX_BUF[USART3_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART3_RX_STA;         		//接收状态标记	

void Initial_UART3(unsigned long baudrate);

#endif
