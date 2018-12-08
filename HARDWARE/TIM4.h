#ifndef _TIM4_H
#define _TIM4_H
#include "main.h"
#include "sys.h"

void TIM4_Int_Init(u16 arr,u16 psc);
void TIM4_IRQHandler(void);

#endif

