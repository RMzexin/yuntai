#ifndef _TIM3_H
#define _TIM3_H
#include "main.h"
#include "sys.h"

void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_IRQHandler(void);
#endif
