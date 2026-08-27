#ifndef __LEIDA_PWM_H
#define __LEIDA_PWM_H

#include "sys.h" 

void Leida_PWM_Init(uint16_t psc,uint16_t arr,uint16_t puse);
void TIM9_PWM_Init(u32 psc,u32 arr,u32 pulse);

#endif

