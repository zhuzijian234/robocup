#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
#include "moto.h"
#include "LEIDA_DATA.h" 
#include "BLUE.h"
#include "Servo.h"
#include "moto.h"



extern u16 Moto_pwm ;

void daoche(uint16_t daoche_speed,uint16_t ms,u8 zuoyou,u16 ZC_speed);
void TIM5_Int_Init(u16 arr,u16 psc);
//void TIM14_Int_Init(u16 arr,u16 psc);

#endif
