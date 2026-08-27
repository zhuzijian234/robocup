#ifndef _PWM_H
#define _PWM_H

#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//注意修改定时器通道，即GPIO 引脚
//////////////////////////////////////////////////////////////////////////////////////////////////////////


//通用定时器PWM输出 初始化
//psc 预分频器  arr 自动重装值
void TIM2_PWM_Init(u32 psc,u32 arr);//通用定时器TIM2  32位
void TIM3_PWM_Init(u32 psc,u32 arr);//通用定时器TIM3  16位

void TIM_SetCompare1(TIM_TypeDef* TIMx, uint32_t Compare1);//设置改变占空比

//void TIM2_IRQHandler(void);//定时器2中断服务函数
//void TIM3_IRQHandler(void);//定时器3中断服务函数

#endif  
