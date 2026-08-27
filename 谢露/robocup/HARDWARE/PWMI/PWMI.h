#ifndef _PWMI_H
#define _PWMI_H

#include "sys.h"

void PWMI_Init(void); //PWM检测口 				PB6		   TIM4_CH1

/**
  * 函    数：获取输入捕获的频率
  * 参    数：无
  * 返 回 值：捕获得到的频率
  */
uint32_t IC_GetFreq(void);//这两个函数变定时器的时候别忘了改

/**
  * 函    数：获取输入捕获的占空比
  * 参    数：无
  * 返 回 值：捕获得到的占空比
  */
uint32_t IC_GetDuty(void);


#endif  
