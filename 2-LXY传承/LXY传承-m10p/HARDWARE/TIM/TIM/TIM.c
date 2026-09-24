/**
 * @file    TIM.c
 * @brief   TIM14中断定时器驱动（预留SPWM用）
 *
 * 内含预计算的800点正弦波查找表，用于SPWM输出。
 * TIM14中断以载波频率触发，每次更新TIM1 CCR2为查找表下一值。
 * 注: SPWM功能在量产代码中已注释，保留用于未来逆变器/电机驱动应用。
 */

#include "stm32f4xx.h"
#include "TIM.h"

/**
 * @brief  初始化TIM14为周期中断定时器
 *
 * SPWM示例: psc=0, arr=2100-1 -> 载波40kHz
 * TIM14 ISR从正弦查找表更新TIM1占空比。
 */
void TIM14_Init(u16 psc, u16 arr)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE); /*使能TIM14时钟*/

    TIM_TimeBaseInitStructure.TIM_Period        = arr; /*自动重装载值*/
    TIM_TimeBaseInitStructure.TIM_Prescaler     = psc; /*预分频值*/
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up; /*向上计数*/
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;  /*不分频*/

    TIM_TimeBaseInit(TIM14, &TIM_TimeBaseInitStructure); /*写入硬件*/

    TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM14, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM8_TRG_COM_TIM14_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
