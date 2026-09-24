/**
 * @file    leida_pwm.c
 * @brief   雷达电机转速PWM控制驱动
 *
 * TIM1 CH4 (PA11): 雷达电机转速控制
 * TIM9 CH1 (PE5):  通用PWM（预留）
 */

#include "leida_pwm.h"

void LEIDA_PWM_Init(u32 arr, u32 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;           /* PA11 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler         = psc;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = arr;
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    /* TIM1 CH4 PWM模式1 */
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_Low;
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);  /* 高级定时器(TIM1/TIM8)必须调用此函数才能输出PWM */
}

void TIM9_PWM_Init(u32 psc, u32 arr, u32 pulse)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;           /* PE5 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler       = psc;
    TIM_TimeBaseStructure.TIM_CounterMode     = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period          = arr;
    TIM_TimeBaseStructure.TIM_ClockDivision   = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

    /* TIM9 CH1 PWM模式1 */
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse       = pulse;
    TIM_OC1Init(TIM9, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM9, ENABLE);

    TIM_Cmd(TIM9, ENABLE);
}
