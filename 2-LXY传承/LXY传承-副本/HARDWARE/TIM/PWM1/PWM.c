/**
 * @file    PWM1.c
 * @brief   简化版PWM初始化（TIM2 CH1, TIM3 CH1）— 使用硬编码初始脉宽1050
 *
 * PWM驱动变体，不含pulse参数，初始脉宽固定为1050。
 * 用于部分配置下的电机和舵机初始化。
 */

#include "stm32f4xx.h"
#include "PWM.h"

/**
 * @brief  TIM2 CH1 PWM初始化 (PA2) — 32位定时器, 初始脉宽=1050
 */
void TIM2_PWM_Init(u32 psc, u32 arr)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;           /* PA2 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler       = psc;
    TIM_TimeBaseStructure.TIM_CounterMode     = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period          = arr;
    TIM_TimeBaseStructure.TIM_ClockDivision   = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* TIM2 CH1 PWM模式1, 初始脉宽 = 1050 */
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse       = 1050;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief  TIM3 CH1 PWM初始化 (PA6) — 16位定时器, 初始脉宽=1050
 */
void TIM3_PWM_Init(u32 psc, u32 arr)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;           /* PA6 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler       = psc;
    TIM_TimeBaseStructure.TIM_CounterMode     = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period          = arr;
    TIM_TimeBaseStructure.TIM_ClockDivision   = TIM_CKD_DIV1;
    TIM_OCInitStructure.TIM_Pulse             = 1050;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* TIM3 CH1 PWM模式1 */
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}
