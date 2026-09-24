/**
 * @file    PWM_pn_out.c
 * @brief   高级定时器TIM1互补PWM输出（含死区）
 *
 * TIM1 CH1:  PA8 (P通道输出)
 * TIM1 CH1N: PA7 (N通道/互补输出)
 *
 * 死区时间: ~3us (TIM_DeadTime = 11)
 * 用途: 半桥/H桥电机驱动，互补PWM+死区防止上下管直通。
 */

#include "stm32f4xx.h"
#include "PWM_PN_OUT.h"

void TIM_PWM_PN_Init(u16 psc, u16 arr)
{
    GPIO_InitTypeDef         GPIO_PWMInit;
    TIM_TimeBaseInitTypeDef  TIM1_TIMERType;
    TIM_OCInitTypeDef        TIM1_PWMOC;
    TIM_BDTRInitTypeDef      TIM1_BDTRType;

    /* 使能时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* 引脚复用: PA7 -> TIM1 CH1N, PB8 -> TIM1 CH1 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM1);

    /* PA7: CH1N (互补输出) */
    GPIO_PWMInit.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_PWMInit.GPIO_OType = GPIO_OType_PP;
    GPIO_PWMInit.GPIO_Pin   = GPIO_Pin_7;
    GPIO_PWMInit.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_PWMInit.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_PWMInit);

    /* PA8: CH1 (主输出) */
    GPIO_PWMInit.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_PWMInit.GPIO_OType = GPIO_OType_PP;
    GPIO_PWMInit.GPIO_Pin   = GPIO_Pin_8;
    GPIO_PWMInit.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_PWMInit.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_PWMInit);

    /* 定时器基础配置 */
    TIM1_TIMERType.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM1_TIMERType.TIM_Period        = arr;
    TIM1_TIMERType.TIM_Prescaler     = psc;
    TIM1_TIMERType.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM1_TIMERType);

    /* PWM输出配置 — CH1 + CH1N */
    TIM1_PWMOC.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM1_PWMOC.TIM_Pulse        = 1000;                        /* 初始占空比 */
    TIM1_PWMOC.TIM_OCIdleState  = TIM_OCIdleState_Reset;       /* 空闲时输出低 */
    TIM1_PWMOC.TIM_OutputState  = TIM_OutputState_Enable;      /* 主输出使能 */
    TIM1_PWMOC.TIM_OCPolarity   = TIM_OCPolarity_High;          /* 主输出高有效 */
    TIM1_PWMOC.TIM_OutputNState = TIM_OutputNState_Enable;     /* 互补输出使能 */
    TIM1_PWMOC.TIM_OCNIdleState = TIM_OCNIdleState_Reset;      /* 互补空闲低 */
    TIM1_PWMOC.TIM_OCNPolarity  = TIM_OCNPolarity_High;         /* 互补输出高有效 */
    TIM_OC1Init(TIM1, &TIM1_PWMOC);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* 刹车和死区配置 */
    TIM1_BDTRType.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;  /* 自动输出使能 */
    TIM1_BDTRType.TIM_Break           = TIM_Break_Disable;           /* 刹车功能关闭 */
    TIM1_BDTRType.TIM_BreakPolarity   = TIM_BreakPolarity_High;
    TIM1_BDTRType.TIM_DeadTime        = 11;                          /* 死区时间 ~3us */
    TIM1_BDTRType.TIM_LOCKLevel       = TIM_LOCKLevel_OFF;           /* 不锁定寄存器 */
    TIM1_BDTRType.TIM_OSSIState       = TIM_OSSIState_Disable;
    TIM1_BDTRType.TIM_OSSRState       = TIM_OSSRState_Disable;
    TIM_BDTRConfig(TIM1, &TIM1_BDTRType);

    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);  /* 启动OC和OCN输出 (TIM1/TIM8必须) */
}
