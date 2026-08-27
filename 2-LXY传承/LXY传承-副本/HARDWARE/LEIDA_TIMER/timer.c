/**
 * @file    timer.c
 * @brief   系统定时器模块 — 定时测速与速度PID控制
 *
 * TIM5: 10ms定时中断
 *   时钟: 84MHz / 8400预分频 = 10kHz计数
 *   周期: 100 -> 100/10000 = 10ms
 *   每10ms: 读编码器 -> 计算速度 -> 位置式PI -> 更新电机PWM
 *
 * TIM14: 辅助定时器（预留）
 */

#include "timer.h"
#include "moto.h"
#include "centre_line.h"

/**
 * @brief  初始化TIM5为10ms周期中断定时器
 * @param  arr: 自动重装载值 (100-1 = 99, 对应10ms@10kHz)
 * @param  psc: 预分频值 (8400-1 = 8399, 84MHz/8400 = 10kHz)
 */
void TIM5_Int_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period          = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler       = psc;
    TIM_TimeBaseInitStructure.TIM_CounterMode     = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision   = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM5, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  初始化TIM14为周期中断定时器（预留）
 */
void TIM14_Int_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period          = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler       = psc;
    TIM_TimeBaseInitStructure.TIM_CounterMode     = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision   = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM14, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM14, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM8_TRG_COM_TIM14_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

uint16_t daoche_flag     = 0;  /* 倒车标志 */
uint8_t  ENCODER_TIM     = 0;
uint8_t  TIM_IRQ_COUNTER = 0;

/**
 * @brief  TIM5中断服务函数 — 主速度控制循环（10ms周期）
 *
 * 每次中断:
 *   1. 如果倒车标志=1: 施加50%制动PWM
 *   2. 否则: 读编码器 -> 计算速度 -> 位置式PI -> 更新电机PWM
 */
void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET) {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

        if (daoche_flag == 1) {
            TIM_SetCompare4(TIM2, (uint16_t)(100 * 0.5));  /* 50%制动,不足以驱动小车 */
        } else {
            Get_Encoder();
            moto_pwm = PID_realize(Speed_now, Speed_mubiao, &Speed_pid);
            Moto_Speed(moto_pwm);
        }
    }
}
