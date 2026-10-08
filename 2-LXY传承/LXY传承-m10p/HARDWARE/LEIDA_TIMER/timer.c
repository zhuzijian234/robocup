/**
 * @file    timer.c
 * @brief   系统定时器模块 — 定时测速与速度PID控制
 *
 * TIM5: 10ms定时中断
 *   时钟: 84MHz / 8400预分频 = 10kHz计数
 *   周期: 100 -> 100/10000 = 10ms
 *   每10ms: 读编码器 -> 计算速度 -> 位置式PI -> 更新电机PWM
 */

#include "timer.h"
#include "ble_diag.h"
#include "moto.h"
#include "centre_line.h"
#include "m10p.h"

/* 竞速模式：速度环独立运行，雷达只负责转向，不再仲裁电机输出。 */
volatile uint32_t Radar_invalid_inputs = 0;
volatile float Radar_effective_target;
static volatile uint32_t control_end_us;
static volatile uint8_t have_control;

/* 仅记录最近一次有效转向的来源时间，供蓝牙诊断，不影响速度环。 */
void Radar_RecordControl(uint32_t end_us)
{
    uint32_t p = __get_PRIMASK(); __disable_irq();
    control_end_us = end_us;
    have_control = 1;
    __set_PRIMASK(p);
}
uint32_t Radar_ControlAgeUs(void)
{
    return have_control ? (uint32_t)(Diag_TimeUs()-control_end_us) : 0xffffffffu;
}

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

/** 每10ms读取编码器并执行PI；近墙、坏帧、断流均不清零目标或积分。 */
void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET) {
        extern uint16_t Encoder_cnt_temp;
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
        Get_Encoder();
        Radar_effective_target = Speed_mubiao;
        moto_pwm = PID_realize(Speed_now, Radar_effective_target, &Speed_pid);
        Moto_Speed(moto_pwm);
        Diag_MotorTick(Encoder_cnt_temp, 1, 1);
    }
}
