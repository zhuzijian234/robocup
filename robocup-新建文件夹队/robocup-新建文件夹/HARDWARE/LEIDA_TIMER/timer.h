/**
 * @file    timer.h
 * @brief   系统定时器模块 — 定时测速与速度PID计算
 *
 * TIM5: 10ms定时中断，读取编码器 -> 计算速度 -> 位置式PI -> 更新电机PWM
 */

#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
#include "DMA.h"

extern uint16_t moto_pwm;

/* 雷达诊断与电机控制相互独立，没有启动许可、超时停车或锁停状态。 */
extern volatile uint32_t Radar_invalid_inputs;
extern volatile float Radar_effective_target;
void Radar_RecordControl(uint32_t end_us);
uint32_t Radar_ControlAgeUs(void); /* 最近有效转向帧的年龄；无帧返回UINT32_MAX */

void TIM5_Int_Init(u16 arr, u16 psc);

#endif
