/**
 * @file    timer.h
 * @brief   系统定时器模块 — 定时测速与速度PID计算
 *
 * TIM5: 10ms定时中断，读取编码器 -> 计算速度 -> 位置式PI -> 更新电机PWM
 * TIM14: 辅助定时器（预留）
 */

#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
#include "DMA.h"

extern uint8_t ENCODER_TIM;
extern uint8_t TIM_IRQ_COUNTER;
extern uint16_t moto_pwm;
extern uint16_t daoche_flag;   /* 倒车标志 */

/* TIM5 is configured for 10ms. Fault is latched until MCU reset. */
#define RADAR_TIMEOUT_TICKS 50u
extern volatile uint16_t Radar_age_ticks;
extern volatile uint8_t Radar_started;
extern volatile uint8_t Radar_stop_latched;
extern volatile uint32_t Radar_timeout_count;
extern volatile uint32_t Radar_invalid_inputs;
void Radar_ControlCompleted(void);
extern volatile uint8_t Radar_drive_enabled; /* 诊断采集可临时禁止驱动，不解除锁停 */
extern volatile float Speed_effective; /* 真正送入PI的速度，Speed_mubiao仍是用户请求 */
extern volatile uint8_t Radar_limit_reason;
void Radar_SetSpeedLimit(float limit, uint8_t reason);
void Radar_GuardTick(void);

void TIM5_Int_Init(u16 arr, u16 psc);
void TIM14_Int_Init(u16 arr, u16 psc);

#endif
