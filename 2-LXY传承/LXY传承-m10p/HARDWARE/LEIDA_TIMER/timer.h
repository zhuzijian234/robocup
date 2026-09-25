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

/* TIM5每10ms仲裁一次电机输出；500ms无有效观测后锁存停机，复位才能清除。 */
#define RADAR_TIMEOUT_TICKS 50u
extern volatile uint16_t Radar_age_ticks;
extern volatile uint8_t Radar_started;
extern volatile uint8_t Radar_stop_latched;
extern volatile uint32_t Radar_timeout_count;
extern volatile uint32_t Radar_invalid_inputs;
uint8_t Radar_Permitted(void);
void Radar_Invalidate(void);
void Radar_Observe(uint32_t seq, uint32_t front_us, uint32_t epoch, float scale);
extern volatile float Radar_effective_target;
void Radar_GuardTick(void);

void TIM5_Int_Init(u16 arr, u16 psc);

#endif
