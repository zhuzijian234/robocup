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
#include "ble_diag.h"
#include "moto.h"
#include "centre_line.h"
#include "m10p.h"

/* 控制许可由主循环提交，由TIM5最终仲裁；主循环不得直接绕过许可驱动电机。
 * 共享观测的修改使用短临界区，保证帧号、接收时间和epoch属于同一次观测。 */
volatile uint16_t Radar_age_ticks = 0;
volatile uint8_t Radar_started = 0;
volatile uint8_t Radar_stop_latched = 0;
volatile uint32_t Radar_timeout_count = 0;
volatile uint32_t Radar_invalid_inputs = 0;

static volatile uint32_t observation_us, observation_epoch, observation_seq, completed_us;
static volatile float observation_scale;
static volatile uint8_t observation_valid, warmup;
volatile float Radar_effective_target;
uint8_t Radar_Permitted(void)
{
    return Radar_started && !Radar_stop_latched && observation_valid &&
        observation_epoch == LidarRx_epoch &&
        (uint32_t)(Diag_TimeUs()-observation_us) <= M10P_MAX_AGE_US;
}
void Radar_Invalidate(void)
{
    /* 一次无效观测即撤销许可，同时清连续有效帧计数；恢复必须重新积累3帧。 */
    uint32_t p = __get_PRIMASK(); __disable_irq();
    observation_valid = 0; warmup = 0;
    __set_PRIMASK(p);
}
void Radar_Observe(uint32_t seq, uint32_t front_us, uint32_t epoch, float scale)
{
    /* 只有已完成几何/控制计算的新鲜扫描才能提交；串口收到字节不等于有效观测。 */
    uint32_t p = __get_PRIMASK(), now = Diag_TimeUs();
    __disable_irq();
    if (Radar_stop_latched || epoch != LidarRx_epoch ||
        (uint32_t)(now-front_us) > M10P_MAX_AGE_US || seq == observation_seq) {
        observation_valid = 0; warmup = 0;
    } else {
        if (seq != observation_seq + 1u || now - completed_us > M10P_MAX_AGE_US) warmup = 0;
        observation_seq = seq; observation_us = front_us; observation_epoch = epoch;
        completed_us = now;
        observation_scale = scale < 0 ? 0 : scale > 1 ? 1 : scale;
        if (warmup < 3) warmup++;
        observation_valid = warmup >= 3;
        if (observation_valid) { Radar_age_ticks = 0; Radar_started = 1; }
    }
    __set_PRIMASK(p);
}
void Radar_GuardTick(void)
{
    /* 即使主循环卡住，10ms定时环仍按接收时间检查过期，不依赖主循环喂字节。 */
    if (Radar_started && !Radar_stop_latched) {
        if (Radar_age_ticks < RADAR_TIMEOUT_TICKS) Radar_age_ticks++;
        if (Radar_age_ticks >= RADAR_TIMEOUT_TICKS) {
            Radar_stop_latched = 1; Radar_timeout_count++;
        }
    }
    if (observation_epoch != LidarRx_epoch ||
        (uint32_t)(Diag_TimeUs()-observation_us) > M10P_MAX_AGE_US) {
        observation_valid = 0; warmup = 0;
    }
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

/**
 * @brief  初始化TIM14为周期中断定时器（预留）
 */
#if 0 /* 未使用的预留定时器；M10P只使用TIM5速度环与TIM6诊断时钟 */
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
#endif /* M10P不编译上述历史实现 */

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

        uint8_t encoder_fresh=0,pi_fresh=0;
        extern uint16_t Encoder_cnt_temp;
        Radar_GuardTick();
        if (!Radar_started || Radar_stop_latched || !observation_valid) {
            /* Zero duty removes propulsion; it is not an active brake.
             * Skip PI so its integral cannot accumulate during inhibition. */
            Get_Encoder();encoder_fresh=1;
            Radar_effective_target = 0;
            Speed_PID_Reset(&Speed_pid);
            moto_pwm = 0;
            Moto_Speed(0);
        } else if (daoche_flag == 1) {
            TIM_SetCompare1(TIM2, (uint16_t)(100 * 0.5));  /* 50%制动,不足以驱动小车 */
        } else {
            Get_Encoder();encoder_fresh=1;
            {
                float requested = Speed_mubiao * observation_scale;
                /* Native encoder units, 0.25/tick acceleration; deceleration immediate. */
                if (requested > Radar_effective_target + 0.25f) requested = Radar_effective_target + 0.25f;
                Radar_effective_target = requested;
            }
            moto_pwm = PID_realize(Speed_now, Radar_effective_target, &Speed_pid);pi_fresh=1;
            Moto_Speed(moto_pwm);
        }
        Diag_MotorTick(Encoder_cnt_temp,encoder_fresh,pi_fresh);
    }
}
