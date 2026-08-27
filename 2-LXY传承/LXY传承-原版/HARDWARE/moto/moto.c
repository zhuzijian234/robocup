/**
 * @file    moto.c
 * @brief   直流电机驱动与编码器测速
 *
 * 硬件:
 *   电机方向: PE5(高=正转), PE4(低=使能)
 *   电机PWM:  TIM2 CH3, PA2, 84MHz/42/100 = 20kHz
 *   编码器:   TIM4 正交编码器, PD12/PD13
 *
 * 速度计算（10ms采样）:
 *   减速比:   6.25 : 1
 *   编码器PPR: 11
 *   公式:     Speed = (EncoderCNT均值 * 100) / (4 * 11 * 6.25)
 *   范围:     0 ~ 27（拟合单位）
 */

#include "stm32f4xx.h"
#include "PWM.h"

void Moto_Init(uint16_t psc, uint16_t arr, uint16_t puse)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_SetBits(GPIOE, GPIO_Pin_5);     /* PE5高 = 正转方向 */
    GPIO_ResetBits(GPIOE, GPIO_Pin_4);   /* PE4低 = 使能 */
    TIM2_PWM_Init(psc - 1, arr - 1, puse); /*启动pwm*/
}/*psc (预分频器):
  168MHz / psc = 定时器计数频率
  比如 psc = 84 → 168MHz / 84 = 2MHz（每 0.5us 计一次数）

arr (自动重载值):
  计数到 arr 后归零，决定 PWM 周期
  比如 arr = 100 → PWM 周期 = 100 × 0.5us = 50us → 20kHz

puse (初始占空比):
  计数器从 0 到 puse 期间输出高电平
  比如 puse = 50 → 50% 占空比（一半时间有电）*/

void Moto_Speed(uint16_t Compare)
{
    TIM_SetCompare3(TIM2, (uint16_t)Compare);
}

void Encoder_Init(void)
{
    TIM4_PWM_Init();
}

float Speed_now    = 0;
float Speed_mubiao = 0;

float Encoder_cnt          = 0;
uint16_t Encoder_cnt_arr[5];      /* 5采样滑动平均缓冲区 */
uint16_t Encoder_cnt_temp = 0;

uint16_t moto_pwm = 0;

/**
 * @brief  读取编码器并通过5采样滑动平均计算速度
 *
 * 速度公式:
 *   Speed = (EncoderCNT * 100) / (4 * PPR * 减速比)
 *         = (EncoderCNT * 100) / (4 * 11 * 6.25)
 *
 * 由TIM5中断服务函数每10ms调用一次。
 * 速度范围: 0 ~ 27（拟合比例）
 */
void Get_Encoder(void)
{
    uint16_t i;
    Encoder_cnt_temp = TIM_GetCounter(TIM4);
    TIM_SetCounter(TIM4, 0);
    Encoder_cnt = Encoder_cnt_temp; /*清除计数器*/

    for (i = 0; i < 5 - 1; i++) {
        Encoder_cnt_arr[i] = Encoder_cnt_arr[i + 1];
        Encoder_cnt += Encoder_cnt_arr[i];
    }
    Encoder_cnt_arr[i] = Encoder_cnt_temp;
    Encoder_cnt /= 5;
    Speed_now = (Encoder_cnt * 100) / (4 * 11 * 6.25);
}
/*
100     =10ms记一次,本来是x圈/10ms,方便计算,换算为x圈/秒


4       = 4 倍频
          TIM4 编码器模式对 A/B 两相的每个边沿都计数（上升沿+下降沿）
          所以编码器每输出 1 个完整脉冲周期，CNT 实际 +4

11      = 编码器 PPR（Pulses Per Revolution）
          编码器轴每转 1 圈，A 相输出 11 个完整脉冲
          配合 4 倍频 → 轴每转 1 圈，CNT 增加 4×11 = 44

6.25    = 减速比
          电机轴转 6.25 圈 → 轮子才转 1 圈,转速用圈/秒表示*/ 