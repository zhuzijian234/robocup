/**
 * @file PWM.c
 * @brief M10P小车执行器驱动：电机PWM、舵机PWM及编码器。
 * TIM2_CH2/PA1：电机；TIM3_CH1/PA6：舵机；TIM4/PD12、PD13：编码器。
 * M10P内部驱动雷达电机，本模块只初始化小车执行器。
 */
#include "stm32f4xx.h"
#include "PWM.h"

/**
 * @brief  TIM2 CH2 PWM初始化 (PA1) — 电机PWM
 *
 * M144Z-M4 Mini Board: PA1直接引出到JP1第5脚，使用AF1映射TIM2_CH2。
 * 占空比写入和诊断读取同步使用CCR2；定时器时钟及PWM频率保持不变。
 */
void TIM2_PWM_Init(u32 psc, u32 arr, u32 pulse)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;            /* PA1 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler       = psc;
    TIM_TimeBaseStructure.TIM_CounterMode     = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period          = arr;
    TIM_TimeBaseStructure.TIM_ClockDivision   = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* TIM2 CH2 PWM模式1；PA1高电平有效，比较值为0时保持低电平。 */
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; /*使能比较输出,使比较结果输出到引脚上*/
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;   /*输出极性=高有效*/
    TIM_OCInitStructure.TIM_Pulse       = pulse;                 /*初始比较值,即ccr*/
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);

    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief  TIM3 CH1 PWM初始化 (PA6) — 舵机PWM
 */
void TIM3_PWM_Init(u32 psc, u32 arr, u32 pulse)
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
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler       = psc;
    TIM_TimeBaseStructure.TIM_CounterMode     = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period          = arr;
    TIM_TimeBaseStructure.TIM_ClockDivision   = TIM_CKD_DIV1;
    TIM_OCInitStructure.TIM_Pulse             = pulse;
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

/**                           CH1(TI1)  CH2(TI2)
 * @brief  TIM4 编码器模式初始化 (PD12, PD13) — 正交编码器
 */
void TIM4_PWM_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);

    TIM_TimeBaseStructure.TIM_Prescaler       = 0;
    TIM_TimeBaseStructure.TIM_CounterMode     = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period          = 65535;
    TIM_TimeBaseStructure.TIM_ClockDivision   = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /* 输入捕获通道初始化 — 双通道数字滤波抑制噪声 */
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 0xf;        /* CH1 (PD12, A相): 8次采样一致才有效 */
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);        /* CH2 (PD13, B相): 8次采样一致才有效 */

    /* 正交编码器模式: TI1和TI2双边沿均计数（4倍频），必须在ICInit之后配置 */
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
}
