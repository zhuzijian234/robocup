/**
 * @file    PWM.c
 * @brief   通用定时器PWM输出与编码器接口驱动
 *
 * 本项目定时器分配（对应谢露版引脚复用）:
 *   TIM2  CH4 (PB11):  电机PWM
 *   TIM3  CH1 (PA6):   舵机PWM, 同时用作中断定时器
 *   TIM4  (PD12/13):   正交编码器接口
 *   TIM9  CH1 (PA2):   雷达电机转速PWM (1kHz)
 *   TIM10 CH1 (PF6):   通用PWM
 *   TIM11 CH1 (PB9):   通用PWM
 */

#include "stm32f4xx.h"
#include "PWM.h"
#include "math.h"

uint16_t b;

/* ======================== 定时器中断初始化 ======================== */

/**
 * @brief  初始化TIM3为周期中断定时器
 */
void TIM3_Int_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period        = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler     = psc;
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up; /*计数方向*/
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; /*数字滤波器用的时钟分频,对基本定时功能无影响*/

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); /*使能更新中断*/
    TIM_Cmd(TIM3, ENABLE); /*启动定时器*/

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* ======================== PWM输出初始化 ======================== */

/**
 * @brief  TIM11 CH1 PWM初始化 (PB9)
 */
void TIM11_PWM_Init(u32 psc, u32 arr, u32 pulse)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM11); /*引脚复用映射*/

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;           /* PB9 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;        /*复用模式*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   /*推挽输出*/
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;   /*内部上拉*/
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler       = psc;
    TIM_TimeBaseStructure.TIM_CounterMode     = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period          = arr;
    TIM_TimeBaseStructure.TIM_ClockDivision   = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM11, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse       = pulse;
    TIM_OC1Init(TIM11, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM11, ENABLE);
    TIM_Cmd(TIM11, ENABLE);
}

/**
 * @brief  TIM2 CH4 PWM初始化 (PB11) — 电机PWM
 *
 * 对应谢露版: 电机PWM从PA2(TIM2_CH3)移至PB11(TIM2_CH4)
 */
void TIM2_PWM_Init(u32 psc, u32 arr, u32 pulse)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;          /* PB11 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler       = psc;
    TIM_TimeBaseStructure.TIM_CounterMode     = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period          = arr;
    TIM_TimeBaseStructure.TIM_ClockDivision   = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* TIM2 CH4 PWM模式1 */
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; /*使能比较输出,使比较结果输出到引脚上*/
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;   /*输出极性=高有效*/
    TIM_OCInitStructure.TIM_Pulse       = pulse;                 /*初始比较值,即ccr*/
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);

    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
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
    TIM_TimeBaseStructure.TIM_Period          = 65535 - 1;
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

/* ======================== 雷达电机PWM ======================== */

/**
 * @brief  雷达电机PWM初始化 (TIM9 CH1, PA2)
 *
 * 对应谢露版: 雷达电机PWM从TIM9_CH2(PE6)移至TIM9_CH1(PA2)
 * 时钟: 168MHz, 预分频: 1680-1 -> 100kHz计数
 * 周期: 100-1 -> 1kHz PWM
 *
 * ⚠ PA2同时被USART2_TX使用，通过main中的初始化顺序保证
 *   (USART2先初始化, TIM9后初始化覆盖PA2的AF)。
 *   雷达只需DMA接收(RX on PA3)，USART2_TX不需要。
 */
void PWM_Init_leida(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM9);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;           /* PA2 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler       = 1680 - 1;  /* 168MHz / 1680 = 100kHz */
    TIM_TimeBaseStructure.TIM_CounterMode     = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period          = 100 - 1;   /* 100kHz / 100 = 1kHz */
    TIM_TimeBaseStructure.TIM_ClockDivision   = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

    /* TIM9 CH1 PWM模式1 */
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OC1Init(TIM9, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM9, ENABLE);
    TIM_Cmd(TIM9, ENABLE);
}

void PWM_SetCompare_leida(uint16_t Compare)
{
    TIM_SetCompare1(TIM9, Compare);
}
