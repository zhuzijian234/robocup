#include "stm32f4xx.h"                  // Device header
#include "PWMI.h"                  // Device header
#include "math.h" 

//PWMI检测口 									PB6		   TIM4_CH1
void PWMI_Init()  
{  
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
    TIM_ICInitTypeDef TIM_ICInitStructure;  
    GPIO_InitTypeDef GPIO_InitStructure;  
  
    // 使能TIM3和GPIOD的时钟  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  
  
    // 初始化GPIO为复用推挽输出（虽然这里是输入，但复用功能需要设置为推挽）  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 注意这里使用PP，因为AF模式通常需要PP或OD  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // 假设PD14用于编码器输入  
    GPIO_Init(GPIOB, &GPIO_InitStructure);  
  
    // 配置GPIO复用功能为TIM3  
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);  
  
    // 初始化TIM3时间基础设置  
    TIM_TimeBaseStructure.TIM_Prescaler = 84-1; // 预分频器  
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数  
    TIM_TimeBaseStructure.TIM_Period = 65535; // 自动重载寄存器周期  
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分频  
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);  
  
    // 初始化TIM3的输入捕获  
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; // 选择通道1  
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; // 上升沿捕获  
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // 直接连接到输入捕获  
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // 输入捕获预分频器  
    TIM_ICInitStructure.TIM_ICFilter = 0x0; // 输入捕获滤波器  
//    TIM_ICInit(TIM4, &TIM_ICInitStructure);  
	
	TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);						//将结构体变量交给TIM_PWMIConfig，配置TIM3的输入捕获通道
																	//此函数同时会把另一个通道配置为相反的配置，实现PWMI模式

  
	TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1);					//触发源选择TI1FP1

    TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);					//从模式选择复位
																	//即TI1产生上升沿时，会触发CNT归零
	
    // 启用TIM3  
    TIM_Cmd(TIM4, ENABLE);  
//  
   
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
//    TIM_ICInitTypeDef TIM_ICInitStructure;  
//    GPIO_InitTypeDef GPIO_InitStructure;  
//  
//    // 使能TIM3和GPIOD的时钟  
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);  
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);  
//  
//    // 初始化GPIO为复用推挽输出（虽然这里是输入，但复用功能需要设置为推挽）  
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 注意这里使用PP，因为AF模式通常需要PP或OD  
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; // 假设PD14用于编码器输入  
//    GPIO_Init(GPIOE, &GPIO_InitStructure);  
//  
//    // 配置GPIO复用功能为TIM3  
//    GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);  
//  
//    // 初始化TIM3时间基础设置  
//    TIM_TimeBaseStructure.TIM_Prescaler = 84-1; // 预分频器  
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数  
//    TIM_TimeBaseStructure.TIM_Period = 65535; // 自动重载寄存器周期  
//    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分频  
//    TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);  
//  
//    // 初始化TIM3的输入捕获  
//    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; // 选择通道1  
//    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; // 上升沿捕获  
//    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // 直接连接到输入捕获  
//    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // 输入捕获预分频器  
//    TIM_ICInitStructure.TIM_ICFilter = 0xF; // 输入捕获滤波器  
////    TIM_ICInit(TIM9, &TIM_ICInitStructure);  
//	
//	TIM_PWMIConfig(TIM9, &TIM_ICInitStructure);						//将结构体变量交给TIM_PWMIConfig，配置TIM3的输入捕获通道
//																	//此函数同时会把另一个通道配置为相反的配置，实现PWMI模式

//  
//	TIM_SelectInputTrigger(TIM9, TIM_TS_TI1FP1);					//触发源选择TI1FP1

//    TIM_SelectSlaveMode(TIM9, TIM_SlaveMode_Reset);					//从模式选择复位
//																	//即TI1产生上升沿时，会触发CNT归零
//	
//    // 启用TIM3  
//    TIM_Cmd(TIM9, ENABLE);  
  
    // （可选）设置TIM3中断（如果需要的话）  
    // TIM_ITConfig(TIM3, TIM_IT_Update | TIM_IT_CC1, ENABLE);  
    // NVIC_InitTypeDef NVIC_InitStructure;  
    // NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;  
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;  
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    // NVIC_Init(&NVIC_InitStructure);  
}

/**
  * 函    数：获取输入捕获的频率
  * 参    数：无
  * 返 回 值：捕获得到的频率
  */
uint32_t IC_GetFreq(void)
{
	return 1000000 / (TIM_GetCapture1(TIM4) + 1);		//测周法得到频率fx = fc / N，这里不执行+1的操作也可
}

/**
  * 函    数：获取输入捕获的占空比
  * 参    数：无
  * 返 回 值：捕获得到的占空比
  */
uint32_t IC_GetDuty(void)
{
	return (TIM_GetCapture2(TIM4) + 1) * 100 / (TIM_GetCapture1(TIM4) + 1);	//占空比Duty = CCR2 / CCR1 * 100，这里不执行+1的操作也可
}

