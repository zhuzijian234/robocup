#include "leida_pwm.h"

void Leida_PWM_Init(uint16_t psc,uint16_t arr,uint16_t puse)
{
	TIM9_PWM_Init(psc-1,arr-1,puse);//113 183
}

void TIM9_PWM_Init(u32 psc,u32 arr,u32 pulse)
{		 					 
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  	//TIM3时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM9); //GPIOA6复用为定时器3
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;           //GPIOA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        //不拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //初始化PA6
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_OCInitStructure.TIM_Pulse = pulse;
	TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);//初始化定时器3
	
	//初始化TIM3 Channe1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM9, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC1

	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM9,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM9, ENABLE);  //使能TIM3
   

} 
