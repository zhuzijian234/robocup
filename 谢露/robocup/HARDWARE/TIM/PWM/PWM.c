#include "stm32f4xx.h"                  // Device header
#include "PWM.h"                  // Device header
#include "math.h" 


//电机初始化 		PB11 	   TIM2_CH4
//arr：自动重装值
//psc：时钟预分频数
void TIM2_PWM_Init(u32 psc,u32 arr,u32 pulse)
{		 					 
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM2时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_TIM2); //PB11复用为定时器2
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;           
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;     //不拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);              
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  					//定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;   //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   					//自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	//初始化TIM2 Channe4 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 				//选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 		//输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_Pulse = pulse;
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);  						//根据T指定的参数初始化外设TIM2 4OC1

	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);  				//使能TIM2在CCR1上的预装载寄存器
 
	TIM_ARRPreloadConfig(TIM2,ENABLE);								//ARPE使能 
	
	TIM_Cmd(TIM2, ENABLE);  										//使能TIM2
 						  
}  




//舵机初始化			PA6     TIM3_CH1    
//arr：自动重装值
//psc：时钟预分频数
void TIM3_PWM_Init(u32 psc,u32 arr,u32 pulse)
{		 					 
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM3时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3); //GPIOA6复用为定时器3
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;           //GPIOA6
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
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器3
	
	//初始化TIM3 Channe1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC1

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
   
} 

////定时器3中断服务函数如果要用需要在tim3初始化中加入一段优先分级初始化
//void TIM3_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
//	{

//	}
//	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
//}




//编码器初始化			PD12 PD13  TIM4_CH3 TIM4_CH4
void TIM4_PWM_Init()
{		 					 
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	   
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);    
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
	
	TIM_TimeBaseStructure.TIM_Prescaler=0;  
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period=65535-1;   
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	TIM_ICStructInit(&TIM_ICInitStructure); 
	TIM_ICInitStructure.TIM_ICFilter = 0xf;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	TIM_SetCounter(TIM4,0);

	TIM_Cmd(TIM4, ENABLE);  
   
}




////TIM11 PWM部分初始化(备用）   PB9 TIM11_CH1  /TIM4_CH4
////PWM输出初始化
////arr：自动重装值
////psc：时钟预分频数
//void TIM11_PWM_Init(u32 psc,u32 arr,u32 pulse)
//{		 					 
//	//此部分需手动修改IO口设置
//	
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);  	//TIM11时钟使能    
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTB时钟	
//	
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_TIM11); //GPIOB9复用为TIM11
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           //GPIOB9
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
//	GPIO_Init(GPIOB,&GPIO_InitStructure);              //初始化PB9
//	
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM11,&TIM_TimeBaseStructure);//初始化TIM11
//	
//	//初始化TIM11 Channe1 PWM模式	 
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
// 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
//	TIM_OCInitStructure.TIM_Pulse = pulse;
//	TIM_OC1Init(TIM11, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM11 OC1

//	TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Enable);  //使能TIM11在CCR1上的预装载寄存器
// 
//  TIM_ARRPreloadConfig(TIM11,ENABLE);//ARPE使能 
//	
//	TIM_Cmd(TIM11, ENABLE);  //使能TIM11
// 						  
//}  





//void SineWave_Data(u16 cycle, double* D, float Um)
//{
//	uint16_t i;
//	for (i = 0; i < cycle; i++)
//	{
//		D[i] = (double)(Um * arm_sin_f32((1.0 * i / (cycle - 1)) * 2 * PI));   //正弦离散化公式
//	}
//}

//void CosWave_Data(u16 cycle, double* D, float Um)
//{
//	uint16_t i;
//	for (i = 0; i < cycle; i++)
//	{
//		D[i] = (double)(Um * arm_cos_f32((1.0 * i / (cycle - 1)) * 2 * PI));   //正弦离散化公式
//	}
//}

//void Sine_N_Wave_Data(u16 cycle, double* D, float Um)
//{
//	uint16_t i;
//	for (i = 0; i < cycle; i++)
//	{
//		D[i] = (double)(Um * (-arm_sin_f32((1.0 * i / (cycle - 1)) * 2 * PI)));   //正弦离散化公式
//	}
//}


