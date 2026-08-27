#include "stm32f4xx.h"                  // Device header
#include "PWM_PN_OUT.h"                  // Device header


//高级定时器TIM1  1通道   PWM互补输出
//PWM_P    GPIOA8
//PWM_N    GPIOA7

void TIM_PWM_PN_Init(u16 psc,u16 arr)
{
    
    GPIO_InitTypeDef         GPIO_PWMInit;
    
    TIM_TimeBaseInitTypeDef  TIM1_TIMERType;
    
    TIM_OCInitTypeDef        TIM1_PWMOC;   //选择第一通道
    
    TIM_BDTRInitTypeDef      TIM1_BDTRType;
    
    //启动对应RCC时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
    
    //复用引脚///
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM1);
    
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM1);
    
    
    //初始化定义
    GPIO_PWMInit.GPIO_Mode  = GPIO_Mode_AF;
    
    GPIO_PWMInit.GPIO_OType = GPIO_OType_PP;
    
    GPIO_PWMInit.GPIO_Pin   = GPIO_Pin_7;
    
    GPIO_PWMInit.GPIO_Speed = GPIO_Speed_100MHz;
    
    GPIO_PWMInit.GPIO_PuPd  = GPIO_PuPd_UP;
    
    GPIO_Init(GPIOA,&GPIO_PWMInit);
		
     //初始化定义
    GPIO_PWMInit.GPIO_Mode  = GPIO_Mode_AF;
    
    GPIO_PWMInit.GPIO_OType = GPIO_OType_PP;
    
    GPIO_PWMInit.GPIO_Pin   = GPIO_Pin_8;
    
    GPIO_PWMInit.GPIO_Speed = GPIO_Speed_100MHz;
    
    GPIO_PWMInit.GPIO_PuPd  = GPIO_PuPd_UP;
    
    GPIO_Init(GPIOA,&GPIO_PWMInit);
    
    
    //定时器初始化定义/
    TIM1_TIMERType.TIM_ClockDivision = TIM_CKD_DIV1;
    
    TIM1_TIMERType.TIM_Period        = arr;   //自动重装载值
    
    TIM1_TIMERType.TIM_Prescaler     = psc;   //分频系数
    
    TIM1_TIMERType.TIM_CounterMode   =  TIM_CounterMode_Up; //向上计数模式
    
    TIM_TimeBaseInit(TIM1,&TIM1_TIMERType);
    
    
    
    //PWM初始化配置/
    TIM1_PWMOC.TIM_OCMode       = TIM_OCMode_PWM1; //使用脉冲调制宽度模式1
    
    TIM1_PWMOC.TIM_Pulse        = 1000; //占空比 取值必须在0x0000到0xFFFF
    
    TIM1_PWMOC.TIM_OCIdleState  = TIM_OCIdleState_Reset; //输出空闲时为低电平
    
    TIM1_PWMOC.TIM_OutputState  = TIM_OutputState_Enable; //输出使能
    
    TIM1_PWMOC.TIM_OCPolarity   = TIM_OCPolarity_High; //输出极性高
    
    TIM1_PWMOC.TIM_OutputNState = TIM_OutputNState_Enable; //互补输出打开
    
    TIM1_PWMOC.TIM_OCNIdleState = TIM_OCNIdleState_Reset;  //互补输出空闲时为低电平
    
    TIM1_PWMOC.TIM_OCNPolarity  = TIM_OCNPolarity_High;   //互补输出极性为高
    
    TIM_OC1Init(TIM1,&TIM1_PWMOC);
    
    TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
    ///
    
    
    //刹车死区配置///
    
    TIM1_BDTRType.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;  //自动输出功能使能
    
    TIM1_BDTRType.TIM_Break           = TIM_Break_Disable;           //失能刹车输入
    
    TIM1_BDTRType.TIM_BreakPolarity   = TIM_BreakPolarity_High;      //刹车输入管脚极性高
    
    TIM1_BDTRType.TIM_DeadTime        = 11;                        //死区时间配置 参考CSDN计算方法,这里是3us
    
    TIM1_BDTRType.TIM_LOCKLevel       = TIM_LOCKLevel_OFF;           //锁电平参数：不锁任何位
		
    TIM1_BDTRType.TIM_OSSIState       = TIM_OSSIState_Disable;       //设置在运行模式下非工作状态
		
    TIM1_BDTRType.TIM_OSSRState       = TIM_OSSRState_Disable;       //设置在运行模式下非工作状态
		
    TIM_BDTRConfig(TIM1,&TIM1_BDTRType);  
    ///
    
    TIM_ARRPreloadConfig(TIM1,ENABLE);                              //ARPE使能
    
    TIM_Cmd(TIM1,ENABLE);                                           //使能TIM1
    
    TIM_CtrlPWMOutputs(TIM1,ENABLE);                                //开启OC和OCN输出
    
}
 

//由于使能定时器通道1  故改变占空比的函数为
//void TIM_SetCompare1(TIM_TypeDef* TIMx, uint32_t Compare1);
