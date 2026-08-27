#include "timer.h"



void TIM5_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  ///使能TIM5时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);//初始化TIM5
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); //允许定时器5更新中断
	TIM_Cmd(TIM5,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}






u16 daoche_Speed=1500;
u16 ZC_speed=2000;


//定时器5中断服务函数		用来控制电机速度的中断函数
void TIM5_IRQHandler(void)
{

	
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET) //溢出中断
	{
		if(BLUE_change_sign==1){
			
			if(BLUE_BUFFER_STA ==stop) Moto_pwm=0;
			else {
				
				Moto_pwm=3300;//8.5v
			}
			BLUE_change_sign=0;
		}
		if(y_z_daoche_flag==youdao)
		{
			daoche(daoche_Speed,500,youdao,ZC_speed);
			y_z_daoche_flag=budao;
		}else if(y_z_daoche_flag==zuodao)
		{
			daoche(daoche_Speed,500,zuodao,ZC_speed);
			y_z_daoche_flag=budao;
		}else {
				Get_Encoder();
				Moto_Speed(Moto_pwm);	
					
		}
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);  //清除中断标志位
	}
	
}


void daoche(uint16_t daoche_speed,uint16_t ms,u8 zuoyou,u16 ZC_speed)
{
	
	if(zuoyou==youdao)//左撞，右倒
	{
		GPIO_ResetBits(GPIOF,GPIO_Pin_9);//亮灯
		
//		Servo_ChangePwm(servo_left);
//		Moto_Speed(daoche_speed);
//		GPIO_ResetBits(GPIOB,GPIO_Pin_10);
//		delay_ms(ms);
		
		Servo_ChangePwm(servo_right);
		Moto_Speed(ZC_speed);
		delay_ms(ms);
		
		GPIO_SetBits(GPIOF,GPIO_Pin_9);//灭灯
	}else
	{
		GPIO_ResetBits(GPIOF,GPIO_Pin_9);//亮灯
		
//		Servo_ChangePwm(servo_right);
//		Moto_Speed(daoche_speed);
//		GPIO_ResetBits(GPIOB,GPIO_Pin_10);
//		delay_ms(ms);
		
		Servo_ChangePwm(servo_left);
		
		Moto_Speed(ZC_speed);
		delay_ms(ms);
		
		GPIO_SetBits(GPIOF,GPIO_Pin_9);//灭灯
	}	
	
}



//void TIM14_Int_Init(u16 arr,u16 psc)
//{
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);  ///使能TIM3时钟
//	
//  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
//	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
//	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM14,&TIM_TimeBaseInitStructure);//初始化TIM3
//	
//	TIM_ITConfig(TIM14,TIM_IT_Update,ENABLE); //允许定时器3更新中断
//	TIM_Cmd(TIM14,ENABLE); //使能定时器3
//	
//	NVIC_InitStructure.NVIC_IRQChannel=TIM8_TRG_COM_TIM14_IRQn; //定时器3中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	
//}



//void TIM8_TRG_COM_TIM14_IRQHandler(void)			//系统函数猜想和tim5中断函数差不多，所以选择注释掉
//{

//	if(TIM_GetITStatus(TIM14,TIM_IT_Update)==SET) //????
//	{
//		
//		
//    TIM_ClearITPendingBit(TIM14,TIM_IT_Update);  //???????
//	}
//	
//}
