#include "stm32f4xx.h"    
#include "PWM.h"
#include "moto.h"

void DIR_Init()//电机方向初始化								PB10
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Speed=GPIO_Fast_Speed;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
//	GPIO_ResetBits(GPIOB,GPIO_Pin_10);
	GPIO_SetBits(GPIOB,GPIO_Pin_10);
}


void Moto_Init(uint16_t psc,uint16_t arr,uint16_t puse)//电机初始化 		PB11 	   TIM2_CH4
{
	DIR_Init();
	TIM2_PWM_Init(psc-1,arr-1,puse);
	
}



void Moto_Speed(uint16_t Compare)//改上面的函数的时候记得改这个函数
{
			TIM_SetCompare4(TIM2, (uint16_t)Compare);
	
}

void Encoder_Init()//编码器初始化			PD12 PD13  TIM4_CH3 TIM4_CH4
{
  TIM4_PWM_Init();
}

float Speed_now = 0;
u16 Moto_pwm=0;

void Get_Encoder()
{
	static float Encoder_cnt = 0;
	static uint16_t Encoder_cnt_arr[5];			//5个取一次均值
	static uint16_t Encoder_cnt_temp = 0;
	uint16_t i; 
	
	Encoder_cnt_temp = TIM_GetCounter(TIM4);
	
	TIM_SetCounter(TIM4,0);
	Encoder_cnt = Encoder_cnt_temp;
	
	for(i=0;i<5-1;i++){
			Encoder_cnt_arr[i] = Encoder_cnt_arr[i+1];
			Encoder_cnt +=Encoder_cnt_arr[i];
	}
	Encoder_cnt_arr[i] = Encoder_cnt_temp;
	
	Encoder_cnt /=5;
	Speed_now = Encoder_cnt/11*100;//240;    //单位转每分钟，(encoder_cnt/ppr/4/ratio)*(1000/cnt_time)*60  转速 =Encoder_cnt/11/4/6.25*1000*60/10=Encoder_cnt*240/11
}

