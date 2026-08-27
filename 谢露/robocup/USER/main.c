#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "Test.h"	
#include "DMA.h"
#include "LEIDA_DATA.h"
#include "Leida_usart.h"
#include "BLUE.h"
#include "lwdg.h"
#include "timer.h"
#include "moto.h"
#include "Servo.h"
#include "PWM.h"
#include "PWMI.h"
#include "led.h" 



int main(void)
{
	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(84);
	
//	BLUE_init(115200);		//蓝牙初始化							PC6-TX  PC7-RX   USART6
	uart_init(115200);		//调试串口初始化						PA9-TX  PA10-RX  USART1	
	
	uart2_init(230400);		//雷达初始化							PA2-TX  PA3-RX   USART2     只用PA3，也可以试试PA2 TIM2_CH3 调速
	DMA_Initializes();		//USART2->DR到DMA_USART2_RX_BUF		DMA_USART2_RX_BUF_LEN=3000  DMA_RX_DONE DMA_USART2_RX_BUF_r[]这三个在主函数中会用			
	
	Servo_Init(84,20000,servo_Midpwm1);//舵机初始化 servo_midpwm = 1550  50hz 20ms		PA6        TIM3_CH1    pwm0,5~2.5ms 90度：pwm=1.5ms puse=150（根据实际情况浮动）puse:50~250 
	
//	DIR_Init();						  //电机方向初始化,主函数不用初始化，moto里有		PB10
	Moto_Init(1,4200,0);		  //电机初始化 moto_pwm = 0  20kHZ     			PB11 	   TIM2_CH4
	
	//PWMI_Init();				      //PWMI检测口 									PB6		   TIM4_CH1

	Encoder_Init();					  //编码器初始化									PD12 PD13  TIM4_CH3 TIM4_CH4	
	TIM5_Int_Init(100-1,8400-1);	  //10ms进一次中断
	LED_Init();
	IWDG_Init(4,100); //与分频数为64,重载值为50,溢出时间为1s	
	

//测试程序
	
//	while(1){
//		
//		
//		printf("测试Servo_f= %d\r\n",IC_GetFreq());//PA6 duoji
//		printf("测试Servo_DUTY= %d\r\n\r\n",IC_GetDuty());
//		
//		
//		
//	}


#if 1
	Handle3_TEST(); //
#endif


	while(1){
		
		printf("ok");
		delay_ms(10);
	
	}
}





