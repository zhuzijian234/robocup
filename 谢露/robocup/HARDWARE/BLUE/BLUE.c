#include "BLUE.h"
#include "sys.h"

#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif

#if 0
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 


FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART6->SR&0X40)==0);//循环发送,直到发送完毕   
	USART6->DR = (u8) ch;      
	return ch;
}
#endif


void BLUE_init(u32 bound)  //PC6-TX  PC7-RX   USART6
	{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
 

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //PC6
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //PC7
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); 

   
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
	USART_Init(USART6, &USART_InitStructure); 
	USART_Cmd(USART6, ENABLE);                                                                            	
	USART_ClearFlag(USART6, USART_FLAG_TC);
	USART_ClearFlag(USART6, USART_FLAG_RXNE);
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);


	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	

}
	
u8 BLUE_change_sign=1;
u8 BLUE_BUFFER_STA = start;

void USART6_IRQHandler(void){
	u8 res = 0;
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET){
					res =USART_ReceiveData(USART6);
					if(res == 0xFF){
						
						BLUE_BUFFER_STA = stop;
						printf("\r\n\r\n\r\nstop\r\n\r\n\r\n\r\n\r\n\r\n");
						
					}					
					else if(res == 0xEE){
						
						BLUE_BUFFER_STA = start;
						printf("\r\n\r\n\r\nstart\r\n\r\n\r\n\r\n\r\n\r\n");
					}
					BLUE_change_sign=1;
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);				
	}

}


