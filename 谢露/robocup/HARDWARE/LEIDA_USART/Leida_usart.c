#include "sys.h"
#include "Leida_usart.h"	


#if SYSTEM_SUPPORT_OS
#include "includes.h"					
#endif

#if 0
#pragma import(__use_no_semihosting)             
               
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       

void _sys_exit(int x) 
{ 
	x = x; 
} 


//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
	USART2->DR = (u8) ch;      
	return ch;
}
#endif
 


#if EN_USART2_RX   //如果使能了接收
//串口1中断服务程序
 	
u8 USART2_RX_BUF[USART2_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART2_RX_STA=0;       //接收状态标记	
#endif

//初始化调速


//初始化IO 串口1 
//bound:波特率
void uart2_init(u32 bound){//雷达初始化		PA2-TX  PA3-RX   USART2     只用PA3，也可以试试PA2 TIM2_CH3 调速

#if 1
   //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
	
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART1
	
	
	//USART2端口配置 rx pa3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; //GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能？？？？？？？？？？？？？？？
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA3	

	//USART2端口配置 tx pa2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; //GPIOA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉  LD14P 的 Rx 引脚下拉接地，LD14P 上电启动后，
													//LD14P 将进入内部控速模式，默认扫描频率为 6Hz。	
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA2
   //USART2 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//
	USART_Init(USART2, &USART_InitStructure); //初始化串口2
	
	USART_Cmd(USART2, ENABLE);  //使能串口2 
	
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);                           //使能USART2DMA接收
	USART_ClearFlag(USART2, USART_FLAG_TC);
#endif

#if 0																													//使能DMA接收后不开启UART的中断，使用DMA中断即可
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;	//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							

#endif
	
}

#if EN_USART2_RX	
void USART2_IRQHandler(void)                	
{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART2);//(USART2->DR);	//读取接收到的数据
		
		if((USART2_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART2_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART2_RX_STA=0;//接收错误,重新开始
				else USART2_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART2_RX_STA|=0x4000;
				else
				{
					USART2_RX_BUF[USART2_RX_STA&0X3FFF]=Res ;
					USART2_RX_STA++;
					if(USART2_RX_STA>(USART2_REC_LEN-1))USART2_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}   		 
  } 
#if SYSTEM_SUPPORT_OS 	
	OSIntExit();  											 
#endif
} 
#endif	

 



