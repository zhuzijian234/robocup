
#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif


//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
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
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	
#endif


//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){ //调试串口初始化						PA9-TX  PA10-RX  USART1	
   //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);                           //使能USART1DMA接收
	USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if 1																													//使能DMA接收后不开启UART的中断，使用DMA中断即可
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif
	
}

 

u8 USART_BUFFER = 0;
u8 USART_BUFFER_STA[USART_BUFFER_STA_Len];

void USART1_IRQHandler(void){
	static uint8_t RxState = 0;		//定义表示当前状态机状态的静态变量
	static uint8_t pRxPacket = 0;	//定义表示当前接收数据位置的静态变量
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
		uint8_t RxData = USART_ReceiveData(USART1);
		if (RxState == 0)
		{
			if (RxData == 0xFF)			//如果数据确实是包头
			{
				RxState = 1;			//置下一个状态
				
			}
		}
		else if (RxState == 1)
		{
			if (RxData == 0xFE)			//如果数据确实是包头2
			{
				RxState = 2;			//状态归0
				pRxPacket = 0;			//数据包的位置归零
			}
		}		
		/*当前状态为1，接收数据包数据*/
		else if (RxState == 2)
		{
			USART_BUFFER_STA[pRxPacket] = RxData;	//将数据存入数据包数组的指定位置
			pRxPacket ++;				//数据包的位置自增
			if (pRxPacket >= (USART_BUFFER_STA_Len))			//如果收够4个数据66-1=65
			{
				RxState = 0;			//置下一个状态
				pRxPacket=0;
				canshu_gengxin(USART_BUFFER_STA,USART_BUFFER_STA_Len);
				GPIO_ResetBits(GPIOF,GPIO_Pin_10);//灭红灯
				GPIO_ResetBits(GPIOF,GPIO_Pin_9);//灭红灯
				delay_ms(1000);
				GPIO_SetBits(GPIOF,GPIO_Pin_10);//灭红灯
				GPIO_SetBits(GPIOF,GPIO_Pin_9);//灭红灯
				delay_ms(1000);
				GPIO_ResetBits(GPIOF,GPIO_Pin_10);//灭红灯
				GPIO_ResetBits(GPIOF,GPIO_Pin_9);//灭红灯
				delay_ms(1000);
				GPIO_SetBits(GPIOF,GPIO_Pin_10);//灭红灯
				GPIO_SetBits(GPIOF,GPIO_Pin_9);//灭红灯
				delay_ms(1000);
			}
		}
			
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);		//清除标志位				
	}

}

void canshu_gengxin(u8 canshu[],u16 size)
{
	u16 i=0;
	float float_value;  
	// 确保数组有足够的字节来包含至少两个字节（用于Moto_pwm）和后续可能的浮点数  
    if (size < 6) { // 至少需要6个字节（2字节用于Moto_pwm，4字节用于至少一个浮点数）  
        printf("Error: Array size too small.\n");  
        return;  
    }  
  
    // 将前两个字节合并成u16类型的Moto_pwm  
    Moto_pwm = (uint16_t)canshu[0] | ((uint16_t)canshu[1] << 8);  
  
    // 打印Moto_pwm的值  
    printf("\r\nMoto_pwm: %d\n", Moto_pwm);  
  
    // 处理剩余的字节作为浮点数  
    // 注意：我们需要以4字节为一组来处理  
    for (i = 2; i < size; i += 4) {  
        // 检查是否有足够的字节来形成一个浮点数  
        if (i + 3 >= size) {  
            printf("Warning: Insufficient bytes for a float at index %u\n", i);  
            break;  
        }  
  
        // 使用数组索引和类型转换来提取浮点数（这里假设float是4字节且小端）  
       
        memcpy(&float_value, &canshu[i], sizeof(float)); // 更安全的转换方式  
		switch(((i - 2) / 4+1))
		{
			case 1:
				R_CL_a=float_value;
				break;
			case 2:
				R_CL_d=float_value;
				break;
			case 3:
				R_CL_kp=float_value;
				break;
			case 4:
				R_CL_kd=float_value;
				break;
			case 5:
				L_CL_a=float_value;
				break;
			case 6:
				L_CL_d=float_value;
				break;
			case 7:
				L_CL_kp=float_value;
				break;
			case 8:
				L_CL_kd=float_value;
				break;
			case 9:
				YZ_j_a=float_value;
				break;
			case 10:
				YZ_j_d=float_value;
				break;
			case 11:
				YZ_j_kp=float_value;
				break;
			case 12:
				YZ_j_kd=float_value;
				break;
			case 13:
				ZZ_j_a=float_value;
				break;
			case 14:
				ZZ_j_d=float_value;
				break;
			case 15:
				ZZ_j_kp=float_value;
				break;
			case 16:
				ZZ_j_kd=float_value;
				break;
			case 17:
				Duandian_d=float_value;
				break;
			case 18:
				zhuandian=float_value;
				break;
			default: break;
			
		}
        // 或者，如果你确定你的系统和编译器使用小端字节序，并且float是4字节的，  
        // 你可以使用联合体或直接的位操作，但这里为了清晰和可移植性，我们使用memcpy  
  
        // 处理或显示浮点数（这里只是打印）  
        printf("Float %u: %.2f\n", (i - 2) / 4, float_value); // 使用(i - 2) / 4作为浮点数的索引  
    }  
}

