#include "DMA.h"


u8 DMA_USART2_RX_BUF[DMA_USART2_RX_BUF_LEN];
u8 DMA_USART2_RX_BUF_r[DMA_USART2_RX_BUF_LEN];
u8 DMA_RX_DONE = 0;


//USART2->DR到DMA_USART2_RX_BUF		DMA_USART2_RX_BUF_LEN=3000  DMA_RX_DONE DMA_USART2_RX_BUF_r[]这三个在主函数中会用			
void DMA_Initializes(void)		
{ 
 
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能

  	DMA_DeInit(DMA1_Stream5);
	while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);//等待DMA可配置 
  	/* ?? DMA1 Stream5,USART2接收 */
	DMA_InitStructure.DMA_Channel            = DMA_Channel_4;               //通道4
  	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR;            //DMA外设地址   传输到串口一的dr寄存器
  	DMA_InitStructure.DMA_Memory0BaseAddr    = (u32)DMA_USART2_RX_BUF;      //DMA 存储器0地址
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;  //外设到存储器模式
  	DMA_InitStructure.DMA_BufferSize         = DMA_USART2_RX_BUF_LEN;       //数据传输量
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;   //外设非增量
  	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;        //存储器增量
  	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8位
  	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;     
  	DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;             //普通模式
  	DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;         //中等优先级
  	DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;         
  	DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
  	DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;      //存储器突发单次传输
  	DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;  //外设突发单次传输
  	DMA_Init(DMA1_Stream5, &DMA_InitStructure);                             
	
#if DMA1_Stream5_IQ_ENABLE
	DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);							//DMA1传输完成中断

	NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Stream5_IRQn ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;                 //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;		          //子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;			  //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);
#endif

		DMA_Cmd(DMA1_Stream5, ENABLE);
}

#if DMA1_Stream5_IQ_ENABLE

void DMA1_Stream5_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_Stream5,DMA_FLAG_TCIF5)!=RESET)//&&DMA_RX_DONE == 0
	{
				DMA_Cmd(DMA1_Stream5, DISABLE);             //关闭DMA，防止处理期间有数据   

									memcpy(DMA_USART2_RX_BUF_r,DMA_USART2_RX_BUF,	DMA_USART2_RX_BUF_LEN);	
									DMA_RX_DONE = 1;			
								
    	DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5 | DMA_FLAG_FEIF5 
    	| DMA_FLAG_DMEIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5);		//清除DMA传输完成标志
    	DMA_SetCurrDataCounter(DMA1_Stream5, DMA_USART2_RX_BUF_LEN);			//重新设置DMA传输数据量	
    	
								DMA_Cmd(DMA1_Stream5, ENABLE);
		
	}
}
#endif







