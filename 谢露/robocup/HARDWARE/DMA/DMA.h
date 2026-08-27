#ifndef __DMA_H
#define __DMA_H

#include "sys.h" 


void DMA_Initializes(void);								//USART2->DR到DMA_USART2_RX_BUF		DMA_USART2_RX_BUF_LEN=3000  DMA_RX_DONE   DMA_USART2_RX_BUF_r[]这三个在主函数中会用			


#define DMA1_Stream5_IQ_ENABLE 1						//中断使能
#define DMA_USART2_RX_BUF_LEN 	1798 				//传输数目 2820	2914 2867		
extern u8 DMA_RX_DONE;
extern u8 DMA_USART2_RX_BUF[];
extern u8 DMA_USART2_RX_BUF_r[];
#endif









