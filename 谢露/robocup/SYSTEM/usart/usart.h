#ifndef __USART_H
#define __USART_H
#include "stdio.h"	

#include "sys.h" 
#include "LEIDA_DATA.h"
#include "moto.h"
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
#define	 USART_BUFFER_STA_Len   74
extern u8 USART_BUFFER ;
extern u8 USART_BUFFER_STA[];	 
//extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
//extern u16 USART_RX_STA;         		//接收状态标记	
void canshu_gengxin(u8 canshu[],u16 size);
void uart_init(u32 bound); 				//调试串口初始化						PA9-TX  PA10-RX  USART1	
#endif


