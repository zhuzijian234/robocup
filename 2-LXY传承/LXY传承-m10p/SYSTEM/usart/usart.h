#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "stm32f4xx_conf.h"
#include "sys.h"

#define USART_REC_LEN 200 /* 最大接收字节数 200 */
#define EN_USART1_RX  0   /* 使能（1）/禁止（0）串口1接收 */

extern u8 USART_RX_BUF[USART_REC_LEN]; /* 接收缓冲, 最大USART_REC_LEN个字节. 末字节为换行符 */
extern u16 USART_RX_STA;               /* 接收状态标记 */
/* 如果想用串口中断接收，请不要注释以下宏定义 */
void uart_init(u32 bound);
void uart2_init(u32 bound); /* USART2初始化 — 雷达串口 (PA2=TX, PA3=RX) */
void uart3_init(u32 bound); /* USART3初始化 — 调试串口 (PC10=TX, PC11=RX), printf重定向目标 */
#endif
