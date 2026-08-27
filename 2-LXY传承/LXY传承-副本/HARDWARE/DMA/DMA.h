/**
 * @file    DMA.h
 * @brief   DMA驱动 — 雷达串口(USART2)高速数据接收
 *
 * 配置: DMA1 Stream5, Channel 4, 外设到内存, 单次模式
 * 每收到1798字节后触发传输完成中断，双缓冲后通知主循环处理。
 *
 * 对应谢露版引脚: USART2 (PA2=TX, PA3=RX)
 */

#ifndef __DMA_H
#define __DMA_H

#include "sys.h"
#include "timer.h"

void DMA_Initializes(void);

#define DMA1_Stream5_IQ_ENABLE 1     /* DMA传输完成中断使能 */
#define DMA_USART2_RX_BUF_LEN  1798  /* DMA接收缓冲区大小（字节），匹配谢露版 */

extern u8 DMA_RX_DONE;
extern u8 LEIDA_HANDLE_DONE;
extern u8 DMA_USART2_RX_BUF[];
extern u8 DMA_USART2_RX_BUF_r[];

#endif
