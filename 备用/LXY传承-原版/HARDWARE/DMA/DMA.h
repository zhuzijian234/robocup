/**
 * @file    DMA.h
 * @brief   DMA驱动 — 雷达串口(USART6)高速数据接收
 *
 * 配置: DMA2 Stream1, Channel 5, 外设到内存, 单次模式
 * 每收到2000字节后触发传输完成中断，双缓冲后通知主循环处理。
 */

#ifndef __DMA_H
#define __DMA_H

#include "sys.h"
#include "timer.h"

void DMA_Initializes(void);

#define DMA2_Stream1_IQ_ENABLE 1     /* DMA传输完成中断使能 */
#define DMA_USART6_RX_BUF_LEN  2000  /* DMA接收缓冲区大小（字节） */

extern u8 DMA_RX_DONE;
extern u8 LEIDA_HANDLE_DONE;
extern u8 DMA_USART6_RX_BUF[];
extern u8 DMA_USART6_RX_BUF_r[];

#endif
