/**
 * @file    DMA.c
 * @brief   DMA驱动 — 雷达串口USART6高速数据接收
 *
 * 配置:
 *   DMA2 Stream1, Channel 5
 *   外设: USART6->DR (字节)
 *   内存: DMA_USART6_RX_BUF[2000]
 *   模式: Normal（单次，ISR中重新使能）
 *   方向: 外设->内存
 *
 * 传输完成中断处理流程:
 *   1. 拷贝数据到 DMA_USART6_RX_BUF_r（双缓冲）
 *   2. 置位 DMA_RX_DONE = 1（通知主循环）
 *   3. 重新使能DMA准备下一帧
 */

#include "DMA.h"

u8 DMA_USART6_RX_BUF[DMA_USART6_RX_BUF_LEN];
u8 DMA_USART6_RX_BUF_r[DMA_USART6_RX_BUF_LEN];
u8 DMA_RX_DONE       = 0;
u8 LEIDA_HANDLE_DONE = 1;

void DMA_Initializes(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    DMA_DeInit(DMA2_Stream1);
    while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);  /* 等待DMA完全关闭 */

    /* 配置 DMA2 Stream1 用于 USART6 RX */
    DMA_InitStructure.DMA_Channel            = DMA_Channel_5;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART6->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (u32)DMA_USART6_RX_BUF;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = DMA_USART6_RX_BUF_LEN;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;   /* 外设地址固定 */
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;        /* 内存地址递增 */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; /* 8位传输 */
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;             /* 单次模式 */
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream1, &DMA_InitStructure);

#if DMA2_Stream1_IQ_ENABLE
    DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);  /* 使能传输完成中断 */

    NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

    DMA_Cmd(DMA2_Stream1, ENABLE);
}

#if DMA2_Stream1_IQ_ENABLE
/**
 * @brief  DMA2 Stream1 传输完成中断服务函数
 *
 * 完整一帧（2000字节）LiDAR数据接收完成后触发。
 * 双缓冲数据并通知主循环处理。
 */
void DMA2_Stream1_IRQHandler(void)
{
    if (DMA_GetFlagStatus(DMA2_Stream1, DMA_FLAG_TCIF1) != RESET) {
        DMA_Cmd(DMA2_Stream1, DISABLE);  /* 暂停DMA以安全拷贝缓冲区 */

        memcpy(DMA_USART6_RX_BUF_r, DMA_USART6_RX_BUF, DMA_USART6_RX_BUF_LEN);
        DMA_RX_DONE = 1;  /* 通知主循环: 一帧数据就绪 */

        DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_FEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1);
        DMA_SetCurrDataCounter(DMA2_Stream1, DMA_USART6_RX_BUF_LEN);  /* 重置计数 */

        DMA_Cmd(DMA2_Stream1, ENABLE);  /* 重新使能DMA */
    }
}
#endif
