/**
 * @file    DMA.c
 * @brief   DMA驱动 — 雷达串口USART2高速数据接收
 *
 * 配置:
 *   DMA1 Stream5, Channel 4
 *   外设: USART2->DR (字节)
 *   内存: DMA_USART2_RX_BUF[1798]
 *   模式: Normal（单次，ISR中重新使能）
 *   方向: 外设->内存
 *
 * 传输完成中断处理流程:
 *   1. 拷贝数据到 DMA_USART2_RX_BUF_r（双缓冲）
 *   2. 置位 DMA_RX_DONE = 1（通知主循环）
 *   3. 重新使能DMA准备下一帧
 *
 * 对应谢露版: USART2 (PA2=TX, PA3=RX), DMA1 Stream5 Channel4
 */

#include "DMA.h"

u8 DMA_USART2_RX_BUF[DMA_USART2_RX_BUF_LEN];
u8 DMA_USART2_RX_BUF_r[DMA_USART2_RX_BUF_LEN];
u8 DMA_RX_DONE       = 0;
u8 LEIDA_HANDLE_DONE = 1;

void DMA_Initializes(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Stream5);
    while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);  /* 等待DMA完全关闭 */

    /* 配置 DMA1 Stream5 用于 USART2 RX */
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (u32)DMA_USART2_RX_BUF;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = DMA_USART2_RX_BUF_LEN;
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
    DMA_Init(DMA1_Stream5, &DMA_InitStructure);

#if DMA1_Stream5_IQ_ENABLE
    DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);  /* 使能传输完成中断 */

    NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Stream5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

    DMA_Cmd(DMA1_Stream5, ENABLE);
}

#if DMA1_Stream5_IQ_ENABLE
/**
 * @brief  DMA1 Stream5 传输完成中断服务函数
 *
 * 完整一帧（1798字节）LiDAR数据接收完成后触发。
 * 双缓冲数据并通知主循环处理。
 */
void DMA1_Stream5_IRQHandler(void)
{
    if (DMA_GetFlagStatus(DMA1_Stream5, DMA_FLAG_TCIF5) != RESET) {
        DMA_Cmd(DMA1_Stream5, DISABLE);  /* 暂停DMA以安全拷贝缓冲区 */

        memcpy(DMA_USART2_RX_BUF_r, DMA_USART2_RX_BUF, DMA_USART2_RX_BUF_LEN);
        DMA_RX_DONE = 1;  /* 通知主循环: 一帧数据就绪 */

        DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_FEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5);
        DMA_SetCurrDataCounter(DMA1_Stream5, DMA_USART2_RX_BUF_LEN);  /* 重置计数 */

        DMA_Cmd(DMA1_Stream5, ENABLE);  /* 重新使能DMA */
    }
}
#endif
