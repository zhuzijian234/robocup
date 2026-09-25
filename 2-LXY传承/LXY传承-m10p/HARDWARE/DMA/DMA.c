#include "DMA.h"
#include "ble_diag.h"
#include <string.h>
/* Normal RX never disables DMA. Only error recovery restarts the stream. */
uint8_t DMA_USART2_RX_BUF[DMA_USART2_RX_BUF_LEN];
static uint8_t queue[LIDAR_RX_BLOCKS][LIDAR_RX_BLOCK];
static LidarRxStamp stamps[LIDAR_RX_BLOCKS];
static volatile uint32_t head, tail;
static volatile uint8_t fault;
static uint8_t expected_half;
static uint32_t last_us, last_cycles;
volatile uint32_t LidarRx_epoch, LidarRx_peak, LidarRx_blocks;
volatile uint32_t LidarRx_late, LidarRx_copy_max_cycles;
#define ALL_FLAGS (DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_FEIF5)
void LidarRx_Fault(void)
{
    /* RX interrupts have equal preemption priority. Main calls with IRQ mask. */
    if (!fault) { LidarRx_epoch++; fault = 1; }
}
void DMA_Initializes(void)
{
    DMA_InitTypeDef d;
    NVIC_InitTypeDef n;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Stream5);
    DMA_StructInit(&d);
    d.DMA_Channel = DMA_Channel_4;
    d.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    d.DMA_Memory0BaseAddr = (uint32_t)DMA_USART2_RX_BUF;
    d.DMA_DIR = DMA_DIR_PeripheralToMemory;
    d.DMA_BufferSize = DMA_USART2_RX_BUF_LEN;
    d.DMA_MemoryInc = DMA_MemoryInc_Enable;
    d.DMA_Mode = DMA_Mode_Circular;
    d.DMA_Priority = DMA_Priority_High;
    DMA_Init(DMA1_Stream5, &d);
    DMA_ITConfig(DMA1_Stream5, DMA_IT_HT | DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);
    n.NVIC_IRQChannel = DMA1_Stream5_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 0;
    n.NVIC_IRQChannelSubPriority = 0;
    n.NVIC_IRQChannelCmd = ENABLE; NVIC_Init(&n);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    head = tail = 0; fault = expected_half = 0;
    last_us = Diag_TimeUs(); last_cycles = DWT->CYCCNT;
    DMA_ClearFlag(DMA1_Stream5, ALL_FLAGS);
    DMA_Cmd(DMA1_Stream5, ENABLE);
}
void LidarRx_Service(void)
{
    uint32_t p, sr;
    if (!fault) return;
    DMA_Cmd(DMA1_Stream5, DISABLE);
    if (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE) return;
    p = __get_PRIMASK(); __disable_irq();
    tail = head;
    sr = USART2->SR; sr = USART2->DR; (void)sr;
    DMA_ClearFlag(DMA1_Stream5, ALL_FLAGS);
    DMA_SetCurrDataCounter(DMA1_Stream5, DMA_USART2_RX_BUF_LEN);
    expected_half = 0; last_us = Diag_TimeUs(); last_cycles = DWT->CYCCNT;
    fault = 0; DMA_Cmd(DMA1_Stream5, ENABLE);
    __set_PRIMASK(p);
}
uint16_t LidarRx_Read(uint8_t *dst, uint16_t capacity, LidarRxStamp *stamp)
{
    uint32_t t = tail, epoch = LidarRx_epoch;
    if (fault || capacity < LIDAR_RX_BLOCK || t == head) return 0;
    __DMB();
    *stamp = stamps[t & (LIDAR_RX_BLOCKS - 1)];
    memcpy(dst, queue[t & (LIDAR_RX_BLOCKS - 1)], LIDAR_RX_BLOCK);
    __DMB(); tail = t + 1;
    return !fault && epoch == LidarRx_epoch ? LIDAR_RX_BLOCK : 0;
}
void DMA1_Stream5_IRQHandler(void)
{
    uint32_t flags = DMA1->HISR, now = Diag_TimeUs(), cycle = DWT->CYCCNT;
    uint32_t used, elapsed, ndtr;
    uint8_t half;
    if (flags & (DMA_HISR_TEIF5 | DMA_HISR_DMEIF5 | DMA_HISR_FEIF5)) {
        Diag_dma_errors++; LidarRx_Fault();
    }
    if (fault) { DMA_ClearFlag(DMA1_Stream5, ALL_FLAGS); return; }
    if ((flags & (DMA_HISR_HTIF5 | DMA_HISR_TCIF5)) == (DMA_HISR_HTIF5 | DMA_HISR_TCIF5)) {
        LidarRx_late++; LidarRx_Fault(); DMA_ClearFlag(DMA1_Stream5, ALL_FLAGS); return;
    }
    if (!(flags & (DMA_HISR_HTIF5 | DMA_HISR_TCIF5))) return;
    half = (flags & DMA_HISR_HTIF5) ? 0 : 1;
    DMA_ClearFlag(DMA1_Stream5, half ? DMA_FLAG_TCIF5 : DMA_FLAG_HTIF5);
    ndtr = DMA_GetCurrDataCounter(DMA1_Stream5);
    if (half != expected_half || cycle - last_cycles > SystemCoreClock / 50u || !ndtr ||
        (!half && ndtr > LIDAR_RX_BLOCK) || (half && ndtr <= LIDAR_RX_BLOCK)) {
        LidarRx_late++; LidarRx_Fault(); return;
    }
    used = head - tail;
    if (used >= LIDAR_RX_BLOCKS) { Diag_rx_overflow++; LidarRx_Fault(); return; }
    memcpy(queue[head & 15u], DMA_USART2_RX_BUF + half * LIDAR_RX_BLOCK, LIDAR_RX_BLOCK);
    ndtr = DMA_GetCurrDataCounter(DMA1_Stream5);
    /* NDTR alone can look safe again after a complete DMA revolution.
     * A half-buffer takes at least 10 ms at 512000 baud, 8N1. */
    if (!ndtr || DWT->CYCCNT - cycle >= SystemCoreClock / 100u ||
        (!half && ndtr > LIDAR_RX_BLOCK) || (half && ndtr <= LIDAR_RX_BLOCK)) {
        LidarRx_late++; LidarRx_Fault(); return;
    }
    /* The previous IRQ timestamp is AFTER the hardware half boundary.
     * Subtract its bounded service latency, including across timer wrap. */
    stamps[head & 15u].start_us = last_us - LIDAR_RX_SERVICE_MAX_US;
    stamps[head & 15u].end_us = now;
    stamps[head & 15u].epoch = LidarRx_epoch;
    __DMB(); head++;
    LidarRx_blocks++;
    if ((used + 1u) * LIDAR_RX_BLOCK > LidarRx_peak) LidarRx_peak = (used + 1u) * LIDAR_RX_BLOCK;
    expected_half ^= 1u; last_us = now; last_cycles = cycle;
    elapsed = DWT->CYCCNT - cycle;
    if (elapsed > LidarRx_copy_max_cycles) LidarRx_copy_max_cycles = elapsed;
}
