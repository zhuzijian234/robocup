#include "DMA.h"
#include "ble_diag.h"
#include <string.h>
/* 接收层只搬运字节，不做协议解析/浮点运算/打印。
 * 正常运行保持循环DMA；只有故障恢复时才能停止并重启接收流。 */
uint8_t DMA_USART2_RX_BUF[DMA_USART2_RX_BUF_LEN];
static uint8_t queue[LIDAR_RX_BLOCKS][LIDAR_RX_BLOCK];
static LidarRxStamp stamps[LIDAR_RX_BLOCKS];
/* 单生产者/单消费者：ISR只推进head，主循环只推进tail；发布前使用DMB。
 * 不允许额外的IDLE中断绕过这套所有权再向队列写入。 */
static volatile uint32_t head, tail;
static volatile uint8_t fault;
static uint8_t expected_half;
static uint32_t last_us, last_cycles;
volatile uint32_t LidarRx_epoch, LidarRx_peak, LidarRx_blocks;
volatile uint32_t LidarRx_late, LidarRx_copy_max_cycles;
#define ALL_FLAGS (DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_FEIF5)
void LidarRx_Fault(void)
{
    /* UART与DMA错误中断抢占优先级相同；主循环调用本函数时必须先屏蔽中断。
     * 同一次故障只增加一次epoch，防止旧扫描跨越接收恢复后继续驱动。 */
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
    /* 恢复只在主循环做，ISR不等待DMA停止；清队列与换回正常态是短临界区。 */
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
    /* 复制完成才释放槽位；期间发生故障/换代则返回0，调用者不得解析本块。 */
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
    /* HISR原始位必须用DMA_HISR_*，不能用带库内部选择位的DMA_FLAG_*判断。
     * 半区顺序/NDTR/周期任一异常即丢弃，不尝试拼接可能已被覆盖的字节。 */
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
    memcpy(queue[head & (LIDAR_RX_BLOCKS - 1u)], DMA_USART2_RX_BUF + half * LIDAR_RX_BLOCK, LIDAR_RX_BLOCK);
    ndtr = DMA_GetCurrDataCounter(DMA1_Stream5);
    /* NDTR alone can look safe again after a complete DMA revolution.
     * A half-buffer takes at least 10 ms at 512000 baud, 8N1. */
    if (!ndtr || DWT->CYCCNT - cycle >= SystemCoreClock / 100u ||
        (!half && ndtr > LIDAR_RX_BLOCK) || (half && ndtr <= LIDAR_RX_BLOCK)) {
        LidarRx_late++; LidarRx_Fault(); return;
    }
    /* The previous IRQ timestamp is AFTER the hardware half boundary.
     * Subtract its bounded service latency, including across timer wrap. */
    stamps[head & (LIDAR_RX_BLOCKS - 1u)].start_us = last_us - LIDAR_RX_SERVICE_MAX_US;
    stamps[head & (LIDAR_RX_BLOCKS - 1u)].end_us = now;
    stamps[head & (LIDAR_RX_BLOCKS - 1u)].epoch = LidarRx_epoch;
    __DMB(); head++;
    LidarRx_blocks++;
    if ((used + 1u) * LIDAR_RX_BLOCK > LidarRx_peak) LidarRx_peak = (used + 1u) * LIDAR_RX_BLOCK;
    expected_half ^= 1u; last_us = now; last_cycles = cycle;
    elapsed = DWT->CYCCNT - cycle;
    if (elapsed > LidarRx_copy_max_cycles) LidarRx_copy_max_cycles = elapsed;
}
