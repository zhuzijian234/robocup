#ifndef __DMA_H
#define __DMA_H
#include "sys.h"
/* 接收层容量独立于协议包长：160字节包允许横跨任意512字节DMA块。 */
#define DMA_USART2_RX_BUF_LEN 1024u
#define LIDAR_RX_BLOCK 512u
#define LIDAR_RX_BLOCKS 16u
/* 允许的最大服务间隔，也是时间戳需扣除的中断延迟保守余量。 */
#define LIDAR_RX_SERVICE_MAX_US 20000u
#if DMA_USART2_RX_BUF_LEN != (2u * LIDAR_RX_BLOCK)
#error "DMA buffer must contain exactly two receive blocks"
#endif
#if !LIDAR_RX_BLOCKS || (LIDAR_RX_BLOCKS & (LIDAR_RX_BLOCKS - 1u))
#error "Receive queue capacity must be a power of two"
#endif
/* 同一个块的接收时间下界/上界/故障代次，不允许用解析时刻替代。 */
typedef struct { uint32_t start_us, end_us, epoch; } LidarRxStamp;
extern volatile uint32_t LidarRx_epoch, LidarRx_peak, LidarRx_blocks;
extern volatile uint32_t LidarRx_late, LidarRx_copy_max_cycles;
extern uint8_t DMA_USART2_RX_BUF[DMA_USART2_RX_BUF_LEN];
void DMA_Initializes(void);
uint16_t LidarRx_Read(uint8_t *dst, uint16_t capacity, LidarRxStamp *stamp);
void LidarRx_Fault(void);
void LidarRx_Service(void);
#endif
