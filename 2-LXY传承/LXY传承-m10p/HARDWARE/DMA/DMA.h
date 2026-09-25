#ifndef __DMA_H
#define __DMA_H
#include "sys.h"
#define DMA_USART2_RX_BUF_LEN 1024u
#define LIDAR_RX_BLOCK 512u
#define LIDAR_RX_BLOCKS 16u
/* Maximum accepted interval between DMA services. Also bounds IRQ lateness. */
#define LIDAR_RX_SERVICE_MAX_US 20000u
typedef struct { uint32_t start_us, end_us, epoch; } LidarRxStamp;
extern volatile uint32_t LidarRx_epoch, LidarRx_peak, LidarRx_blocks;
extern volatile uint32_t LidarRx_late, LidarRx_copy_max_cycles;
extern uint8_t DMA_USART2_RX_BUF[DMA_USART2_RX_BUF_LEN];
void DMA_Initializes(void);
uint16_t LidarRx_Read(uint8_t *dst, uint16_t capacity, LidarRxStamp *stamp);
void LidarRx_Fault(void);
void LidarRx_Service(void);
#endif
