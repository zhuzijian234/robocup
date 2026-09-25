
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "m10p.h"
#define PI 3.14159265358979323846f
#define arm_cos_f32 cosf
#define arm_sin_f32 sinf
typedef struct {float angle,distance;} _LEIDA_DATA;
static uint32_t clock_us;
uint32_t Diag_TimeUs(void){return clock_us;}
static unsigned checks;
#define CHECK(x) do{++checks;if(!(x)){printf("FAIL %d: %s\n",__LINE__,#x);return 1;}}while(0)
uint32_t __get_PRIMASK(void){return 0;}
void __disable_irq(void){}
void __set_PRIMASK(uint32_t x){(void)x;}
void __DMB(void){}

#define LIDAR_RX_BLOCK 512u
#define LIDAR_RX_BLOCKS 16u
#define LIDAR_RX_SERVICE_MAX_US 20000u
typedef struct{uint32_t start_us,end_us,epoch;} LidarRxStamp;
static struct{uint32_t HISR;} dma_regs;
static struct{uint32_t CYCCNT;} dwt_regs;
#define DMA1 (&dma_regs)
#define DWT (&dwt_regs)
#define DMA1_Stream5 5
uint32_t SystemCoreClock=168000000;
#define DMA_HISR_FEIF5 0x40u
#define DMA_HISR_DMEIF5 0x100u
#define DMA_HISR_TEIF5 0x200u
#define DMA_HISR_HTIF5 0x400u
#define DMA_HISR_TCIF5 0x800u
#define DMA_FLAG_FEIF5 (0x20000000u|DMA_HISR_FEIF5)
#define DMA_FLAG_DMEIF5 (0x20000000u|DMA_HISR_DMEIF5)
#define DMA_FLAG_TEIF5 (0x20000000u|DMA_HISR_TEIF5)
#define DMA_FLAG_HTIF5 (0x20000000u|DMA_HISR_HTIF5)
#define DMA_FLAG_TCIF5 (0x20000000u|DMA_HISR_TCIF5)
#define ALL_FLAGS (DMA_FLAG_TCIF5|DMA_FLAG_HTIF5|DMA_FLAG_TEIF5|DMA_FLAG_DMEIF5|DMA_FLAG_FEIF5)
static uint32_t ndtr;
static unsigned ndtr_reads, simulate_copy_wrap, simulate_copy_cross;
void DMA_ClearFlag(int s,uint32_t f){(void)s;DMA1->HISR &= ~(f&0x0fffffffu);}
uint32_t DMA_GetCurrDataCounter(int s){(void)s;
 if (++ndtr_reads==2) {
  if(simulate_copy_wrap)DWT->CYCCNT+=SystemCoreClock/40u;
  if(simulate_copy_cross)return 1000;
 }
 return ndtr;
}
static uint8_t DMA_USART2_RX_BUF[1024],queue[16][512];
static LidarRxStamp stamps[16];
static volatile uint32_t head,tail;
static volatile uint8_t fault;
static uint8_t expected_half;
static uint32_t last_us,last_cycles;
volatile uint32_t LidarRx_epoch,LidarRx_peak,LidarRx_blocks,LidarRx_late,LidarRx_copy_max_cycles;
volatile uint32_t Diag_dma_errors,Diag_rx_overflow;
void LidarRx_Fault(void)
{
    /* UART与DMA错误中断抢占优先级相同；主循环调用本函数时必须先屏蔽中断。
     * 同一次故障只增加一次epoch，防止旧扫描跨越接收恢复后继续驱动。 */
    if (!fault) { LidarRx_epoch++; fault = 1; }
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
static void reset(void){head=tail=0;fault=expected_half=0;last_us=last_cycles=0;clock_us=0;DWT->CYCCNT=0;LidarRx_epoch=0;ndtr_reads=simulate_copy_wrap=simulate_copy_cross=0;}
static void irq(unsigned half){clock_us+=11000;DWT->CYCCNT+=1848000;ndtr=half?1000:500;DMA1->HISR=half?0x800:0x400;DMA1_Stream5_IRQHandler();}
int main(void){unsigned i;uint8_t out[512];LidarRxStamp s;
 reset();memset(DMA_USART2_RX_BUF,0x12,512);memset(DMA_USART2_RX_BUF+512,0x34,512);
 irq(0);CHECK(head==1 && !fault);CHECK(!DMA1->HISR);CHECK(!LidarRx_Read(out,511,&s));CHECK(tail==0);
 CHECK(LidarRx_Read(out,512,&s)==512);CHECK(out[0]==0x12 && out[511]==0x12);CHECK(s.start_us==0u-LIDAR_RX_SERVICE_MAX_US && s.end_us==11000);
 irq(1);CHECK(LidarRx_Read(out,512,&s)==512);CHECK(out[0]==0x34);CHECK(!LidarRx_Read(out,512,&s));
 for(i=0;i<100;i++){irq(i&1);CHECK(LidarRx_Read(out,512,&s)==512);}CHECK(!fault);
 reset();for(i=0;i<16;i++)irq(i&1);CHECK(head-tail==16);CHECK(LidarRx_peak==8192);CHECK(!fault);
 irq(0);CHECK(fault && Diag_rx_overflow==1 && LidarRx_epoch==1);CHECK(!LidarRx_Read(out,512,&s));
 reset();DMA1->HISR=0xc00;DMA1_Stream5_IRQHandler();CHECK(fault);CHECK(!head);
 reset();irq(1);CHECK(fault);CHECK(!head);
 reset();DMA1->HISR=0x200;DMA1_Stream5_IRQHandler();CHECK(fault && Diag_dma_errors==1);
 reset();DWT->CYCCNT=4000000;irq(0);CHECK(fault);CHECK(!head);
 /* Hardware boundary ambiguity, crossing while copying, and full-wrap ABA. */
 reset();DMA1->HISR=0x400;ndtr=0;DMA1_Stream5_IRQHandler();CHECK(fault && !head);
 reset();simulate_copy_cross=1;irq(0);CHECK(fault && !head);
 reset();simulate_copy_wrap=1;irq(0);CHECK(fault && !head);
 /* A delayed previous interrupt must not make the next block look younger. */
 reset();irq(0);irq(1);CHECK(LidarRx_Read(out,512,&s)==512);CHECK(LidarRx_Read(out,512,&s)==512);
 CHECK(s.start_us==11000u-LIDAR_RX_SERVICE_MAX_US);
 CHECK((uint32_t)(22000u-s.start_us)==31000u);
 reset();head=tail=0xfffffff0u;for(i=0;i<32;i++){irq(i&1);CHECK(LidarRx_Read(out,512,&s)==512);}CHECK(head==16 && tail==16);
 printf("PASS %u DMA ISR/FIFO checks\n",checks);return 0;
}
