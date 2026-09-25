"""Execute production adapter, safety guard, PI and DMA ISR with hardware mocks."""
from pathlib import Path
import re, json
from check_m10p import P, OUT, run
def source(path):return (P/path).read_text(encoding='utf-8-sig')
def function(text,name):
    m=re.search(r'^(?:static )?\w+\s+'+name+r'\([^;]*?\)\s*\{',text,re.M)
    assert m,name
    pos=text.index('{',m.start())+1; depth=1
    while depth:depth+=(text[pos]=='{')-(text[pos]=='}');pos+=1
    return text[m.start():pos]
prefix=r'''
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
'''
vehicle=source('HARDWARE/LEIDA_DATA/m10p_vehicle.c')
timer=source('HARDWARE/LEIDA_TIMER/timer.c')
centre=source('HARDWARE/CENTRE_LINE/CENTRE_LINE.c')
program=prefix+'\n#include "m10p.c"\n'+r'''
uint32_t LidarRx_epoch;
static uint16_t bins[720];
static uint8_t rx[512];
static uint32_t seen_epoch,seen_discontinuities,seen_rejected;
typedef struct{uint32_t start_us,end_us,epoch;} LidarRxStamp;
void LidarRx_Service(void){}
uint16_t LidarRx_Read(uint8_t *p,uint16_t n,LidarRxStamp *s){(void)p;(void)n;(void)s;return 0;}
void Radar_Invalidate(void);
uint32_t M10P_control_seq,M10P_control_front_us,M10P_control_epoch;
uint16_t M10P_front_bins,M10P_left_bins,M10P_right_bins;
float M10P_clearance_mm,M10P_speed_scale;
uint8_t M10P_perception_ok;
#define RADAR_TIMEOUT_TICKS 50u
volatile uint16_t Radar_age_ticks;
volatile uint8_t Radar_started,Radar_stop_latched;
volatile uint32_t Radar_timeout_count;
typedef struct {float kp,ki,kd,err,err_l,err_sum;} pid_type;
volatile float Diag_motor_integral,Diag_motor_prelimit;
'''
program+=function(vehicle,'M10P_Build')+function(vehicle,'M10P_Poll')
program+=timer[timer.index('static volatile uint32_t observation_us'):timer.index('/**',timer.index('static volatile uint32_t observation_us'))]
program+=function(centre,'PID_realize')+'\n'+function(centre,'Speed_PID_Reset')
program+=r'''
static M10P_Scan s;static _LEIDA_DATA out[720];
int main(void){unsigned i;pid_type pid={8.5f,.505f,0};float pwm;
 s.seq=1;s.epoch=0;s.front_seen=1;s.front_us=50000;clock_us=100000;
 for(i=0;i<720;i++){s.points[i].angle_cdeg=(uint16_t)(i*50);s.points[i].range_mm=1000;}s.count=720;
 CHECK(M10P_Build(&s,out,720)==720);CHECK(M10P_perception_ok);CHECK(M10P_front_bins==81);
 CHECK(out[0].angle==0 && out[180].angle==90 && out[360].angle==180);
 /* 40 valid front bins can still hide a 20-degree central blind sector. */
 for(i=0;i<720;i++)if(i<=20 || i>=700)s.points[i].range_mm=0;
 M10P_Build(&s,out,720);CHECK(M10P_front_bins==40);CHECK(!M10P_perception_ok);
 for(i=0;i<720;i++)s.points[i].range_mm=1000;
 M10P_Build(&s,out,720);CHECK(M10P_perception_ok);
 CHECK(!M10P_Build(&s,out,719));CHECK(!M10P_perception_ok);
 s.points[0].range_mm=200;M10P_Build(&s,out,720);CHECK(!M10P_perception_ok);CHECK(M10P_clearance_mm<201);
 s.points[0].range_mm=80;M10P_Build(&s,out,720);CHECK(!M10P_perception_ok);CHECK(M10P_clearance_mm<81);
 s.points[0].range_mm=1000;clock_us=s.front_us+M10P_MAX_AGE_US+1;M10P_Build(&s,out,720);CHECK(!M10P_perception_ok);
 clock_us=100000;s.count=0;M10P_Build(&s,out,720);CHECK(!M10P_perception_ok);
 s.count=720;s.epoch=1;M10P_Build(&s,out,720);CHECK(!M10P_perception_ok);s.epoch=0;
 Radar_Observe(1,50000,0,1);CHECK(!observation_valid && !Radar_started);
 clock_us+=83000;Radar_Observe(2,clock_us-50000,0,.6f);CHECK(!observation_valid);
 clock_us+=83000;Radar_Observe(3,clock_us-50000,0,.6f);CHECK(observation_valid && Radar_started);CHECK(observation_scale==.6f);
 Radar_Observe(3,clock_us-50000,0,1);CHECK(!observation_valid);CHECK(warmup==0);
 for(i=4;i<=6;i++){clock_us+=83000;Radar_Observe(i,clock_us-50000,0,1);}CHECK(observation_valid);
 clock_us=observation_us+M10P_MAX_AGE_US+1;Radar_GuardTick();CHECK(!observation_valid);
 for(i=0;i<50;i++)Radar_GuardTick();CHECK(Radar_stop_latched && Radar_timeout_count==1);
 Radar_Observe(7,clock_us,0,1);CHECK(!observation_valid);CHECK(Radar_stop_latched);
 Radar_stop_latched=Radar_started=0;warmup=0;observation_seq=0;clock_us=100000;
 for(i=1;i<=3;i++){clock_us+=83000;Radar_Observe(i,clock_us-50000,0,1);}CHECK(observation_valid);
 LidarRx_epoch++;Radar_GuardTick();CHECK(!observation_valid);CHECK(warmup==0);
 Radar_Invalidate();CHECK(!observation_valid);
 /* Rejecting a newly completed circle revokes the prior driving permission
  * immediately, without waiting for its age timeout. */
 seen_epoch=LidarRx_epoch;seen_discontinuities=M10P_stats.discontinuities;
 seen_rejected=M10P_stats.rejected;observation_valid=1;warmup=3;
 M10P_stats.rejected++;M10P_Poll();CHECK(!observation_valid && !warmup);
 observation_valid=1;warmup=3;M10P_stats.discontinuities++;
 M10P_Poll();CHECK(!observation_valid && !warmup);
 for(i=0;i<20;i++)pwm=PID_realize(0,10,&pid);CHECK(pwm==100);CHECK(pid.err_sum==200);
 Speed_PID_Reset(&pid);CHECK(pid.err_sum==0 && pid.err_l==0);CHECK(Diag_motor_integral==0);
 CHECK(PID_realize(0,0,&pid)==0);
 printf("PASS %u adapter/guard/PI checks\n",checks);return 0;
}
'''
p=OUT/'integration.c';p.write_text(program,encoding='utf-8');results=[run('integration',p)]
dma=source('HARDWARE/DMA/DMA.c')
program=prefix+r'''
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
'''
program+='\n'.join(function(dma,n) for n in ['LidarRx_Fault','LidarRx_Read','DMA1_Stream5_IRQHandler'])
program+=r'''
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
'''
p=OUT/'dma_test.c';p.write_text(program,encoding='utf-8');results.append(run('dma',p))
(OUT/'integration_summary.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
