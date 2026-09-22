#include "ble_diag.h"
#include "bsp_bluetooth.h"
#include "ble_tune.h"
#include "centre_line.h"
#include "LEIDA_DATA.h"
#include "moto.h"
#include "timer.h"
#include "stdio.h"
#include "string.h"
#include "float.h"
#include "diag_build_id.h"

uint8_t Diag_mode=1, Diag_rate=1;
uint32_t Diag_revision=0, Diag_session=0;
volatile uint32_t Diag_input_seq, Diag_input_ms, Diag_input_us, Diag_input_drop;
volatile uint32_t Diag_uart_errors, Diag_dma_errors, Diag_rx_overflow;
static volatile uint32_t tick_ms;
static uint32_t tx_seq, control_seq, last_us, last_ms, last_valid_ms, process_max;
static uint32_t begin_us, in_ms, mask, updated, last_health;
static uint8_t have_control, have_valid, last_ok, sample, config_pending, config_index;
static uint32_t config_id;
static float fields[26], config_params[16];
static uint8_t frame[256];
uint32_t Diag_detail_u[24];
float Diag_detail_f[16];
volatile float Diag_motor_integral, Diag_motor_prelimit;
/* Single TIM5 producer / main consumer; payload published before head. */
typedef struct {uint32_t seq,rev,us;uint16_t raw,pwm;float speed,target,integral,prelimit;uint32_t flags;} MotorSample;
static MotorSample motor_ring[64];
static volatile uint32_t motor_head,motor_tail,motor_seq,motor_dropped;
static uint32_t hold_count;
static void motor_reset(void){uint32_t p=__get_PRIMASK();__disable_irq();motor_tail=motor_head;__set_PRIMASK(p);}
void Diag_MotorTick(uint16_t raw,uint8_t fresh,uint8_t pi){
    MotorSample *s;uint32_t seq=++motor_seq;
    if(Diag_mode!=3 || !Diag_session)return;
    if(motor_head-motor_tail>=64){motor_dropped++;return;}
    s=&motor_ring[motor_head&63];s->seq=seq;s->rev=Diag_revision;s->us=Diag_TimeUs();
    s->raw=raw;s->pwm=(uint16_t)TIM2->CCR1;s->speed=Speed_now;s->target=Speed_mubiao;
    s->integral=Diag_motor_integral;s->prelimit=Diag_motor_prelimit;
    s->flags=(fresh?1u:0u)|(pi?2u:0u)|(Radar_started?4u:0u)|(Radar_stop_latched?8u:0u)|(daoche_flag?16u:0u);
    __DMB();motor_head++;
}
/* CRC8 matches the LD14P development manual table (poly 0x4d, init 0).
 * Required acceptance checks; also counts rejected candidate packets. */
uint8_t Diag_RadarPacket(const uint8_t *a){
    uint8_t crc=0,b;uint16_t i,start=(uint16_t)(a[4]|a[5]<<8),end=(uint16_t)(a[42]|a[43]<<8);
    Diag_detail_u[12]++;
    for(i=0;i<46;i++){crc^=a[i];for(b=0;b<8;b++)crc=(uint8_t)((crc<<1)^((crc&0x80)?0x4d:0));}
    if(crc!=a[46])Diag_detail_u[13]++;
    if(a[1]!=0x2c)Diag_detail_u[14]++;
    if(start>=36000 || end>=36000)Diag_detail_u[15]++;
    return a[0]==0x54 && a[1]==0x2c && start<36000 && end<36000 && crc==a[46];
}

static const uint16_t scales[26]={10,1,1,1000,1,1000,1,1000,1000,1,1,1,1,1,1,1,1,1,10,1,1,1,1,1,1,1};
static void put16(uint8_t*p,uint16_t v){p[0]=v;p[1]=v>>8;}
static void put32(uint8_t*p,uint32_t v){put16(p,(uint16_t)v);put16(p+2,(uint16_t)(v>>16));}
uint16_t Diag_CRC(const uint8_t*p,uint16_t n){uint16_t c=0xffff;uint8_t i;while(n--){c^=(uint16_t)*p++<<8;for(i=0;i<8;i++)c=(c&0x8000)?(uint16_t)((c<<1)^0x1021):(uint16_t)(c<<1);}return c;}
int16_t Diag_Encode(uint8_t i,float v,uint8_t*valid,uint8_t*clipped){
    float limit=0, scaled; double rounded;
    *valid=0;*clipped=0;
    if(i>=26 || !(v<=FLT_MAX && v>=-FLT_MAX))return 0;
    if(i==2 && (v<0 || v>9 || v!=(int)v))return 0;
    if(i==20 && v!=0 && v!=1)return 0;
    if(i==0)limit=600;
    if(i==3 || i==5 || i==7 || i==8)limit=30;
    if(i==4 || i==6)limit=30000;
    if(!limit && i!=18 && v<0)return 0;
    *valid=1;
    if(limit){if(v>limit){v=limit;*clipped=1;}if(v< -limit){v= -limit;*clipped=1;}}
    scaled=v*scales[i];
    if(scaled>32767){*clipped=1;return 32767;}
    if(scaled< -32768){*clipped=1;return -32768;}
    rounded=(double)scaled+(scaled>=0?0.5:-0.5);
    return (int16_t)(int32_t)rounded;
}
void TIM6_DAC_IRQHandler(void){if(TIM6->SR&TIM_SR_UIF){TIM6->SR=0;tick_ms++;}}
static void clock_read(uint32_t*ms,uint16_t*us){
    uint32_t p=__get_PRIMASK();__disable_irq();
    *ms=tick_ms;*us=(uint16_t)TIM6->CNT;
    if(TIM6->SR&TIM_SR_UIF){(*ms)++;*us=(uint16_t)TIM6->CNT;}
    __set_PRIMASK(p);
}
uint32_t Diag_TimeUs(void){uint32_t m;uint16_t u;clock_read(&m,&u);return m*1000u+u;}
uint32_t Diag_TimeMs(void){uint32_t m;uint16_t u;clock_read(&m,&u);return m;}
void Diag_Init(void){
    NVIC_InitTypeDef n;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    TIM6->CR1=0;TIM6->PSC=84-1;TIM6->ARR=1000-1;TIM6->EGR=TIM_EGR_UG;TIM6->SR=0;
    tick_ms=0;TIM6->DIER=TIM_DIER_UIE;
    n.NVIC_IRQChannel=TIM6_DAC_IRQn;n.NVIC_IRQChannelPreemptionPriority=0;n.NVIC_IRQChannelSubPriority=1;n.NVIC_IRQChannelCmd=ENABLE;NVIC_Init(&n);
    TIM6->CR1=TIM_CR1_CEN;
}
void Diag_InputDone(void){Diag_input_us=Diag_TimeUs();Diag_input_ms=Diag_TimeMs();Diag_input_seq++;}
void Diag_Begin(uint32_t ms,uint32_t us){mask=updated=0;memset(fields,0,sizeof fields);in_ms=ms;(void)us;begin_us=Diag_TimeUs();
    memset(Diag_detail_u,0,sizeof Diag_detail_u);memset(Diag_detail_f,0,sizeof Diag_detail_f);
}
void Diag_Field(uint8_t i,float v,uint8_t valid){if(i<26){fields[i]=v;updated|=1u<<i;if(valid)mask|=1u<<i;else mask&=~(1u<<i);}}
void Diag_HeldField(uint8_t i,float value,uint8_t valid){Diag_Field(i,value,valid);updated&=~(1u<<i);}
void Diag_Fit(const void*ptr,uint8_t valid){
    const Midline_type*l=(const Midline_type*)ptr;
    if(ptr==&Midline){Diag_Field(3,l->k,valid);Diag_Field(4,l->b,valid);}
    if(ptr==&Midline_forward){Diag_Field(5,l->k,valid);Diag_Field(6,l->b,valid);}
    if(ptr==&Midline_forward_2)Diag_Field(7,l->k,valid);
    if(ptr==&Midline_forward_3)Diag_Field(8,l->k,valid);
}
static void header(uint8_t type,uint16_t n){memset(frame,0,n);frame[0]=0xaa;frame[1]=0x55;frame[2]=2;frame[3]=type;put16(frame+4,n);put32(frame+10,Diag_session);}
void Diag_Finalize(uint8_t*p,uint16_t n){if(n>=16 && p[0]==0xaa && p[1]==0x55 && p[2]==2){put32(p+6,tx_seq++);put16(p+n-2,Diag_CRC(p+2,n-4));}}
static void motor_poll(void){
    uint32_t tail=motor_tail,head=motor_head,seq,rev;uint8_t n=0;uint16_t off;MotorSample *s;
    if(Diag_mode!=3 || !Diag_session || tail==head || !BLE_NormalSpace())return;
    if(head-tail<8 && (uint32_t)(Diag_TimeUs()-motor_ring[tail&63].us)<80000u)return;
    seq=motor_ring[tail&63].seq;rev=motor_ring[tail&63].rev;
    while(n<8 && tail+n!=head && motor_ring[(tail+n)&63].rev==rev && motor_ring[(tail+n)&63].seq==seq+n)n++;
    header(6,(uint16_t)(32+28*n));put16(frame+14,1);put16(frame+16,n);put32(frame+18,seq);put32(frame+22,rev);put32(frame+26,motor_dropped);
    for(n=0,off=30;tail+n!=head && n<8;n++,off+=28){
        s=&motor_ring[(tail+n)&63];if(s->rev!=rev || s->seq!=seq+n)break;
        put32(frame+off,s->us);put16(frame+off+4,s->raw);put16(frame+off+6,s->pwm);
        memcpy(frame+off+8,&s->speed,4);memcpy(frame+off+12,&s->target,4);
        memcpy(frame+off+16,&s->integral,4);memcpy(frame+off+20,&s->prelimit,4);put32(frame+off+24,s->flags);
    }
    if(BLE_Queue(frame,(uint16_t)(32+28*n),0)){__DMB();motor_tail=tail+n;}
}
static void detail_submit(uint32_t elapsed,uint8_t action,uint8_t ok){
    uint8_t i;
    (void)action;
    Diag_detail_u[0]=1;Diag_detail_u[1]=control_seq;Diag_detail_u[2]=Diag_revision;
    Diag_detail_u[5]=elapsed;Diag_detail_u[11]=hold_count;
    (void)ok;
    Diag_detail_u[19]=LEIDA_sync_failures;Diag_detail_u[20]=LEIDA_missing_packets;
    Diag_detail_u[21]=Diag_input_drop;Diag_detail_u[22]=Diag_tx_drop;Diag_detail_u[23]=motor_dropped;
    header(5,176);for(i=0;i<24;i++)put32(frame+14+4*i,Diag_detail_u[i]);
    for(i=0;i<16;i++)memcpy(frame+110+4*i,&Diag_detail_f[i],4);
    BLE_Queue(frame,176,0);
}
void Diag_Submit(uint16_t mode,uint16_t raw,uint16_t speed,uint8_t ok){
    uint32_t now=Diag_TimeUs(),ms=Diag_TimeMs(),clip=0,v=mask,u=updated,p,dt=0;
    uint8_t i,valid,clipped,action,motor;int16_t value;
    control_seq++;last_ok=ok;
    if(have_control && (uint32_t)(ms-last_ms)<4294967u){dt=now-last_us;v|=1u<<30;u|=1u<<30;}
    last_us=now;last_ms=ms;have_control=1;
    if(ok){last_valid_ms=ms;have_valid=1;}
    if(now-begin_us>process_max)process_max=now-begin_us;
    action=mode<=9?1:mode==BLE_MODE_INVALID?5:mode==BLE_MODE_FORCED?((TIM3->CCR1==SERVO_PWM_MAX)?3:4):2;
    Diag_Field(1,(float)TIM3->CCR1,1);
    if(mode<=9){Diag_Field(0,Servo_pd.err,1);Diag_Field(2,(float)mode,1);}
    p=__get_PRIMASK();__disable_irq();
    Diag_Field(18,Speed_now,1);Diag_Field(19,Speed_mubiao,1);
    motor=Radar_stop_latched?3:Radar_started?2:1;
    __set_PRIMASK(p);
    /* Snapshot final main-context line values, including sign normalization. */
    fields[3]=Midline.k;fields[4]=Midline.b;fields[5]=Midline_forward.k;fields[6]=Midline_forward.b;
    fields[7]=Midline_forward_2.k;fields[8]=Midline_forward_3.k;
    v|=mask;u|=updated;
    v|=(1u<<28)|(1u<<29);u|=(1u<<28)|(1u<<29);
    if(raw){v|=(1u<<26)|(1u<<27);u|=(1u<<26)|(1u<<27);}
    hold_count=action==2?hold_count+1:0;
    if(++sample<Diag_rate)return;sample=0;
    if(Diag_mode<2)return;
    header(1,108);
    for(i=0;i<26;i++){
        value=Diag_Encode(i,fields[i],&valid,&clipped);
        if(!valid)v&=~(1u<<i);
        if(clipped && (v&(1u<<i)))clip|=1u<<i;
        put16(frame+14+2*i,(v&(1u<<i))?(uint16_t)value:0);
    }
    if(Diag_mode==2){uint8_t sum=0;memmove(frame+4,frame+14,52);frame[2]=1;frame[3]=(uint8_t)control_seq;for(i=4;i<56;i++)sum+=frame[i];frame[56]=sum;BLE_Queue(frame,57,0);return;}
    put32(frame+66,control_seq);put32(frame+70,in_ms);put32(frame+74,ms);put32(frame+78,dt);
    put32(frame+82,v);put32(frame+86,u);put32(frame+90,clip);frame[94]=action;frame[95]=motor;frame[96]=DIAG_LIDAR_ID;frame[97]=2;
    put32(frame+98,Diag_revision);put16(frame+102,raw);put16(frame+104,raw?speed:0);BLE_Queue(frame,108,0);
    detail_submit(now-begin_us,action,ok);
}
/* CONFIG registry: 1 layout,2 build,3 lidar,4 algorithm,5 input_kind,6 baud,
 * 7 rate,8 timeout_ms,9 health_ms,10 units,11 DMA bytes,12 capacities,
 * 13 geometry contract,14 lidar nominal config; keys100..115 float parameters. */
#define CFG_COUNT 30
void Diag_ConfigStart(void){uint8_t i;uint16_t key;for(i=0;i<16;i++)Tune_ConfigValue(i,&key,&config_params[i]);config_pending=1;config_index=0;config_id++;}
static void config_poll(void){
    uint16_t key=1+config_index,n=4;uint8_t type=1;uint32_t number=0;const char*text=0;float val;
    if(!config_pending || !BLE_NormalSpace())return;
    if(config_index>=14){key=100+config_index-14;type=3;val=config_params[config_index-14];}
    else switch(key){
      case 1:text=DIAG_LAYOUT;break;case 2:text=DIAG_BUILD_ID;break;
      case 3:number=DIAG_LIDAR_ID;break;case 4:number=2;break;case 5:number=DIAG_INPUT_KIND;break;
      case 6:number=BLE_UART_BAUD;break;case 7:number=Diag_rate;break;case 8:number=RADAR_TIMEOUT_TICKS*10;break;
      case 9:number=1000;break;case 10:text="mm;CCR;encoder_units;slope;hold=control_count;PD=dt115ms-Dcap60-dirguard";break;
      case 11:number=DMA_USART2_RX_BUF_LEN;break;case 12:number=LEIDA_DATA_COUNTER;break;
      case 13:text="break=550/600mm;width=600..900mm;PWM=1170/1445/1720;PDmid=1445;fit=checked;center=paired";break;
      case 14:text="LD14P nominal4000pts/s@6Hz;PWM97;raw_crc=required;input=DMA-stream";break;
    }
    if(text){type=4;n=(uint16_t)strlen(text);}
    header(3,33+n);put32(frame+14,config_id);put32(frame+18,Diag_revision);put16(frame+22,config_index);put16(frame+24,CFG_COUNT);
    put16(frame+26,key);frame[28]=type;put16(frame+29,n);
    if(text)memcpy(frame+31,text,n);else if(type==3)memcpy(frame+31,&val,4);else put32(frame+31,number);
    if(BLE_Queue(frame,33+n,1) && ++config_index==CFG_COUNT)config_pending=0;
}
static void event(uint16_t key,float old,float value){header(4,40);put32(frame+14,Diag_TimeMs());put32(frame+18,Diag_revision+1);put16(frame+22,1);put16(frame+24,12);put16(frame+26,key);frame[28]=3;frame[29]=4;memcpy(frame+30,&old,4);memcpy(frame+34,&value,4);/* actual total = 40 */BLE_Queue(frame,40,2);}
static void text_event(uint16_t code,const char*text){uint16_t n=(uint16_t)strlen(text);if(n>100)n=100;header(4,28+n);put32(frame+14,Diag_TimeMs());put32(frame+18,Diag_revision);put16(frame+22,code);put16(frame+24,n);memcpy(frame+26,text,n);BLE_Queue(frame,28+n,2);}
void Diag_Reject(const char*reason){if(Diag_mode==3 && BLE_FreeCritical()>=2)text_event(2,reason);Send_Bluetooth_Data((char*)reason);}
uint8_t Diag_Parameter(uint16_t key,float*ptr,float value){uint32_t p;if(config_pending || BLE_FreeCritical()<2)return 0;if(Diag_mode==3)event(key,*ptr,value);p=__get_PRIMASK();__disable_irq();*ptr=value;Diag_revision++;__set_PRIMASK(p);return 1;}
static uint8_t number(const char*s,uint32_t*value){uint32_t n=0;uint8_t digits=0;while(*s){if(*s<'0'||*s>'9'||n>429496729u||(n==429496729u && *s>'5'))return 0;n=n*10+(*s++-'0');digits=1;}*value=n;return digits;}
uint8_t Diag_Command(char*line){uint32_t n;char ack[180];
    if(!strcmp(line,"info")){sprintf(ack,"INFO proto=1,2 layout=%s fw=%s lidar=%u algorithm=2 input=%u atomic=0 detail=1 motor=1\r\n",DIAG_LAYOUT,DIAG_BUILD_ID,(unsigned)DIAG_LIDAR_ID,(unsigned)DIAG_INPUT_KIND);Send_Bluetooth_Data(ack);return 1;}
    if(!strcmp(line,"getcfg")){if(Diag_mode!=3){Diag_Reject("ERR mode\r\n");return 1;}Diag_ConfigStart();Send_Bluetooth_Data("OK getcfg\r\n");return 1;}
    if(!strncmp(line,"session ",8)){
        if(!number(line+8,&n)||!n){Diag_Reject("ERR session\r\n");return 1;}
        BLE_FlushPending();Diag_session=n;motor_reset();tx_seq=0;config_pending=0;sample=0;sprintf(ack,"OK session %lu\r\n",(unsigned long)n);Send_Bluetooth_Data(ack);return 1;}
    if(!strncmp(line,"tele ",5)){
        if(!number(line+5,&n)||n>3){Diag_Reject("ERR tele\r\n");return 1;}
        BLE_FlushPending();Diag_mode=(uint8_t)n;motor_reset();sample=0;config_pending=0;sprintf(ack,"OK tele %lu\r\n",(unsigned long)n);Send_Bluetooth_Data(ack);return 1;}
    if(!strncmp(line,"rate ",5)){
        if(!number(line+5,&n)||n<1||n>10 || config_pending){Diag_Reject("ERR rate\r\n");return 1;}
        Diag_rate=(uint8_t)n;sample=0;Diag_revision++;sprintf(ack,"OK rate %lu\r\n",(unsigned long)n);Send_Bluetooth_Data(ack);return 1;}
    return 0;
}
void Diag_Poll(void){
    uint32_t now=Diag_TimeMs(),flags=128,values[16],p,input_ms,seq;
    uint8_t i;static uint8_t previous_stop=255;static uint32_t reported_drop;char event_text[64];
    BLE_TxPoll();
    if(!Get_Bluetooth_ConnectFlag())return;
    config_poll();
    motor_poll();
    if(Diag_mode==3 && BLE_FreeCritical()>=3){
        if(previous_stop!=Radar_stop_latched){sprintf(event_text,"motor_stop_latched=%u",(unsigned)Radar_stop_latched);text_event(3,event_text);previous_stop=Radar_stop_latched;}
        else if(reported_drop!=Diag_tx_drop){sprintf(event_text,"tx_drop_total=%lu",(unsigned long)Diag_tx_drop);text_event(4,event_text);reported_drop=Diag_tx_drop;}
    }
    if(Diag_mode!=3 || now-last_health<(Radar_stop_latched?200u:1000u))return;
    last_health=now;p=__get_PRIMASK();__disable_irq();input_ms=Diag_input_ms;seq=Diag_input_seq;__set_PRIMASK(p);
    if(Radar_stop_latched)flags|=1;
    if(!have_valid || !last_ok)flags|=2;
    if(Diag_tx_drop)flags|=4;
    if(config_pending)flags|=8;
    if(DIAG_INPUT_KIND)flags|=16;
    if(seq)flags|=32;if(have_valid)flags|=64;
    values[0]=now;values[1]=seq?now-input_ms:0;values[2]=have_valid?now-last_valid_ms:0;values[3]=seq;values[4]=control_seq;
    values[5]=Diag_uart_errors;values[6]=Diag_dma_errors;values[7]=Diag_rx_overflow;values[8]=0;values[9]=Diag_input_drop;
    values[10]=Diag_tx_drop;values[11]=process_max;values[12]=0;values[13]=Diag_revision;values[14]=flags;values[15]=(1u<<5)|(1u<<6)|(1u<<8)|(1u<<12);
    header(2,80);for(i=0;i<16;i++)put32(frame+14+i*4,values[i]);BLE_Queue(frame,80,1);
}
