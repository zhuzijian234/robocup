
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define __DMB() ((void)0)
#define BSP_BLUETOOTH 0
#define USART_IT_TXE 1
#define ENABLE 1
#define DISABLE 0
static uint32_t Diag_session=1,tx_seq;
static uint8_t connected=1,wire[1024];static unsigned used;static int tx_enabled;
static uint32_t __get_PRIMASK(void){return 0;}
static void __disable_irq(void){}
static void __set_PRIMASK(uint32_t m){}
static void Bluetooth_Mode(void){}
static uint8_t Get_Bluetooth_ConnectFlag(void){return connected;}
static void USART_ITConfig(int p,int flag,int en){tx_enabled=en;}
static void USART_SendData(int p,uint8_t byte){wire[used++]=byte;}

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = v;
    p[1] = v >> 8;
}
static void put32(uint8_t *p, uint32_t v)
{
    put16(p, (uint16_t)v);
    put16(p + 2, (uint16_t)(v >> 16));
}
uint16_t Diag_CRC(const uint8_t *p, uint16_t n)
{
    uint16_t c = 0xffff;
    uint8_t i;
    while (n--) {
        c ^= (uint16_t)*p++ << 8;
        for (i = 0; i < 8; i++)
            c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
    return c;
}
void Diag_Finalize(uint8_t *p, uint16_t n)
{
    if (n >= 16 && p[0] == 0xaa && p[1] == 0x55 && p[2] == 2) {
        put32(p + 6, tx_seq++);
        put16(p + n - 2, Diag_CRC(p + 2, n - 4));
    }
}/* Four normal slots + two reserved for acknowledgements/events. */
typedef struct{uint8_t data[256];uint16_t len;uint8_t priority;volatile uint8_t ready;uint32_t order;} TxSlot;
static TxSlot pool[6];
static volatile int8_t active=-1;
static volatile uint16_t position;
static uint32_t order;
volatile uint32_t Diag_tx_drop;
uint8_t BLE_NormalSpace(void){uint8_t i;for(i=0;i<4;i++)if(!pool[i].ready && active!=i)return 1;return 0;}
uint8_t BLE_FreeCritical(void){uint8_t i,n=0;for(i=0;i<6;i++)if(!pool[i].ready && active!=i)n++;return n;}
uint8_t BLE_Queue(const uint8_t*p,uint16_t n,uint8_t priority){
    uint8_t i,limit=priority==2?6:priority==1?5:4;
    Bluetooth_Mode();if(!Get_Bluetooth_ConnectFlag())return 0;
    if(!n||n>256){Diag_tx_drop++;return 0;}
    for(i=0;i<limit;i++)if(!pool[i].ready && active!=i){
        memcpy(pool[i].data,p,n);pool[i].len=n;pool[i].priority=priority;pool[i].order=order++;
        __DMB();pool[i].ready=1;return 1;
    }
    Diag_tx_drop++;return 0;
}
void BLE_FlushPending(void){uint8_t i;uint32_t p=__get_PRIMASK();__disable_irq();for(i=0;i<6;i++)if(i!=active && pool[i].ready){pool[i].ready=0;Diag_tx_drop++;}__set_PRIMASK(p);}
/* 仅在关中断或USART6 ISR内调用；不会打断正在发送的完整消息。 */
static void tx_start_next(void)
{
    uint8_t i;
    int8_t best = -1;
    if (active >= 0) return;
    for (i = 0; i < 6; i++)
        if (pool[i].ready && (best < 0 || pool[i].priority > pool[best].priority ||
            (pool[i].priority == pool[best].priority && (int32_t)(pool[i].order - pool[best].order) < 0)))
            best = i;
    if (best < 0) return;
    Diag_Finalize(pool[best].data, pool[best].len);
    position = 0;
    active = best;
    __DMB();
    USART_ITConfig(BSP_BLUETOOTH, USART_IT_TXE, ENABLE);
}
void BLE_TxPoll(void)
{
    static uint8_t connected;
    uint32_t mask;
    Bluetooth_Mode();
    if (!Get_Bluetooth_ConnectFlag()) {
        if (connected) { BLE_FlushPending(); Diag_session = 0; }
        connected = 0;
        return;
    }
    connected = 1;
    mask = __get_PRIMASK();
    __disable_irq();
    tx_start_next();
    __set_PRIMASK(mask);
}
void BLE_TxIRQ(void){
    if(active<0){USART_ITConfig(BSP_BLUETOOTH,USART_IT_TXE,DISABLE);return;}
    USART_SendData(BSP_BLUETOOTH,pool[active].data[position++]);
    if(position>=pool[active].len){pool[active].ready=0;active=-1;USART_ITConfig(BSP_BLUETOOTH,USART_IT_TXE,DISABLE);tx_start_next();}
}
static int checks;
#define CHECK(c) do{checks++;if(!(c)){printf("FAIL %d: %s\n",__LINE__,#c);return 1;}}while(0)
int main(void){
    uint8_t low[16]={0xaa,0x55,2,5,16},high[16]={0xaa,0x55,2,2,16};
    int i;
    CHECK(BLE_Queue(low,16,0));CHECK(BLE_Queue(high,16,2));
    BLE_TxPoll();
    /* 不再调用main轮询，ISR仍需发送全部排队帧。 */
    for(i=0;i<32;i++)BLE_TxIRQ();
    CHECK(used==32 && wire[3]==2 && wire[19]==5 && !tx_enabled);
    CHECK(wire[6]==0 && wire[22]==1);
    CHECK((wire[14]|wire[15]<<8)==Diag_CRC(wire+2,12));
    CHECK((wire[30]|wire[31]<<8)==Diag_CRC(wire+18,12));
    CHECK(BLE_FreeCritical()==6);
    for(i=0;i<4;i++)CHECK(BLE_Queue(low,16,0));
    CHECK(!BLE_Queue(low,16,0));
    CHECK(BLE_Queue(high,16,1));CHECK(BLE_Queue(high,16,2));
    BLE_TxPoll();
    for(i=0;i<96;i++)BLE_TxIRQ();
    CHECK(BLE_FreeCritical()==6 && !tx_enabled);
    CHECK(Diag_tx_drop==1);
    printf("PASS %d TX scheduling/CRC checks (production C)\n",checks);
    return 0;
}
