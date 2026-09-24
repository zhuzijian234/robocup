#include "ble_diag.h"
#include "bsp_bluetooth.h"
#include "string.h"
/* Four normal slots + two reserved for acknowledgements/events. */
typedef struct{uint8_t data[256];uint16_t len;uint8_t priority;volatile uint8_t ready;uint32_t order;} TxSlot;
static TxSlot pool[6];
static volatile int8_t active=-1;
static volatile uint16_t position;
static uint32_t order;
volatile uint32_t Diag_tx_drop;
uint8_t BLE_NormalSpace(void){uint8_t i;for(i=0;i<4;i++)if(!pool[i].ready && active!=i)return 1;return 0;}
uint8_t BLE_FreeCritical(void){uint8_t i,n=0;for(i=0;i<6;i++)if(!pool[i].ready && active!=i)n++;return n;}
uint8_t BLE_Queue(const uint8_t*p,uint16_t n,uint8_t priority){
    uint8_t i,limit=priority==2?6:4;
    Bluetooth_Mode();if(!Get_Bluetooth_ConnectFlag())return 0;
    if(!n||n>256){Diag_tx_drop++;return 0;}
    for(i=0;i<limit;i++)if(!pool[i].ready && active!=i){
        memcpy(pool[i].data,p,n);pool[i].len=n;pool[i].priority=priority;pool[i].order=order++;
        __DMB();pool[i].ready=1;return 1;
    }
    Diag_tx_drop++;return 0;
}
void BLE_FlushPending(void){uint8_t i;uint32_t p=__get_PRIMASK();__disable_irq();for(i=0;i<6;i++)if(i!=active && pool[i].ready){pool[i].ready=0;Diag_tx_drop++;}__set_PRIMASK(p);}
void BLE_TxPoll(void){
    uint8_t i;int8_t best=-1;static uint8_t connected;
    Bluetooth_Mode();
    if(!Get_Bluetooth_ConnectFlag()){
        if(connected){BLE_FlushPending();Diag_session=0;}
        connected=0;return;
    }
    connected=1;if(active>=0)return;
    for(i=0;i<6;i++)if(pool[i].ready && (best<0 || pool[i].priority>pool[best].priority ||
       (pool[i].priority==pool[best].priority && (int32_t)(pool[i].order-pool[best].order)<0)))best=i;
    if(best<0)return;
    Diag_Finalize(pool[best].data,pool[best].len);
    position=0;active=best;__DMB();USART_ITConfig(BSP_BLUETOOTH,USART_IT_TXE,ENABLE);
}
void BLE_TxIRQ(void){
    if(active<0){USART_ITConfig(BSP_BLUETOOTH,USART_IT_TXE,DISABLE);return;}
    USART_SendData(BSP_BLUETOOTH,pool[active].data[position++]);
    if(position>=pool[active].len){pool[active].ready=0;active=-1;USART_ITConfig(BSP_BLUETOOTH,USART_IT_TXE,DISABLE);}
}
