#ifndef BLE_DIAG_H
#define BLE_DIAG_H
#include "stm32f4xx.h"
#define DIAG_LAYOUT "rc26-mask31-v2.1"
#define DIAG_LIDAR_ID 1u /* 1 LD14P, 2 M10, 3 M10P; parser must match */
#define DIAG_INPUT_KIND 0u /* 0 DMA block, 1 complete scan */
#ifndef BLE_UART_BAUD
#define BLE_UART_BAUD 115200u
#endif
extern uint8_t Diag_mode, Diag_rate;
extern uint32_t Diag_revision, Diag_session;
extern volatile uint32_t Diag_input_seq, Diag_input_ms, Diag_input_us, Diag_input_drop;
extern volatile uint32_t Diag_uart_errors, Diag_dma_errors, Diag_rx_overflow;
extern volatile uint32_t Diag_tx_drop;
uint16_t Diag_CRC(const uint8_t *p, uint16_t n);
int16_t Diag_Encode(uint8_t index, float v, uint8_t *valid, uint8_t *clipped);
void Diag_Init(void);
uint32_t Diag_TimeUs(void);
uint32_t Diag_TimeMs(void);
void Diag_InputDone(void); /* DMA ISR: timestamp/counter only */
void Diag_Begin(uint32_t input_ms, uint32_t input_us);
void Diag_Field(uint8_t i, float value, uint8_t valid);
void Diag_HeldField(uint8_t i,float value,uint8_t valid);
void Diag_Fit(const void *line, uint8_t valid);
void Diag_Submit(uint16_t mode, uint16_t raw, uint16_t speed, uint8_t perception_ok);
void Diag_Poll(void);
uint8_t Diag_Command(char *line);
uint8_t Diag_Parameter(uint16_t key, float *ptr, float value);
void Diag_ConfigStart(void);
void Diag_Reject(const char *reason);
/* Main-context producers, ISR consumes bytes only. */
uint8_t BLE_Queue(const uint8_t *p, uint16_t len, uint8_t priority);
uint8_t BLE_FreeCritical(void);
uint8_t BLE_NormalSpace(void);
void BLE_TxPoll(void);
void BLE_TxIRQ(void);
void BLE_FlushPending(void);
void Diag_Finalize(uint8_t *p, uint16_t n);
/* Parameter table adapter, implemented by ble_tune.c. */
uint8_t Tune_ConfigValue(uint16_t index, uint16_t *key, float *value);
#endif
