/**
 * @file    bsp_bluetooth.h
 * @brief   HC-05蓝牙模块驱动 — 引脚定义与API声明
 *
 * 硬件: USART6 (PC6=TX, PC7=RX), 波特率9600
 * 连接状态引脚: PC2 (HC-05的STATE引脚, 高=已连接, 低=未连接)
 *
 * 对应谢露版引脚复用: 蓝牙从USART3移至USART6 (但谢露版蓝牙未启用)
 *
 * 原始出处: 立创开发板开源BSP (www.lckfb.com)，已适配本项目。
 */

#ifndef _BSP_BLUETOOTH_H_
#define _BSP_BLUETOOTH_H_

#include "stm32f4xx.h"
#include "string.h"
#include "board.h"

#define DEBUG                    1       /* 调试串口开关: 1=开启, 0=关闭 */

#define BLERX_LEN_MAX            200     /* 蓝牙接收缓冲区最大长度 */

/* USART6 引脚和时钟定义 (PC6=TX, PC7=RX) */
#define BSP_BLUETOOTH_TX_RCC     RCC_AHB1Periph_GPIOC
#define BSP_BLUETOOTH_RX_RCC     RCC_AHB1Periph_GPIOC
#define BSP_BLUETOOTH_RCC        RCC_APB2Periph_USART6

#define BSP_BLUETOOTH_TX_PORT    GPIOC
#define BSP_BLUETOOTH_RX_PORT    GPIOC
#define BSP_BLUETOOTH_AF         GPIO_AF_USART6

#define BSP_BLUETOOTH_TX_PIN     GPIO_Pin_6
#define BSP_BLUETOOTH_TX_SOURCE  GPIO_PinSource6
#define BSP_BLUETOOTH_RX_PIN     GPIO_Pin_7
#define BSP_BLUETOOTH_RX_SOURCE  GPIO_PinSource7

#define BSP_BLUETOOTH            USART6
#define BSP_BLUETOOTH_IRQ        USART6_IRQn
#define BSP_BLUETOOTH_IRQHandler USART6_IRQHandler

/* HC-05 STATE 引脚: 高电平=已连接, 低电平=未连接 */
#define BLUETOOTH_LINK_RCC  RCC_AHB1Periph_GPIOC
#define BLUETOOTH_LINK_PORT GPIOC
#define BLUETOOTH_LINK_GPIO GPIO_Pin_2

#define BLUETOOTH_LINK      GPIO_ReadInputDataBit(BLUETOOTH_LINK_PORT, BLUETOOTH_LINK_GPIO)

#define CONNECT             1       /* 蓝牙已连接 */
#define DISCONNECT          0       /* 蓝牙已断开 */

extern unsigned char BLERX_BUFF[BLERX_LEN_MAX];
extern unsigned char BLERX_FLAG;
extern unsigned char BLERX_LEN;

void Bluetooth_Init(void);
unsigned char Get_Bluetooth_ConnectFlag(void);
void Bluetooth_Mode(void);
void Receive_Bluetooth_Data(void);
void BLE_send_String(unsigned char *str);
void Clear_BLERX_BUFF(void);
void Send_Bluetooth_Data(char *dat);

#endif
