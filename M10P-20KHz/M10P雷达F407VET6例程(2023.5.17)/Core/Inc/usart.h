/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-09-01

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-09-01

All rights reserved
***********************************************/
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
#define HEADER_0 0xA5
#define HEADER_1 0x5A

#define END_0 0xFA
#define END_1 0xFB

#define POINT_PER_PACK 255
/* USER CODE END Private defines */

void MX_UART5_Init(void);
void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */


typedef struct PackData
{
	uint8_t header_0;//
	uint8_t header_1;//
	u16 Len_H;//数字帧长度高位
	u16 Len_L;//数字帧长度地位
	u16 angle_H;//开始角度
	u16 angle_L;//结束角度
	u16 speed_H;//速度高位
	u16 speed_L;//速度低位
	u16 point[POINT_PER_PACK];
	uint8_t end_0;
	uint8_t end_1;
}LiDARFrameTypeDef;


typedef struct PointDataProcess_
{
	u16 distance;
	float angle;
}PointDataProcessDef;

extern PointDataProcessDef PointDataProcess[70];//更新70个数据
extern LiDARFrameTypeDef Pack_Data;

void usart1_send(u8 data);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
