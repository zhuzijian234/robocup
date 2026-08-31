/**
 * @file    ble_tune.h
 * @brief   蓝牙实时调参模块 — 接口声明
 *
 * 功能: 手机/电脑经 HC-05 蓝牙(USART6 @9600)在车跑动时实时修改控制参数。
 * 实现见 ble_tune.c, 使用说明见 蓝牙调参说明书.md。
 */

#ifndef _BLE_TUNE_H_
#define _BLE_TUNE_H_

#include "stm32f4xx.h"

void BLE_Tune_Process(void);                                              /* 主循环循线分支每圈调用: 解析蓝牙命令 */
void BLE_Tune_Telemetry(float err, float servo_pwm, uint16_t pid_mode);   /* 每帧雷达处理完调用: 回传7通道波形 */

#endif
