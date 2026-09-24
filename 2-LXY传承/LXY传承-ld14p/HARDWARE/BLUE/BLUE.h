/**
 * @file    BLUE.h
 * @brief   已弃用的蓝牙头文件 — 已被 hc-05/bsp_bluetooth.h 替代
 *
 * 此文件仅作参考保留。所有蓝牙功能已迁移至HC-05驱动模块。
 */

#ifndef __BLUE_H
#define __BLUE_H

#include "sys.h"
#include "stdio.h"

extern u8 BLUE_BUFFER;
extern u8 BLUE_BUFFER_STA;

void BLUE_init(u32 bound);

#endif
