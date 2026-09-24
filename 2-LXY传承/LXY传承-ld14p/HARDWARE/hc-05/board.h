/**
 * @file    board.h
 * @brief   板级引脚定义占位头文件
 *
 * 供 bsp_bluetooth.c 包含使用。如需添加LED、按键等板级定义，可在此扩展。
 */
/*蓝牙驱动文件期望有一个"板级配置文件"来定义板子上的硬件资源。
这是立创开源 BSP 的标准写法——每个项目都应该有自己的 board.h 来放具体的硬件定义。
但移植到这个项目时，作者没有往里填内容，所有引脚定义直接写在了 bsp_bluetooth.h 里。*/
#ifndef _BOARD_MINIMAL_H_
#define _BOARD_MINIMAL_H_

#include "stm32f4xx.h"

#endif
