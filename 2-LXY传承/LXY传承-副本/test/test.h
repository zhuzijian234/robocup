/**
 * @file    test.h
 * @brief   硬件功能测试模块 — 三个独立测试的公共头文件
 *
 * 由 USER/main.c 里的宏 HW_TEST_SELECT 选择运行哪个测试:
 *   #define HW_TEST_SELECT 0   正常循线 (默认)
 *   #define HW_TEST_SELECT 1   测试① 舵机往复扫描
 *   #define HW_TEST_SELECT 2   测试② 电机正反转+测速
 *   #define HW_TEST_SELECT 3   测试③ 蓝牙调参链路
 *
 * 每个测试函数内部自带 while(1) 死循环, 不会返回。
 * 使用说明与预期现象表见 硬件功能测试方案.md。
 */

#ifndef __TEST_H
#define __TEST_H

/* ===== 舵机行程参数 (测试①③共用) =====
 * 当前为谢露车: MIN=1170, MAX=1720, MID=1445。
 * 换回LXY车改成: MIN=1360, MAX=1800, MID=1565 */
#define TEST_SERVO_MIN  1170   /* 右打满 */
#define TEST_SERVO_MAX  1720   /* 左打满 */
#define TEST_SERVO_MID  1445   /* 中位 */

void Test_Servo_Sweep(void);      /* 测试① 舵机往复扫描 */
void Test_Motor_Run(void);        /* 测试② 电机正反转+测速 */
void Test_Bluetooth_Tune(void);   /* 测试③ 蓝牙调参链路 */

#endif
