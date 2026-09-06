/**
 * @file    test_bluetooth.c
 * @brief   测试③ 蓝牙调参链路测试 — 实时遥测 + 手机命令实时改舵机/电机
 *
 * 依赖: main.c 已完成 Bluetooth_Init 等初始化, 且测试分支已关闭TIM5更新中断
 *       (否则手机设置的电机占空比会在10ms内被速度环覆盖)。
 *
 * 功能:
 *   1. 检测HC-05连接状态(STATE脚), 连接/断开变化时打印
 *   2. 已连接时每500ms发一条状态行(蓝牙+调试串口各一份):
 *        s:1565 m:30 v:125    = 舵机PWM / 电机占空比% / 速度×10
 *   3. 手机发命令实时改参数:
 *        s1500   设置舵机PWM (范围见test.h的TEST_SERVO_MIN/MAX)
 *        m50     设置电机占空比% (0~100)
 *        ?       立即回发一条状态行
 *        其他    回 "ERR fmt"
 *
 * 命令解析: 跳过第一个数字前的所有非数字字符再atoi (兼容 s1500 / s:1500 / s 1500);
 *           一个数字都没有 → ERR fmt (防 atoi 对全非数字返回0, 0被误当合法值)。
 *
 * 预期现象表见 硬件功能测试方案.md §5.4。
 * ⚠️ 手机发 m 命令前把车架起来。
 */

#include "test.h"
#include "stm32f4xx.h"
#include "bsp_bluetooth.h"
#include "Servo.h"
#include "moto.h"
#include "delay.h"
#include "stdio.h"
#include "stdlib.h"    /* atoi */

#define TEST_BLE_TICK_PERIOD 50   /* 遥测周期 = 50次主循环 × 10ms = 500ms */

/* 当前参数 (文件级static: 函数退出后仍保留) */
static uint16_t servo_pwm_now = TEST_SERVO_MID;  /* 当前舵机PWM */
static uint16_t moto_pwm_now  = 0;               /* 当前电机占空比% */
static int32_t  speed_x10     = 0;               /* 当前速度×10 (整数, MicroLIB不支持%f) */

/**
 * @brief  采样编码器算速度 (与test_motor.c同理, 这里是500ms窗口)
 */
static void Test_BLE_Sample_Speed(void)
{
    int16_t scnt = (int16_t)TIM_GetCounter(TIM4);
    TIM_SetCounter(TIM4, 0);
    /* 500ms窗口 = 10ms的50倍 → speed×10 = cnt × 1000 / (4×11×6.25×50) */
    speed_x10 = (int32_t)(scnt * 1000.0f / (4.0f * 11.0f * 6.25f * TEST_BLE_TICK_PERIOD));
}

/**
 * @brief  发一条状态行: 蓝牙一份 + 调试串口一份
 */
static void Test_BLE_Status_Send(void)
{
    char buf[64];
    sprintf(buf, "s:%d m:%d v:%d\r\n", (int)servo_pwm_now, (int)moto_pwm_now, (int)speed_x10);
    BLE_send_String((unsigned char *)buf);
    printf("%s", buf);
}

/**
 * @brief  回发一条消息 (蓝牙一份 + 调试串口一份)
 */
static void Test_BLE_Reply(char *msg)
{
    BLE_send_String((unsigned char *)msg);
    printf("回复: %s", msg);
}

/**
 * @brief  处理一帧收到的命令 (BLERX_BUFF里以'\0'结尾)
 */
static void Test_BLE_Handle_Command(void)
{
    char buf[64];
    char *p;
    int val;
    unsigned char cmd = BLERX_BUFF[0];

    /* 命令'?': 立即回发当前状态 */
    if (cmd == '?') {
        Test_BLE_Status_Send();
        return;
    }

    /* 跳过非数字前缀, 找到第一个数字 (兼容 s1500 / s:1500 / s 1500) */
    p = (char *)BLERX_BUFF;
    while (*p && (*p < '0' || *p > '9')) p++;

    /* 一个数字都没有 → ERR fmt (防atoi对全非数字返回0, 0被误当合法值) */
    if (*p == '\0') {
        Test_BLE_Reply("ERR fmt, use: s1500 / m50 / ?\r\n");
        return;
    }
    val = atoi(p);

    if (cmd == 's' || cmd == 'S') {
        /* 舵机命令: 范围检查后立即执行 */
        if (val < TEST_SERVO_MIN || val > TEST_SERVO_MAX) {
            sprintf(buf, "servo out of range [%d,%d]\r\n", TEST_SERVO_MIN, TEST_SERVO_MAX);
            Test_BLE_Reply(buf);
        } else {
            servo_pwm_now = (uint16_t)val;
            Servo_ChangePwm(servo_pwm_now);
            sprintf(buf, "servo=%d OK\r\n", (int)servo_pwm_now);
            Test_BLE_Reply(buf);
        }
    } else if (cmd == 'm' || cmd == 'M') {
        /* 电机命令: 范围检查后立即执行 */
        if (val < 0 || val > 100) {
            Test_BLE_Reply("moto out of range [0,100]\r\n");
        } else {
            moto_pwm_now = (uint16_t)val;
            Moto_Speed(moto_pwm_now);
            sprintf(buf, "moto=%d%% OK\r\n", (int)moto_pwm_now);
            Test_BLE_Reply(buf);
        }
    } else {
        Test_BLE_Reply("ERR fmt, use: s1500 / m50 / ?\r\n");
    }
}

void Test_Bluetooth_Tune(void)
{
    uint32_t tick = 0;
    static uint8_t last_connect = 0;   /* 上一次的连接状态, 用于检测变化 */

    printf("\r\n===== 测试③ 蓝牙调参测试 =====\r\n");
    printf("等待手机连接HC-05...\r\n");
    printf("命令格式:\r\n");
    printf("  s1500  设置舵机PWM (范围%d~%d)\r\n", TEST_SERVO_MIN, TEST_SERVO_MAX);
    printf("  m50    设置电机占空比%% (0~100)\r\n");
    printf("  ?      查询当前状态\r\n");

    while (1) {
        tick++;

        /* 1. 刷新连接状态, 变化时打印 */
        Bluetooth_Mode();
        if (Get_Bluetooth_ConnectFlag() != last_connect) {
            last_connect = Get_Bluetooth_ConnectFlag();
            if (last_connect == 1) {
                printf("\r\n蓝牙已连接! 开始遥测\r\n");
                Test_BLE_Status_Send();       /* 一连上就先发一条 */
            } else {
                printf("\r\n蓝牙已断开!\r\n");
            }
        }

        /* 2. 已连接: 每500ms采样一次速度并发一条状态行 */
        if ((Get_Bluetooth_ConnectFlag() == 1) && (tick % TEST_BLE_TICK_PERIOD == 0)) {
            Test_BLE_Sample_Speed();
            Test_BLE_Status_Send();
        }

        /* 3. 收到一帧命令 → 解析执行 → 清标志等下一帧 */
        if (BLERX_FLAG == 1) {
            printf("收到命令: %s\r\n", BLERX_BUFF);   /* 调试串口回显 */
            Test_BLE_Handle_Command();
            Clear_BLERX_BUFF();
        }

        delay_ms(10);   /* 主循环节奏: 10ms一拍 */
    }
}
