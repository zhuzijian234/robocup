/**
 * @file    test_motor.c
 * @brief   测试② 电机功能测试 — 正反转多档占空比 + 编码器实测速度
 *
 * 依赖: main.c 已完成 Moto_Init/Encoder_Init, 且测试分支已关闭TIM5更新中断
 *       (否则速度环每10ms会覆盖电机PWM, 测试输出不生效)。
 *
 * 流程:
 *   [正转] PB10=高, 占空比20/40/60/80%各1.5秒, 每100ms采样一次速度
 *   [停]   占空比0, 停1秒
 *   [反转] PB10=低, 同样扫20%→80%
 *   [停]   占空比0 → 从头循环
 *
 * 速度公式与 moto.c 一致: speed = 编码器计数 × 100 / (4 × 11 × 6.25)
 *   原公式是10ms窗口; 本测试100ms采一次, 计数是10ms窗口的10倍
 *   → speed = cnt × 100 / (275 × 10), 打印再×10 → cnt × 1000 / 2750
 *
 * 编码器反转时CNT是补码负数(65535表示-1) → 先转int16_t再算, 否则算出超大正数。
 *
 * 预期现象 (对照表见 硬件功能测试方案.md §4.3):
 *   正转 speed>0, 反转 speed<0; 手拨轮子方向与符号一致;
 *   20%不转说明最小启动占空比更高, 记下实际值。
 *
 * ⚠️ 测试前把车架起来! 80%占空比下轮子转速很高。
 */

#include "test.h"
#include "stm32f4xx.h"
#include "moto.h"
#include "delay.h"
#include "stdio.h"

#define TEST_MOTO_SAMPLE_N     10   /* 采样窗口 = 10ms × 10 = 100ms */
#define TEST_MOTO_SAMPLE_TIMES 15   /* 每档占空比采15次 × 100ms = 1.5秒 */

/**
 * @brief  扫一个方向: dir=1正转(PB10高), dir=0反转(PB10低)
 */
static void Test_Moto_Sweep_One_Direction(uint8_t dir)
{
    uint16_t duty;

    if (dir == 1) {
        GPIO_SetBits(GPIOB, GPIO_Pin_10);    /* PB10高 = 正转 */
        printf("\r\n[正转] PB10=高, 占空比20%%→80%%\r\n");
    } else {
        GPIO_ResetBits(GPIOB, GPIO_Pin_10);  /* PB10低 = 反转 */
        printf("\r\n[反转] PB10=低, 占空比20%%→80%%\r\n");
    }

    for (duty = 20; duty <= 80; duty += 20) {
        uint8_t i;
        Moto_Speed(duty);                    /* 设置电机PWM占空比 */
        printf("占空比 %d%%:\r\n", (int)duty);

        for (i = 0; i < TEST_MOTO_SAMPLE_TIMES; i++) {
            int16_t scnt;
            int32_t speed_x10;
            int32_t ab;

            delay_ms(100);

            /* 读编码器并清零 (反转时CNT是补码负数, 转int16_t还原符号) */
            scnt = (int16_t)TIM_GetCounter(TIM4);
            TIM_SetCounter(TIM4, 0);

            /* speed×10 = cnt × 1000 / (4×11×6.25×采样窗口倍数), MicroLIB不支持%f所以×10存整数 */
            speed_x10 = (int32_t)(scnt * 1000.0f / (4.0f * 11.0f * 6.25f * TEST_MOTO_SAMPLE_N));

            /* 拆成 "整数.小数" 打印, 负数时小数位用绝对值 */
            ab = (speed_x10 < 0) ? -speed_x10 : speed_x10;
            printf("  计数:%6d  速度:%4d.%d\r\n", (int)scnt, (int)(speed_x10 / 10), (int)(ab % 10));
        }
    }
}

void Test_Motor_Run(void)
{
    printf("\r\n===== 测试② 电机测试 =====\r\n");
    printf("注意: 车要架起来! 电机PWM已由main.c交给本测试独占(TIM5已关)\r\n");

    while (1) {
        Test_Moto_Sweep_One_Direction(1);    /* 正转扫一遍 */
        Moto_Speed(0);                       /* 停 */
        printf("\r\n[停止1秒]\r\n");
        delay_ms(1000);

        Test_Moto_Sweep_One_Direction(0);    /* 反转扫一遍 */
        Moto_Speed(0);                       /* 停 */
        printf("\r\n[停止1秒]\r\n");
        delay_ms(1000);
    }
}
