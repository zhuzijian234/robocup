/**
 * @file    test_servo.c
 * @brief   测试① 舵机功能测试 — 往复扫描验证中位/行程/方向
 *
 * 依赖: main.c 已完成全部初始化 (Servo_Init 等), 且测试分支已关闭TIM5
 *       (见 main.c 里 HW_TEST_SELECT 附近注释), 所以电机不会转, 车可放地上。
 *
 * 流程:
 *   上电 → 回中位停2秒(肉眼确认轮子回正)
 *        → PWM从最小值开始, 每500ms步进20, 到最大值掉头, 往复循环
 *        → 每走一步串口打印当前PWM
 *
 * 预期现象 (对照表见 硬件功能测试方案.md §3.3):
 *   PWM变大  → 轮子往左打 (本车约定: 数值大=左)
 *   PWM=中位 → 轮子基本正前方
 *   PWM=最小/最大 → 左右打满但不应憋舵机(有持续嗡嗡声=顶到机械限位)
 *
 * 换谢露车只改 test.h 里的三个宏: MIN=1170 MAX=1720 MID=1445
 */

#include "test.h"
#include "stm32f4xx.h"   /* 必须放在Servo.h前面: Servo.h用了uint16_t但自己不include */
#include "Servo.h"
#include "delay.h"
#include "stdio.h"

#define TEST_SERVO_STEP     20    /* 每步PWM变化量 */
#define TEST_SERVO_DELAY_MS 500   /* 每步停留时间(ms) */

void Test_Servo_Sweep(void)
{
    int32_t pwm = TEST_SERVO_MID;  /* 用int32方便做减法, 输出时再转uint16_t */
    int8_t  dir = -1;              /* -1=往小(右)走, +1=往大(左)走 */

    printf("\r\n===== 测试① 舵机测试 =====\r\n");
    printf("先回中位 %d, 停2秒, 确认轮子指向正前方\r\n", TEST_SERVO_MID);
    Servo_ChangePwm((uint16_t)pwm);
    delay_ms(2000);

    printf("开始往复扫描: 范围[%d, %d], 步进%d, 每步%d毫秒\r\n",
           TEST_SERVO_MIN, TEST_SERVO_MAX, TEST_SERVO_STEP, TEST_SERVO_DELAY_MS);

    while (1) {
        Servo_ChangePwm((uint16_t)pwm);

        /* 打印当前PWM, 到端点/中位时多提示一句, 方便肉眼对照 */
        printf("Servo PWM: %d", (int)pwm);
        if (pwm == TEST_SERVO_MIN)      printf("  <- youdaman");
        else if (pwm == TEST_SERVO_MAX) printf("  <- zuodaman");
        else if (pwm == TEST_SERVO_MID) printf("  <- zhongwei");
        printf("\r\n");

        delay_ms(TEST_SERVO_DELAY_MS);

        /* 走一步, 并在端点掉头 */
        pwm += dir * TEST_SERVO_STEP;
        if (pwm >= TEST_SERVO_MAX) { pwm = TEST_SERVO_MAX; dir = -1; }
        else if (pwm <= TEST_SERVO_MIN) { pwm = TEST_SERVO_MIN; dir = 1; }
    }
}
