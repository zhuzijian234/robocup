/** STM32F407ZG / M10P：完整扫描 → 局部路径 → 转向；TIM5 独立运行定速 PI。
 * 雷达 PA2/PA3 USART2；蓝牙 PC6/PC7 USART6；舵机 PA6 TIM3_CH1；
 * 电机 PA1 TIM2_CH2、方向 PB15；编码器 PD12/PD13 TIM4。
 */
#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "DMA.h"
#include "LEIDA_DATA.h"
#include "bsp_bluetooth.h"
#include "ble_tune.h"
#include "ble_diag.h"
#include "centre_line.h"
#include "timer.h"
#include "moto.h"
#include "Servo.h"
#include "test.h"
#include "m10p_vehicle.h"

/* 0 循迹；1 舵机；2 电机；3 蓝牙；4 PA1 高/PB15 低静态测量。 */
#define HW_TEST_SELECT 0
#if HW_TEST_SELECT == 4
/**
 * PA1(PWM) + PB15(方向) 纯GPIO测试：不初始化雷达、蓝牙、舵机及电机定时器。
 * PA1持续输出高电平、PB15持续输出低电平，用于检查新PWM引脚的拉高能力。
 * 万用表直流电压档：黑表笔接STM32 GND，红表笔分别接PA1、PB15。
 * 预期：PA1恒定约3.3V、PB15恒定约0V，不产生PWM或高低切换。
 * 测量时断开驱动板控制线；PA1恒高若接到驱动板，可能使电机全速转动。
 */
int main(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
    /* 先写好两个引脚的输出锁存器(PA1=高, PB15=低), 再打开输出,
     * 避免初始化瞬间出现电平跳变。 */
    GPIO_SetBits(GPIOA, GPIO_Pin_1);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_OUT; /* 普通推挽输出，不使用AF1/PWM */
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOB, &gpio);

    while (1)
    {
        /* GPIO锁存器保持初始化电平；不使用延时或定时器。 */
    }
}
#else
int main(void)
{
    const M10P_Scan *scan;
    uint16_t count, mode, left_opening, right_opening;
    uart_init(115200);
    uart3_init(115200);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Diag_Init();
    delay_init(168);
    M10P_Init();
    uart2_init(M10P_BAUD);
    DMA_Initializes();
    Bluetooth_Init();
    Servo_Init(84, 20000, SERVO_PWM_MID);
    Steering_Init();
    Speed_PID_Init(&Speed_pid, 8.5f, 0.505f, 0);
    Moto_Init(42, 100, 0);
    Encoder_Init();
    Speed_mubiao = 10; /* 固定目标；本轮不改速度或恢复自动停车。 */
#if HW_TEST_SELECT == 0
    TIM5_Int_Init(100 - 1, 8400 - 1);
#endif
    while (1)
    {
#if HW_TEST_SELECT == 1
        Test_Servo_Sweep();
#elif HW_TEST_SELECT == 2
        Test_Motor_Run();
#elif HW_TEST_SELECT == 3
        Test_Bluetooth_Tune();
#else
        Diag_Poll();
        BLE_Tune_Process();
        M10P_Poll();
        scan = M10P_Acquire();
        if (!scan)
            continue;
        Diag_input_seq = scan->seq;
        Diag_input_us = scan->end_us;
        Diag_input_ms = Diag_TimeMs() - (uint32_t)(Diag_TimeUs() - scan->end_us) / 1000u;
        Diag_Begin(Diag_input_ms, scan->end_us);
        Diag_detail_u[3] = scan->seq;
        LEIDA_raw_count = scan->count;
        LEIDA_speed_dps = scan->dps;
        valid_couter = M10P_Build(scan, LEIDA_DATA2, LEIDA_DATA_COUNTER);
        LEIDA_Opening(LEIDA_DATA2, valid_couter, &left_opening, &right_opening);
        Diag_Field(9, right_opening, 1);
        Diag_Field(10, left_opening, 1);
        count = LEIDA_FrontPoints(LEIDA_DATA2, valid_couter);
        Steering_Update(scan, LEIDA_front_points, count);
        if (Steering_command.applied)
        {
            mode = Steering_path.source; /* algorithm=4：0 双侧、1 左侧、2 右侧 */
            Radar_RecordControl(scan->end_us);
        }
        else
        {
            mode = Steering_command.computed ? BLE_MODE_HOLD : BLE_MODE_INVALID;
            Radar_invalid_inputs++;
        }
        Diag_Field(12, valid_couter, 1);
        if (Steering_path.width_measured)
            Diag_Field(11, Steering_path.width, 1);
        else
            Diag_HeldField(11, Steering_path.width, 1);
        Diag_Submit(mode, scan->count, scan->dps, Steering_command.applied);
        Diag_CloudCapture(LEIDA_front_points, count, scan->epoch);
        BLE_Tune_Telemetry(Servo_pd.err, (float)TIM3->CCR1, mode);
        M10P_Release(scan);
#endif
    }
}
#endif
