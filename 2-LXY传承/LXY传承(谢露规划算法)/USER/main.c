/**
 * @file    main.c
 * @brief   雷达小车主程序 — 初始化、主控制循环、多模式转向决策
 *
 * ======================== 系统架构 ========================
 *
 * 硬件平台: STM32F407VET6 @ 168MHz
 * 传感器:   M10系列激光雷达 (USART2, 230400bps, DMA接收)
 * 执行器:   舵机 (TIM3 CH1), 直流电机 (TIM2 CH1, 编码器 TIM4)
 * 通信:     HC-05蓝牙 (USART6)
 *
 * 引脚复用 (对应谢露版):
 *   雷达:     USART2, PA2(TX/被TIM9覆盖) PA3(RX/DMA), DMA1 Stream5 Channel4
 *   蓝牙:     USART6, PC6(TX) PC7(RX)
 *   舵机:     TIM3 CH1, PA6
 *   电机PWM:  TIM2 CH1, PA5  [2026-09-06 PB11杜邦线故障, 临时挪至PA5]
 *   电机方向: PB15 (单IO, 高=正转)  [2026-09-06 PB10杜邦线故障, 临时挪至PB15]
 *   编码器:   TIM4, PD12 PD13
 *   雷达电机: TIM9 CH1, PA2 (⚠ PA2与USART2_TX共用, 初始化顺序保证TIM9后初始化)
 *   Debug:    USART1, PA9(TX) PA10(RX)
 *
 * 主循环流程:
 *   1. 等待 DMA_RX_DONE (一帧雷达数据就绪)
 *   2. 解析雷达数据 -> 极坐标 -> 筛选有效点
 *   3. 计算跑道宽度
 *   4. 提取左/右边界点
 *   5. 扫描前方路径
 *   6. 检测左右边界突变点 (弯道入口)
 *   7. 根据断点位置和前方斜率选择控制模式 (pid_select)
 *   8. 调用中线PD控制器 -> 舵机PWM
 *   9. 速度PI控制由TIM5中断独立运行 (10ms周期)
 *
 * 控制模式决策逻辑:
 *   - 无断点 -> 中线模式 (pid=0) / 垂线模式 (pid=5)
 *   - 右断点 + 前方斜率<0.35 -> 大右转 (pid=3)
 *   - 右断点 + 前方斜率0.35~0.7 + S弯特征 -> 中右转 (pid=8)
 *   - 右断点 + 前方斜率>=0.7 -> 小右转 (pid=1)
 *   - 左断点 + 前方斜率<0.35 -> 大左转 (pid=4)
 *   - 左断点 + 前方斜率0.35~0.7 + S弯特征 -> 中左转 (pid=9)
 *   - 左断点 + 前方斜率>=0.7 -> 小左转 (pid=2)
 *
 * 调试开关:
 *   HW_TEST_SELECT: 0=正常循线, 1=舵机测试, 2=电机测试, 3=蓝牙调参测试
 *   (测试代码见 test/ 目录, 说明见 硬件功能测试方案.md)
 */

#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "DMA.h"
#include "leida_pwm.h"
#include "LEIDA_DATA.h"
#include "bsp_bluetooth.h"
#include "ble_tune.h"           /* 蓝牙实时调参 (HARDWARE/hc-05/ble_tune.c) */
#include "centre_line.h"
#include "timer.h"
#include "moto.h"
#include "Servo.h"
#include "PWM.h"
#include "xielu.h"             /* 谢露舵机算法 (HARDWARE/XIELU/xielu.c) */
#include "test.h"

/* ======================== 硬件测试宏 ========================
 * HW_TEST_SELECT 选择运行模式:
 *   0: 正常循线 (默认)
 *   1: 测试① 舵机往复扫描 (test/test_servo.c)
 *   2: 测试② 电机正反转+测速 (test/test_motor.c)
 *   3: 测试③ 蓝牙调参链路 (test/test_bluetooth.c)
 * 测试模式的说明与预期现象表见 硬件功能测试方案.md。
 */
#define HW_TEST_SELECT 0
/* 全局变量 */
uint16_t RIGHT_duandian;           /* 右边界断点y坐标 */
uint16_t LEFT_duandian;            /* 左边界断点y坐标 */
float paodao_distance       = 700; /* 跑道宽度 (mm) */
float paodao_distance_r     = 700; /* 当前帧跑道宽度 (mm) */
float paodao_distance_r_r   = 0;
float paodao_distance_r_r_r = 0;
uint16_t state_left_cnt     = 0;   /* 左转持续计数 (大角度强制) */
uint16_t state_right_cnt    = 0;   /* 右转持续计数 (大角度强制) */
uint16_t state_left_cnt_2   = 0;   /* 左转二级计数 (连续大左转后强制) */
uint16_t state_right_cnt_2  = 0;   /* 右转二级计数 (连续大右转后强制) */

extern float Speed_now;

#define duandian_distance 600       /* 断点判断距离阈值 */
uint16_t state_sta = 1;

float duandian_DIStance = 600;     /* 断点有效距离阈值 (mm) */

int main(void)
{
    /* 初始化调试串口: USART1(PA9/PA10, 保留备用) + USART3(PC10/PC11, printf已重定向) */
    uart_init(115200);
    uart3_init(115200);                              /* 调试串口2 (PC10=TX, PC11=RX) */

    u32 t                 = 0;
    uint16_t lidar_cnt    = 0;     /* 本帧雷达解析点数 (HANDLE1返回, 谢露流水线用) */
    float jiaodu_piancha;
    float qulu_forward;
    float qulu_jinduan;

    /* ===== 系统初始化 ===== */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  /* 2位抢占，2位子优先级 */
    delay_init(168);                                 /* 延时函数初始化 (参数=主频MHz, 本工程168MHz) */

    /* 注意初始化顺序: USART2先初始化(PA2=USART2 AF),
     * PWM_Init_leida后初始化(PA2=TIM9 CH1 AF, 覆盖USART2_TX)。
     * 雷达用PA3(RX)+DMA接收数据, USART2_TX不需要 */
    uart2_init(230400);                               /* 雷达串口USART2初始化 (先于TIM9) */
    DMA_Initializes();                                /* DMA初始化 (USART2雷达接收) */
    Bluetooth_Init();                                 /* 蓝牙初始化 (USART6, PC6/PC7) */
    PWM_Init_leida();                                 /* 雷达电机PWM初始化 (TIM9 CH1, PA2, 覆盖USART2_TX) */
    PWM_SetCompare_leida(97);                         /* 雷达电机初始占空比 */

    Servo_Init(84, 20000, SERVO_PWM_MID);             /* 舵机初始化 (50Hz), 上电居中1445 */

    /* 速度PID初始化: kp, ki, kd */
    Speed_PID_Init(&Speed_pid, 8.5, 0.505, 0);

    /* 电机初始化: 预分频42, 周期100 */
    moto_pwm = (uint16_t)(90 * 1);
#if HW_TEST_SELECT == 0
    Moto_Init(42, 100, moto_pwm);   /* 正常循线: 初始90%, 之后由速度环接管 */
#else
    Moto_Init(42, 100, 0);          /* 测试模式: 初始0%, 防止上电轮子就转 */
#endif

    Encoder_Init();                                   /* 编码器初始化 (TIM4) */

    /* 速度定时器初始化: 84MHz/8400=10kHz, 周期100=10ms */
    TIM5_Int_Init(100 - 1, 8400 - 1);

#if HW_TEST_SELECT != 0
    /* 测试模式: 关闭速度环中断。
     * TIM5每10ms会写一次电机PWM(Moto_Speed), 会覆盖测试代码的输出,
     * 所以测试模式下必须关掉, 电机PWM完全交给测试代码控制。
     * (舵机测试也不受影响: 关掉后电机保持0%占空比, 车原地不动) */
    TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
#endif

    /* ===== 运行参数配置 ===== */
    Speed_mubiao = 8;                   /* 目标速度 */

    /* 10模式中线PD与BLUE_*参数已退役: 舵机控制已换为谢露方案 (HARDWARE/XIELU/xielu.c) */
    printf("Start\r\n");

    /* ======================== 主循环 ======================== */
    while (1) {

#if HW_TEST_SELECT == 1
        /* 测试① 舵机往复扫描 (函数自带死循环, 不会返回) */
        Test_Servo_Sweep();
#elif HW_TEST_SELECT == 2
        /* 测试② 电机正反转+测速 */
        Test_Motor_Run();
#elif HW_TEST_SELECT == 3
        /* 测试③ 蓝牙调参链路 */
        Test_Bluetooth_Tune();
#else
        /* ======================== 正常循线模式 ======================== */

        BLE_Tune_Process();     /* 蓝牙命令解析: 每圈必跑(含雷达帧等待圈), 命令响应<1ms */

        /* 等待DMA接收完一帧雷达数据 */
        if (DMA_RX_DONE) {
            DMA_RX_DONE = 0;  /* 清除标志 */

            printf("DMA_RX_DONE\r\n");

            /* ===== 第1步: 解析雷达数据 (返回本帧点数) ===== */
            lidar_cnt = LEIDA_DATA_HANDLE1(LEIDA_DATA, DMA_USART2_RX_BUF_r, DMA_USART2_RX_BUF_LEN);

            /* ===== 谢露规划流水线 + 三分支 + qianfang30_PD (HARDWARE/XIELU/xielu.c) ===== */
            /* 帧同步失败返回0 / 点数太少: 跳过本帧, 舵机保持当前位置 */
            if (lidar_cnt > 200) {
                XIELU_ProcessFrame(lidar_cnt);
                /* 本帧雷达处理完: 回传7通道波形给手机/VOFA+ (未连接时内部直接返回, 零开销) */
                BLE_Tune_Telemetry(XIELU_GetErr(), XIELU_GetServoPwm(), XIELU_GetPidSlt());
            }

        } else {
            /* DMA数据未就绪，等待 */
            ;
        }

#endif
    }
}
