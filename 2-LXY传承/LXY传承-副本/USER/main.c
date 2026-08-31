/**
 * @file    main.c
 * @brief   雷达小车主程序 — 初始化、主控制循环、多模式转向决策
 *
 * ======================== 系统架构 ========================
 *
 * 硬件平台: STM32F407VET6 @ 168MHz
 * 传感器:   M10系列激光雷达 (USART2, 230400bps, DMA接收)
 * 执行器:   舵机 (TIM3 CH1), 直流电机 (TIM2 CH4, 编码器 TIM4)
 * 通信:     HC-05蓝牙 (USART6)
 *
 * 引脚复用 (对应谢露版):
 *   雷达:     USART2, PA2(TX/被TIM9覆盖) PA3(RX/DMA), DMA1 Stream5 Channel4
 *   蓝牙:     USART6, PC6(TX) PC7(RX)
 *   舵机:     TIM3 CH1, PA6
 *   电机PWM:  TIM2 CH4, PB11
 *   电机方向: PB10 (单IO, 高=正转)
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
#include "test.h"

/* ======================== 硬件测试宏 ========================
 * HW_TEST_SELECT 选择运行模式:
 *   0: 正常循线 (默认)
 *   1: 测试① 舵机往复扫描 (test/test_servo.c)
 *   2: 测试② 电机正反转+测速 (test/test_motor.c)
 *   3: 测试③ 蓝牙调参链路 (test/test_bluetooth.c)
 * 测试模式的说明与预期现象表见 硬件功能测试方案.md。
 */
#define HW_TEST_SELECT 3

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
    /* 首先初始化调试串口USART1，保证printf可用 */
    uart_init(115200);

    u32 t                 = 0;
    uint16_t ceshi_cnt    = 0;
    uint16_t break_flag   = 0;     /* 数据异常标志 */
    uint16_t danbian_flag = 0;     /* 单边标志 (用于S弯检测) */
    float jiaodu_piancha;
    float servo_pwm;
    float servo_midpwm = 156.5;    /* 舵机中位PWM/10，实际=1565 (约1.5ms) */
    float qulu_forward;
    float qulu_jinduan;
    uint16_t pid_select           = 0;  /* 当前PID模式 */
    uint16_t pid_select_last      = 0;  /* 上一帧PID模式 */
    uint16_t pid_select_last_last = 0;  /* 上上帧PID模式 */
    uint16_t tubian               = 0;

    /* ===== 系统初始化 ===== */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  /* 2位抢占，2位子优先级 */
    delay_init(84);                                  /* 延时函数初始化 */

    /* 注意初始化顺序: USART2先初始化(PA2=USART2 AF),
     * PWM_Init_leida后初始化(PA2=TIM9 CH1 AF, 覆盖USART2_TX)。
     * 雷达用PA3(RX)+DMA接收数据, USART2_TX不需要 */
    uart2_init(230400);                               /* 雷达串口USART2初始化 (先于TIM9) */
    DMA_Initializes();                                /* DMA初始化 (USART2雷达接收) */
    Bluetooth_Init();                                 /* 蓝牙初始化 (USART6, PC6/PC7) */
    PWM_Init_leida();                                 /* 雷达电机PWM初始化 (TIM9 CH1, PA2, 覆盖USART2_TX) */
    PWM_SetCompare_leida(97);                         /* 雷达电机初始占空比 */

    Servo_Init(84, 20000, servo_midpwm);              /* 舵机初始化 (50Hz) */

    /* 舵机PID初始化: kp, kp_2, kp_3, kd, kd_2, kd_3
     * 参数含义: kp/kd=直道, kp_2/kd_2=大转弯, kp_3/kd_3=小转弯 */
    Midline_PD_Init(&Servo_pd, 0.0575, 0.14, 0.0597, 0.12, 0.02, 0.105);

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
    Speed_mubiao = 17;                   /* 目标速度 */

    /* 最终舵机PID参数 (覆盖初始值) */
    Midline_PD_Init(&Servo_pd, 0.035, 0.040, 0.0395, 0.075, 0.022, 0.020);

    /* 左右边界扫描范围 (度) — 影响HANDLE6和HANDLE7 */
    BLUE_ANGLE_LEFT_RIGHT = 90;

    /* 右侧投影距离补偿系数 (小转弯模式) */
    BLUE_DIS_RIGHT = 27;

    /* 左侧投影距离补偿系数 (小转弯模式) */
    BLUE_DIS_LEFT = 50;

    /* 小转弯模式下目标Y坐标: 越小越直 */
    BLUE_Y_RIGHT = 1200;
    BLUE_Y_LEFT  = 1350;

    /* 直道模式选择: 0=用中线末点, 1=用固定BLUE_Y_STRA */
    BLUE_Y_STRA_SEL = 0;

    /* 断点判断距离阈值 */
    duandian_DIStance = 550;

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

            /* ===== 第1步: 解析雷达数据 ===== */
            LEIDA_DATA_HANDLE1(LEIDA_DATA, DMA_USART2_RX_BUF_r, DMA_USART2_RX_BUF_LEN);

            /* ===== 第2步: 筛选有效点 (距离>=100mm) ===== */
            valid_couter = LEIDA_DATA_HANDLE3_2(LEIDA_DATA2, LEIDA_DATA, LEIDA_DATA_COUNTER);

            /* 有效点太少 -> 数据异常，跳过此帧 */
            /* 注意: 不能写break — 这里最内层循环就是while(1)主循环,
             * break会直接跳出主循环, 整个控制程序当场死掉(只剩TIM5速度中断还在跑)。
             * 正确做法是continue: 跳过本帧剩余处理, 等下一帧雷达数据 */
            if (valid_couter <= 20) {
                break_flag = 1;
                printf("Data invalid, skip frame: %d\r\n", valid_couter);  /* 调试打印, 稳定后可删 */
                continue;
            }
            /* HANDLE3_2内部已掐头去尾各10个点，此处不再重复减去 */

            /* ===== 第3步: 计算跑道宽度 ===== */
            /* 雷达测距，600-900mm之间才更新 (防止异常值) */
            paodao_distance_r = LEIDA_Distance(LEIDA_DATA2, valid_couter);
            paodao_distance   = ((paodao_distance_r > 600) && (paodao_distance_r < 900))
                                ? paodao_distance_r : paodao_distance;

            /* ===== 第4步: 提取左右边界点 ===== */
            LEFT_cnt  = LEIDA_DATA_HANDLE6(LEIDA_DATA_LEFT, LEIDA_DATA2, valid_couter);
            RIGHT_cnt = LEIDA_DATA_HANDLE7(LEIDA_DATA_RIGHT, LEIDA_DATA2, valid_couter);

            /* ===== 第5步: 前方路径扫描 ===== */
            /* 前方70°-110°扫描，无障碍物则Forward_cnt=0 */
            Forward_cnt = LEIDA_DATA_HANDLE5(LEIDA_DATA_Forward, LEIDA_DATA2, valid_couter);

            /* 右侧(70°-90°)和左侧(90°-110°)分别扫描，用于S弯检测 */
            Forward_cnt_2 = LEIDA_DATA_HANDLE5_2(LEIDA_DATA_Forward_2, LEIDA_DATA2, valid_couter, 70, 90);/*右边*/
            Forward_cnt_3 = LEIDA_DATA_HANDLE5_2(LEIDA_DATA_Forward_3, LEIDA_DATA2, valid_couter, 90, 110);/*左边*/

            /* ===== 第6步: 前方路径直线拟合 ===== */
            /*前方被堵的前提下*/
            if (Forward_cnt) {
                /* 使用前10%-90%的点拟合，去除两端离群点 */
                Midline_fit(LEIDA_DATA_Forward,
                            (uint16_t)(Forward_cnt * 1.0f / 20 * 2),
                            (uint16_t)(Forward_cnt * 1.0f / 20 * 18),
                            &Midline_forward);

                danbian_flag = 0;
                /* 两侧前方都有5个以上有效点 -> S弯特征 */
                if ((Forward_cnt_2 > 5) && (Forward_cnt_3 > 5)) {
                    Midline_fit(LEIDA_DATA_Forward_2, 1, (uint16_t)(Forward_cnt_2 - 1), &Midline_forward_2);
                    Midline_fit(LEIDA_DATA_Forward_3, 1, (uint16_t)(Forward_cnt_3 - 1), &Midline_forward_3);
                    danbian_flag = 1;
                }
            }

            /* ===== 第7步: 检测左右边界突变点 (弯道入口) ===== */
            RIGHT_duandian = LEIDA_DATA_HANDLE9(LEIDA_DATA_RIGHT, RIGHT_cnt);
            LEFT_duandian  = LEIDA_DATA_HANDLE8(LEIDA_DATA_LEFT, LEFT_cnt);

            /* 如果左右两边同时检测到断点(都在100-550mm之间)，
             * 保留较近的一个，丢弃较远的那个（防止T字路口误判） */
            if ((LEFT_duandian > 100) && (LEFT_duandian < duandian_DIStance)
                && (RIGHT_duandian > 100) && (RIGHT_duandian < duandian_DIStance)) {
                if (LEFT_duandian > RIGHT_duandian)
                    LEFT_duandian = 0;
                else
                    RIGHT_duandian = 0;
            }

            /* ===== 第8步: PID模式历史记录 ===== */
            pid_select_last_last = pid_select_last;
            pid_select_last      = pid_select;

            /* ================================================================
             * 第9步: 右边界断点处理 — 需要右转
             * ================================================================ */
            if ((RIGHT_duandian > 0) && (RIGHT_duandian < duandian_DIStance)) {

                /* ----- 情况A: 前方斜率平缓 (<0.35) -----
                 * 前方无障碍，可以大幅转弯 */
                if ((fabs(Midline_forward.k) < 0.35) && (Forward_cnt)
                    && (RIGHT_duandian < duandian_distance)) {

                    /* 斜率强制取正（右转方向） */
                    if (Midline_forward.k < 0) Midline_forward.k = -Midline_forward.k;
                    printf("k: %f,b: %f\r\n", Midline_forward.k, Midline_forward.b);

                    /* 模式3: 大角度右转，使用前方拟合数据 */
                    pid_select = 3;
                    servo_pwm  = Midline_PD(LEIDA_DATA_Forward, &Servo_pd, &Midline_forward,
                                            servo_midpwm,
                                            (uint16_t)(Forward_cnt / 20.0 * 2),
                                            (uint16_t)(Forward_cnt / 20.0 * 18),
                                            pid_select);
                    state_left_cnt_2 = 0;

                /* ----- 情况B: 前方斜率中等 (0.35~0.7) + S弯特征 ----- */
                } else if ((fabs(Midline_forward.k) >= 0.35) && (fabs(Midline_forward.k) < 0.7)
                           && (Forward_cnt) && (RIGHT_duandian < duandian_distance)
                           && ((fabs(Midline_forward_2.k) < 0.25) || (fabs(Midline_forward_3.k) < 0.25))
                           && (danbian_flag == 1)) {

                    if (Midline_forward.k < 0) Midline_forward.k = -Midline_forward.k;
                    printf("k: %f,b: %f\r\n", Midline_forward.k, Midline_forward.b);

                    /* 模式8: 中等角度右转 (S弯中段) */
                    pid_select = 8;
                    servo_pwm  = Midline_PD(LEIDA_DATA_Forward, &Servo_pd, &Midline_forward,
                                            servo_midpwm,
                                            (uint16_t)(Forward_cnt / 20.0 * 2),
                                            (uint16_t)(Forward_cnt / 20.0 * 18),
                                            pid_select);

                /* ----- 情况C: 前方斜率陡峭 (>=0.7) ----- */
                } else {
                    /* 如果前两帧都是大右转，强制再来一帧小右转 */
                    if ((pid_select_last == 3) && (pid_select_last_last == 3)) {
                        state_left_cnt_2 = 1;
                    }
                    if (state_left_cnt_2 > 0) {
                        /* 强制小角度右转 */
                        pid_select = 1;
                        state_left_cnt_2 -= 1;
                    } else {
                        /* 没有前方数据可用 -> 沿左边界走小角度右转 */
                        LEIDA_DATA_HANDLE2(LEIDA_DATA_LEFT_Plane, LEIDA_DATA_LEFT, LEFT_cnt);
                        LEFT_cnt = LEIDA_DATA_HANDLE10(LEIDA_DATA_LEFT_Plane, LEFT_cnt);
                        /* 使用左边界后75%-95%的点拟合 */
                        Midline_fit(LEIDA_DATA_LEFT_Plane,
                                    (uint16_t)(LEFT_cnt / 20.0 * 15),
                                    (uint16_t)(LEFT_cnt / 20.0 * 19),
                                    &Midline);
                        pid_select = 1;  /* 小角度右转 */
                        servo_pwm  = Midline_PD(LEIDA_DATA_LEFT_Plane, &Servo_pd, &Midline,
                                                servo_midpwm,
                                                (uint16_t)(LEFT_cnt / 20.0 * 15),
                                                (uint16_t)(LEFT_cnt / 20.0 * 19),
                                                pid_select);
                    }
                }
            }

            /* ================================================================
             * 第10步: 左边界断点处理 — 需要左转
             * ================================================================ */
            else if ((LEFT_duandian > 0) && (LEFT_duandian < duandian_DIStance)) {

                /* ----- 前方斜率平缓 -> 大角度左转 ----- */
                if ((fabs(Midline_forward.k) < 0.35) && (Forward_cnt)
                    && (LEFT_duandian < duandian_distance)) {

                    printf("k: %f,b: %f\r\n", Midline_forward.k, Midline_forward.b);
                    if (Midline_forward.k > 0) Midline_forward.k = -Midline_forward.k;
                    /* 斜率强制取负（左转方向） */
                    pid_select = 4;  /* 大角度左转 */
                    servo_pwm  = Midline_PD(LEIDA_DATA_Forward, &Servo_pd, &Midline_forward,
                                            servo_midpwm,
                                            (uint16_t)(Forward_cnt / 20.0 * 2),
                                            (uint16_t)(Forward_cnt / 20.0 * 18),
                                            pid_select);
                    state_right_cnt_2 = 0;

                /* ----- 前方斜率中等 + S弯特征 -> 中等角度左转 ----- */
                } else if ((fabs(Midline_forward.k) >= 0.35) && (fabs(Midline_forward.k) < 0.7)
                           && (Forward_cnt) && (LEFT_duandian < duandian_distance)
                           && ((fabs(Midline_forward_2.k) < 0.25) || (fabs(Midline_forward_3.k) < 0.25))
                           && (danbian_flag == 1)) {

                    printf("k: %f,b: %f\r\n", Midline_forward.k, Midline_forward.b);
                    if (Midline_forward.k > 0) Midline_forward.k = -Midline_forward.k;
                    pid_select = 9;  /* 中等角度左转 */
                    servo_pwm  = Midline_PD(LEIDA_DATA_Forward, &Servo_pd, &Midline_forward,
                                            servo_midpwm,
                                            (uint16_t)(Forward_cnt / 20.0 * 2),
                                            (uint16_t)(Forward_cnt / 20.0 * 18),
                                            pid_select);

                /* ----- 前方斜率陡峭 -> 小角度左转 ----- */
                } else {
                    if ((pid_select_last == 4) && (pid_select_last_last == 4))
                        state_right_cnt_2 = 1;

                    if (state_right_cnt_2 > 0) {
                        /* 强制小角度左转 */
                        pid_select = 2;
                        state_right_cnt_2 -= 1;
                    } else {
                        /* 沿右边界走小角度左转 */
                        LEIDA_DATA_HANDLE2(LEIDA_DATA_RIGHT_Plane, LEIDA_DATA_RIGHT, RIGHT_cnt);
                        RIGHT_cnt = LEIDA_DATA_HANDLE10(LEIDA_DATA_RIGHT_Plane, RIGHT_cnt);
                        Midline_fit(LEIDA_DATA_RIGHT_Plane,
                                    (uint16_t)RIGHT_cnt / 20.0 * 15,
                                    (uint16_t)RIGHT_cnt / 20.0 * 19,
                                    &Midline);
                        pid_select = 2;  /* 小角度左转 */
                        servo_pwm  = Midline_PD(LEIDA_DATA_RIGHT_Plane, &Servo_pd, &Midline,
                                                servo_midpwm,
                                                (uint16_t)(RIGHT_cnt / 20.0 * 15),
                                                (uint16_t)(RIGHT_cnt / 20.0 * 19),
                                                pid_select);
                    }
                }

            /* ================================================================
             * 第11步: 无断点 — 直道模式
             * ================================================================ */
            } else {

                /* 计算中线点 */
                CENTER_cnt = LEIDA_DATA_HANDLE4(LEIDA_DATA_CENTER, LEIDA_DATA2, valid_couter);

                /* 如果前两帧都是大转弯，强制在之后的直道中再补一帧同向转弯 */
                if ((pid_select_last == 3) && (pid_select_last_last == 3)) state_left_cnt = 2;
                if ((pid_select_last == 4) && (pid_select_last_last == 4)) state_right_cnt = 2;

                pid_select = 0;

                /* 执行强制大角度右转 (残留) */
                if (state_left_cnt > 0) {
                    Servo_ChangePwm(1360);  /* 舵机打满右 */
                    state_left_cnt--;
                }

                /* 执行强制大角度左转 (残留) */
                if (state_right_cnt > 0) {
                    Servo_ChangePwm(1800);  /* 舵机打满左 */
                    state_right_cnt--;
                }

                /* 正常直道循线 */
                if ((state_left_cnt == 0) && (state_right_cnt == 0)) {

                    /* 判断中线是否垂直（直道） */
                    zhongxian_chuizhi = LEIDA_DATA_HANDLE11(LEIDA_DATA_CENTER,
                                        (uint16_t)(CENTER_cnt / 20.0 * 8),
                                        (uint16_t)(CENTER_cnt / 20.0 * 18));

                    if (zhongxian_chuizhi == 0) {
                        /* 中线不垂直 -> 普通中线拟合 */
                        Midline_fit(LEIDA_DATA_CENTER,
                                    (uint16_t)(CENTER_cnt / 20.0 * 8),
                                    (uint16_t)(CENTER_cnt / 20.0 * 18),
                                    &Midline);

                        if (fabs(Midline.k) <= 0.1) {
                            /* 斜率很小 -> 直接使用当前舵机位置 */
                            Servo_ChangePwm((uint16_t)servo_pwm);
                        } else if (CENTER_cnt < 8) {
                            /* 中线点太少 -> 使用模式7 (中线均值)*/
                            servo_pwm = Midline_PD(LEIDA_DATA_CENTER, &Servo_pd, &Midline,
                                                   servo_midpwm,
                                                   (uint16_t)(CENTER_cnt / 20.0 * 8),
                                                   (uint16_t)(CENTER_cnt / 20.0 * 18), 7);
                            ceshi_cnt++;
                        } else {
                            /* 使用默认中线模式(pid_select=0) */
                            servo_pwm = Midline_PD(LEIDA_DATA_CENTER, &Servo_pd, &Midline,
                                                   servo_midpwm,
                                                   (uint16_t)(CENTER_cnt / 20.0 * 8),
                                                   (uint16_t)(CENTER_cnt / 20.0 * 18),
                                                   pid_select);
                        }
                    } else {
                        /* 中线垂直 -> 垂线直道模式(pid=5) */
                        servo_pwm = Midline_PD(LEIDA_DATA_CENTER, &Servo_pd, &Midline,
                                               servo_midpwm,
                                               (uint16_t)(CENTER_cnt / 20.0 * 8),
                                               (uint16_t)(CENTER_cnt / 20.0 * 18), 5);
                    }
                }
            }

            /* 本帧雷达处理完: 回传7通道波形给手机/VOFA+ (未连接时内部直接返回, 零开销) */
            BLE_Tune_Telemetry(Servo_pd.err, servo_pwm, pid_select);

        } else {
            /* DMA数据未就绪，等待 */
            ;
        }

        if (break_flag == 1) {
            break_flag = 0;
        }

#endif
    }
}
