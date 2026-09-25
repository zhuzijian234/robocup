#include "ble_diag.h"
#define CONTROL_TRACE(...) ((void)0)
/**
 * @file    main.c
 * @brief   雷达小车主程序 — 初始化、主控制循环、多模式转向决策
 *
 * ======================== 系统架构 ========================
 *
 * 硬件平台: STM32F407ZGT6 @ 168MHz
 * 传感器:   M10系列激光雷达 (USART2, 512000bps, DMA接收)
 * 执行器:   舵机 (TIM3 CH1), 直流电机 (TIM2 CH1, 编码器 TIM4)
 * 通信:     HC-05蓝牙 (USART6)
 *
 * 引脚复用 (对应谢露版):
 *   雷达:     USART2, PA2(TX) PA3(RX/DMA), DMA1 Stream5 Channel4
 *   蓝牙:     USART6, PC6(TX) PC7(RX)
 *   舵机:     TIM3 CH1, PA6
 *   电机PWM:  TIM2 CH1, PA5  [2026-09-06 PB11杜邦线故障, 临时挪至PA5]
 *   电机方向: PB15 (单IO, 高=正转)  [2026-09-06 PB10杜邦线故障, 临时挪至PB15]
 *   编码器:   TIM4, PD12 PD13
 *   雷达电机: M10P内部驱动；本工程不初始化TIM9雷达PWM
 *   Debug:    USART1, PA9(TX) PA10(RX)
 *
 * 主循环流程:
 *   1. 循环DMA收流，M10P按车后切点发布完整扫描
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
/* M10P内部驱动，无需旧LD14P的leida_pwm.h及TIM9初始化。 */
#include "LEIDA_DATA.h"
#include "bsp_bluetooth.h"
#include "ble_tune.h" /* 蓝牙实时调参 (HARDWARE/hc-05/ble_tune.c) */
#include "centre_line.h"
#include "timer.h"
#include "moto.h"
#include "Servo.h"
#include "PWM.h"
#include "test.h"
#include "m10p_vehicle.h"

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
uint16_t RIGHT_duandian;       /* 右边界断点y坐标 */
uint16_t LEFT_duandian;        /* 左边界断点y坐标 */
float paodao_distance = 700;   /* 跑道宽度 (mm) */
float paodao_distance_r = 700; /* 当前帧跑道宽度 (mm) */
#if 0 /* 历史调试变量：无读写调用，保留名称供对照旧版本。 */
float paodao_distance_r_r = 0;
float paodao_distance_r_r_r = 0;
#endif
/* 兼容既有遥测字段；旧的固定帧数强制补打已由TurnGuard替代。 */
uint16_t state_left_cnt = 0;
uint16_t state_right_cnt = 0;
uint16_t state_left_cnt_2 = 0;
uint16_t state_right_cnt_2 = 0;

extern float Speed_now;

#define duandian_distance 600 /* 断点判断距离阈值 */
#if 0 /* 旧控制状态，已由TurnGuard维护 */
uint16_t state_sta = 1;
#endif

float duandian_DIStance = 600; /* 断点有效距离阈值 (mm) */

int main(void)
{
    /* 初始化调试串口: USART1(PA9/PA10, 保留备用) + USART3(PC10/PC11, printf已重定向) */
    uart_init(115200);
    uart3_init(115200); /* 调试串口2 (PC10=TX, PC11=RX) */

    uint16_t parsed_points = 0;
    const M10P_Scan *scan;
    uint8_t forward_fit_ok, side_fit_ok;
    uint16_t ref_start, ref_end, candidate_pwm;
    uint8_t turn_held, straight_evidence;
    static TurnGuard turn_guard;
    uint16_t break_flag = 0;   /* 数据异常标志 */
    uint16_t danbian_flag = 0; /* 单边标志 (用于S弯检测) */
    uint16_t telemetry_mode = BLE_MODE_HOLD;
    float servo_midpwm = SERVO_PWM_MID / 10.0f; /* 舵机中位PWM/10 (行程参数见centre_line.h) */
    /* 候选模式为0~9；输出被弯道状态延续时，遥测单独报告HOLD。 */
    uint16_t pid_select = 0;

    /* ===== 系统初始化 ===== */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); /* 2位抢占，2位子优先级 */
    Diag_Init();
    delay_init(168); /* 延时函数初始化 (参数=主频MHz, 本工程168MHz) */

    /* M10P: PA2 stays USART2_TX; no LD14P TIM9 PWM. */
    M10P_Init();
    uart2_init(M10P_BAUD);
    DMA_Initializes();
    Bluetooth_Init();

    Servo_Init(84, 20000, SERVO_PWM_MID); /* 舵机初始化 (50Hz) */

    /* 舵机PID初始化: kp, kp_2, kp_3, kd, kd_2, kd_3
     * 参数含义: kp/kd=直道, kp_2/kd_2=大转弯, kp_3/kd_3=小转弯 */
    Midline_PD_Init(&Servo_pd, 0.0575, 0.14, 0.0597, 0.12, 0.02, 0.105);

    /* 速度PID初始化: kp, ki, kd */
    Speed_PID_Init(&Speed_pid, 8.5, 0.505, 0);

    /* 电机初始化: 预分频42, 周期100 */
    moto_pwm = 0;
#if HW_TEST_SELECT == 0
    Moto_Init(42, 100, moto_pwm); /* 等首个有效输入处理完成后才允许速度环驱动 */
#else
    Moto_Init(42, 100, 0); /* 测试模式: 初始0%, 防止上电轮子就转 */
#endif

    Encoder_Init(); /* 编码器初始化 (TIM4) */

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
    Speed_mubiao = 10; /* 目标速度 */

    /* 最终舵机PID参数 (覆盖初始值) */
    Midline_PD_Init(&Servo_pd, 0.035, 0.040, 0.0395, 0.035, 0.022, 0.020);

    /* 左右边界扫描范围 (度) — 影响HANDLE6和HANDLE7 */
    BLUE_ANGLE_LEFT_RIGHT = 90;

    /* 右侧投影距离补偿系数 (小转弯模式) */
    BLUE_DIS_RIGHT = 50; /* symmetric half-width reference; prevents entry bias */

    /* 左侧投影距离补偿系数 (小转弯模式) */
    BLUE_DIS_LEFT = 50;

    /* 小转弯模式下目标Y坐标: 越小越直 */
    BLUE_Y_RIGHT = 1200;
    BLUE_Y_LEFT = 1350;

    /* 直道模式选择: 0=用中线末点, 1=用固定BLUE_Y_STRA */
    BLUE_Y_STRA_SEL = 0;

    /* 断点判断距离阈值 */
    duandian_DIStance = 550;

    CONTROL_TRACE("Start\r\n");

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

        Diag_Poll();
        BLE_Tune_Process(); /* 蓝牙命令解析: 每圈必跑(含雷达帧等待圈), 命令响应<1ms */

        M10P_Poll();
        scan = M10P_Acquire();
        if (scan) {
            /* Diagnostic timestamps refer to reception, never parse time. */
            Diag_input_seq = scan->seq;
            Diag_input_us = scan->end_us;
            Diag_input_ms = Diag_TimeMs() - (uint32_t)(Diag_TimeUs() - scan->end_us) / 1000u;
            Diag_Begin(Diag_input_ms, scan->end_us);
            Diag_detail_u[3] = scan->seq;
            telemetry_mode = BLE_MODE_INVALID;
            Servo_PD_valid = 0;
            forward_fit_ok = side_fit_ok = 0;
            danbian_flag = 0;
            parsed_points = scan->count;
            LEIDA_raw_count = scan->count;
            LEIDA_speed_dps = scan->dps;
            valid_couter = M10P_Build(scan, LEIDA_DATA2, LEIDA_DATA_COUNTER);
            Diag_Field(12, valid_couter, 1);
            if (valid_couter <= 20 || !M10P_perception_ok) {
                Radar_invalid_inputs++;
                Radar_Invalidate();
                (void)TurnGuard_Apply(&turn_guard, BLE_MODE_INVALID, 0, 0,
                                      (uint16_t)TIM3->CCR1, Diag_TimeUs(), &turn_held);
                Midline_PD_Reset();
                Diag_Submit(BLE_MODE_INVALID, parsed_points, LEIDA_speed_dps, 0);
                M10P_Release(scan);
                continue;
            }
            /* ===== 第3步: 计算跑道宽度 ===== */
            /* 雷达测距，600-900mm之间才更新 (防止异常值) */
            paodao_distance_r = LEIDA_Distance(LEIDA_DATA2, valid_couter);
            paodao_distance = ((paodao_distance_r > 600) && (paodao_distance_r < 900))
                                  ? paodao_distance_r
                                  : paodao_distance;

            /* ===== 第4步: 提取左右边界点 ===== */
            LEFT_cnt = LEIDA_DATA_HANDLE6(LEIDA_DATA_LEFT, LEIDA_DATA2, valid_couter);
            RIGHT_cnt = LEIDA_DATA_HANDLE7(LEIDA_DATA_RIGHT, LEIDA_DATA2, valid_couter);

            /* ===== 第5步: 前方路径扫描 ===== */
            /* 前方70°-110°扫描，无障碍物则Forward_cnt=0 */
            Forward_cnt = LEIDA_DATA_HANDLE5(LEIDA_DATA_Forward, LEIDA_DATA2, valid_couter);

            /* 右侧(70°-90°)和左侧(90°-110°)分别扫描，用于S弯检测 */
            Forward_cnt_2 = LEIDA_DATA_HANDLE5_2(LEIDA_DATA_Forward_2, LEIDA_DATA2, valid_couter, 70, 90);  /*右边*/
            Forward_cnt_3 = LEIDA_DATA_HANDLE5_2(LEIDA_DATA_Forward_3, LEIDA_DATA2, valid_couter, 90, 110); /*左边*/

            /* ===== 第6步: 前方路径直线拟合 ===== */
            /*前方被堵的前提下*/
            if (Forward_cnt) {
                /* 使用前10%-90%的点拟合，去除两端离群点 */
                forward_fit_ok = Midline_fit(LEIDA_DATA_Forward,
                                             (uint16_t)(Forward_cnt * 1.0f / 20 * 2),
                                             (uint16_t)(Forward_cnt * 1.0f / 20 * 18),
                                             &Midline_forward);

                danbian_flag = 0;
                /* 两侧前方都有5个以上有效点 -> S弯特征 */
                if ((Forward_cnt_2 > 5) && (Forward_cnt_3 > 5)) {
                    side_fit_ok = Midline_fit(LEIDA_DATA_Forward_2, 1, (uint16_t)(Forward_cnt_2 - 1), &Midline_forward_2);
                    side_fit_ok &= Midline_fit(LEIDA_DATA_Forward_3, 1, (uint16_t)(Forward_cnt_3 - 1), &Midline_forward_3);
                    danbian_flag = side_fit_ok;
                }
            }

            /* ===== 第7步: 检测左右边界突变点 (弯道入口) ===== */
            RIGHT_duandian = LEIDA_DATA_HANDLE9(LEIDA_DATA_RIGHT, RIGHT_cnt);
            LEFT_duandian = LEIDA_DATA_HANDLE8(LEIDA_DATA_LEFT, LEFT_cnt);

            /* 如果左右两边同时检测到断点(都在100-550mm之间)，
             * 保留较近的一个，丢弃较远的那个（防止T字路口误判） */
            if ((LEFT_duandian > 100) && (LEFT_duandian < duandian_DIStance) && (RIGHT_duandian > 100) && (RIGHT_duandian < duandian_DIStance)) {
                if (LEFT_duandian > RIGHT_duandian)
                    LEFT_duandian = 0;
                else
                    RIGHT_duandian = 0;
            }

            /* ===== 第8步: 计算候选，仲裁弯道状态，最后只写一次舵机 ===== */
            candidate_pwm = (uint16_t)TIM3->CCR1;
            CENTER_cnt = 0;
            if ((RIGHT_duandian > 0 && RIGHT_duandian < duandian_DIStance) ||
                (LEFT_duandian > 0 && LEFT_duandian < duandian_DIStance)) {
                uint8_t right = (RIGHT_duandian > 0 && RIGHT_duandian < duandian_DIStance);
                uint16_t breakpoint = right ? RIGHT_duandian : LEFT_duandian;
                if (forward_fit_ok && breakpoint < duandian_distance && fabs(Midline_forward.k) < 0.35f) {
                    pid_select = right ? 3 : 4;
                    candidate_pwm = Midline_PD_Calculate(LEIDA_DATA_Forward, &Servo_pd, &Midline_forward, servo_midpwm,
                                                        (uint16_t)(Forward_cnt * 0.1f), (uint16_t)(Forward_cnt * 0.9f), pid_select);
                } else if (forward_fit_ok && breakpoint < duandian_distance && fabs(Midline_forward.k) < 0.7f &&
                           danbian_flag && (fabs(Midline_forward_2.k) < 0.25f || fabs(Midline_forward_3.k) < 0.25f)) {
                    pid_select = right ? 8 : 9;
                    candidate_pwm = Midline_PD_Calculate(LEIDA_DATA_Forward, &Servo_pd, &Midline_forward, servo_midpwm,
                                                        (uint16_t)(Forward_cnt * 0.1f), (uint16_t)(Forward_cnt * 0.9f), pid_select);
                } else {
                    _LEIDA_DATA_plane *boundary = right ? LEIDA_DATA_LEFT_Plane : LEIDA_DATA_RIGHT_Plane;
                    uint16_t count = right ? LEFT_cnt : RIGHT_cnt;
                    LEIDA_DATA_HANDLE2(boundary, right ? LEIDA_DATA_LEFT : LEIDA_DATA_RIGHT, count);
                    count = LEIDA_DATA_HANDLE10(boundary, count);
                    if (right)
                        LEFT_cnt = count;
                    else
                        RIGHT_cnt = count;
                    ref_start = count >= 10 ? (uint16_t)(count * 0.75f) : 0;
                    ref_end = count >= 10 ? (uint16_t)(count * 0.95f) : count;
                    (void)Midline_fit(boundary, ref_start, ref_end, &Midline);
                    pid_select = right ? 1 : 2;
                    candidate_pwm = Midline_PD_Calculate(boundary, &Servo_pd, &Midline, servo_midpwm,
                                                        ref_start, ref_end, pid_select);
                }
            } else {
                CENTER_cnt = LEIDA_DATA_HANDLE4(LEIDA_DATA_CENTER, LEIDA_DATA2, valid_couter);
                Diag_detail_u[10] = CENTER_cnt;
                Diag_detail_u[4] |= 128;
                ref_start = CENTER_cnt >= 8 ? (uint16_t)(CENTER_cnt * 0.4f) : 0;
                ref_end = CENTER_cnt >= 8 ? (uint16_t)(CENTER_cnt * 0.9f) : CENTER_cnt;
                zhongxian_chuizhi = LEIDA_DATA_HANDLE11(LEIDA_DATA_CENTER, ref_start, ref_end);
                Diag_Field(25, zhongxian_chuizhi, LEIDA_vertical_valid);
                if (LEIDA_vertical_valid)
                    pid_select = 5;
                else if (Midline_fit(LEIDA_DATA_CENTER, ref_start, ref_end, &Midline))
                    pid_select = CENTER_cnt >= 8 ? 0 : 7;
                else
                    pid_select = 7;
                /* 近水平中线由Calculate拒绝，不能把无限保舵当作有效感知。 */
                candidate_pwm = Midline_PD_Calculate(LEIDA_DATA_CENTER, &Servo_pd, &Midline, servo_midpwm,
                                                    ref_start, ref_end, pid_select);
            }
            if (scan->epoch != LidarRx_epoch || (uint32_t)(Diag_TimeUs()-scan->front_us) > M10P_MAX_AGE_US)
                Servo_PD_valid = 0;
            straight_evidence = Servo_PD_valid && CENTER_cnt >= 8 &&
                                (pid_select == 0 || pid_select == 5) && fabs(Servo_pd.err) <= TURN_EXIT_ERROR_MM;
            candidate_pwm = TurnGuard_Apply(&turn_guard, pid_select, Servo_PD_valid, straight_evidence,
                                            candidate_pwm, Diag_TimeUs(), &turn_held);
            if (!Servo_PD_valid)
                telemetry_mode = BLE_MODE_INVALID;
            else {
                telemetry_mode = turn_held ? BLE_MODE_HOLD : pid_select;
                if (turn_held) {
                    /* 候选PD未执行：下一次实际PD重新建立D历史，DETAIL不标记已执行PD。 */
                    Midline_PD_Reset();
                    Servo_PD_valid = 1;
                    Diag_detail_u[4] &= ~1u;
                }
                Servo_ChangePwm(candidate_pwm);
            }

            /* 本帧雷达处理完: 回传8通道波形给手机/VOFA+ (未连接时内部直接返回, 零开销) */
            Diag_detail_f[15] = paodao_distance_r;
            Diag_Field(9, RIGHT_duandian, 1);
            Diag_Field(10, LEFT_duandian, 1);
            if (paodao_distance_r > 600 && paodao_distance_r < 900)
                Diag_Field(11, paodao_distance, 1);
            else
                Diag_HeldField(11, paodao_distance, 1);
            Diag_Field(13, RIGHT_cnt, 1);
            Diag_Field(14, LEFT_cnt, 1);
            Diag_Field(15, Forward_cnt, 1);
            Diag_Field(16, Forward_cnt_2, 1);
            Diag_Field(17, Forward_cnt_3, 1);
            Diag_Field(20, danbian_flag, 1);
            Diag_Field(21, state_left_cnt, 1);
            Diag_Field(22, state_right_cnt, 1);
            Diag_Field(23, state_left_cnt_2, 1);
            Diag_Field(24, state_right_cnt_2, 1);
            /* 雷达看门狗: TIM5(10ms) 里 50 个 tick(500ms) 等不到这个调用, 就置
             * Radar_stop_latched 永久清零电机(只能复位)。
             * 有新鲜有效候选的有界HOLD也算有效；感知无效不能借保舵续命。 */
            if (Servo_PD_valid)
                Radar_Observe(scan->seq, scan->front_us, scan->epoch,
                              M10P_speed_scale * ((!turn_held && (pid_select == 0 || pid_select == 5 || pid_select == 7)) ? 1.0f : 0.6f));
            else {
                Radar_invalid_inputs++;
                Radar_Invalidate();
                Midline_PD_Reset();
            }
            Diag_Submit(telemetry_mode, parsed_points, LEIDA_speed_dps, Servo_PD_valid);
            BLE_Tune_Telemetry(Servo_pd.err, (float)TIM3->CCR1, telemetry_mode);
            M10P_Release(scan);

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
