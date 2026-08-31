/**
 * @file    ble_tune.c
 * @brief   蓝牙实时调参模块 (串口助手/VOFA+ → HC-05 → USART6)
 *
 * 两条数据流:
 *   【调参】PC→车: 文本命令 "名字 整数\n" (名字后跟空格或冒号, 如 "kp 45"/"kp:45")。
 *          整数按 scale 缩放为浮点写入参数 (避开 MicroLIB 不支持 sscanf %f 的坑,
 *          用 atoi 解析)。命令以 \n 结尾, 解析层按 \n 分帧 (蓝牙空中分块交货,
 *          IDLE 分帧不可靠, 见 蓝牙实时调参方案.md §3.3)。
 *   【遥测】车→PC: float32小端 × 7通道 + 帧尾 00 00 80 7F = 32字节
 *          (VOFA+ FireWater 二进制引擎自动识别通道, 见 §3.6)。
 *
 * 实时生效原理: 控制循环每拍现读 Servo_pd/Speed_pid/BLUE_* 字段,
 * 所以本模块解析后直接赋值字段即可下一拍生效。
 * 绝不调用 Midline_PD_Init / Speed_PID_Init 改参数 —— 它们会清零 err/err_l,
 * 车在弯里时 D 项突降会猛抖一下。
 *
 * 使用说明见 蓝牙调参说明书.md。
 */

#include "ble_tune.h"
#include "bsp_bluetooth.h"      /* BLERX_BUFF/BLERX_FLAG/BLERX_LEN, 蓝牙收发函数 */
#include "centre_line.h"        /* Servo_pd, Speed_pid, BLUE_DIS_*, BLUE_Y_* */
#include "LEIDA_DATA.h"         /* BLUE_ANGLE_LEFT_RIGHT */
#include "moto.h"               /* Speed_now, Speed_mubiao */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"             /* atoi (MicroLIB支持, 但sscanf %f不支持) */

/* ======================== 【调参】参数表 ========================
 * scale 含义: 发送值 / 10^scale = 实际写入的浮点值
 *   例: {"kp", ..., 3} → 手机发 "kp 45" 实际写入 45/1000 = 0.045
 * min_i/max_i 是"发送值"的合法范围, 超范围回 ERR range 且不写值 */
typedef struct {
    const char *name;   /* 命令名 */
    float      *ptr;    /* 被调参数指针 */
    int16_t     min_i;  /* 发送值下限 */
    int16_t     max_i;  /* 发送值上限 */
    uint8_t     scale;  /* 缩放: 发送值/10^scale = 实际值 */
} Param_t;

static const Param_t param_tab[] = {
    /* 舵机PD: 千分位 (scale=3) */
    {"kp",    &Servo_pd.kp,           0, 1000, 3},   /* 直道PD   发35 → 0.035 */
    {"kp2",   &Servo_pd.kp_2,         0, 1000, 3},   /* 大转弯PD 发40 → 0.040 */
    {"kp3",   &Servo_pd.kp_3,         0, 1000, 3},   /* 小转弯PD 发40 → 0.040 */
    {"kd",    &Servo_pd.kd,           0, 1000, 3},   /* 直道D    发75 → 0.075 */
    {"kd2",   &Servo_pd.kd_2,         0, 1000, 3},   /* 大转弯D  发22 → 0.022 */
    {"kd3",   &Servo_pd.kd_3,         0, 1000, 3},   /* 小转弯D  发20 → 0.020 */
    /* 电机PI: kp×100, ki×1000 */
    {"sp_kp", &Speed_pid.kp,          0, 5000, 2},   /* 发850 → 8.5 */
    {"sp_ki", &Speed_pid.ki,          0, 5000, 3},   /* 发505 → 0.505 */
    {"spd",   &Speed_mubiao,          0,   50, 0},   /* 目标速度 1:1 */
    /* 几何补偿参数: 1:1 (scale=0) */
    {"disr",  &BLUE_DIS_RIGHT,        0,  200, 0},   /* 右侧宽度补偿系数(小转弯) */
    {"disl",  &BLUE_DIS_LEFT,         0,  200, 0},   /* 左侧宽度补偿系数(小转弯) */
    {"yr",    &BLUE_Y_RIGHT,          0, 2000, 0},   /* 右转目标Y坐标 */
    {"yl",    &BLUE_Y_LEFT,           0, 2000, 0},   /* 左转目标Y坐标 */
    {"ystra", &BLUE_Y_STRA,           0, 2000, 0},   /* 直道模式固定目标Y坐标 */
    {"ysel",  &BLUE_Y_STRA_SEL,       0,    1, 0},   /* 直道模式选择: 0=中线末点, 1=固定Y_STRA */
    {"angle", &BLUE_ANGLE_LEFT_RIGHT, 0,  180, 0},   /* 左右边界扫描范围(度) */
};
#define PARAM_NUM (sizeof(param_tab) / sizeof(param_tab[0]))

/* ======================== 【遥测】二进制发送底层 ======================== */
typedef union {
    float   f;
    uint8_t b[4];
} FloatByte_t;

/* 发一个 float32: 小端低字节在前 (Cortex-M4 天然小端, 无需转换) */
static void Tele_SendFloat(float v)
{
    FloatByte_t u;
    u.f = v;
    BLE_Send_Bit(u.b[0]);
    BLE_Send_Bit(u.b[1]);
    BLE_Send_Bit(u.b[2]);
    BLE_Send_Bit(u.b[3]);
}

/* 帧尾: float 正无穷(0x7F800000)的小端表示, VOFA+ 靠它识别通道数 */
static void Tele_SendTail(void)
{
    BLE_Send_Bit(0x00);
    BLE_Send_Bit(0x00);
    BLE_Send_Bit(0x80);
    BLE_Send_Bit(0x7F);
}

/* 每雷达帧调用一次: 7通道 = 误差/舵机PWM/kp/kd/pid模式/当前速度/目标速度
 * 帧长 7×4+4=32字节 ≈ 33ms @9600; 雷达帧周期约78ms, 带宽安全。
 * 未连接时直接返回: 零发送零阻塞, 车跑着不连蓝牙和原来完全一样 */
void BLE_Tune_Telemetry(float err, float servo_pwm, uint16_t pid_mode)
{
    Bluetooth_Mode();                               /* 刷新STATE连接状态 */
    if (Get_Bluetooth_ConnectFlag() == 0) return;   /* 未连接不发 */

    Tele_SendFloat(err);
    Tele_SendFloat(servo_pwm);
    Tele_SendFloat(Servo_pd.kp);
    Tele_SendFloat(Servo_pd.kd);
    Tele_SendFloat((float)pid_mode);
    Tele_SendFloat(Speed_now);
    Tele_SendFloat(Speed_mubiao);
    Tele_SendTail();
}

/* ======================== 【调参】get 命令回显 ======================== */
/* 回发全部参数 "名字=整数\r\n", 整数按 scale 四舍五入 (如 kp → "kp=35")。
 * 注意: 16行 ≈ 230字节 @9600 ≈ 240ms, 主循环会停控约0.24秒, 建议停车时用 */
static void Tune_SendAll(void)
{
    static const int mul[4] = {1, 10, 100, 1000};
    char line[40];
    uint8_t i;

    for (i = 0; i < PARAM_NUM; i++) {
        int iv = (int)(*param_tab[i].ptr * mul[param_tab[i].scale] + 0.5f);
        sprintf(line, "%s=%d\r\n", param_tab[i].name, iv);
        Send_Bluetooth_Data(line);
    }
}

/* ======================== 【调参】单行命令解析 ======================== */
/* 例: "kp 45" / "kp:45" / "kp :45" / "kp:  45" 都能解析出 kp=45。
 * 防护1: 分隔符空格/冒号都认, 取更靠前的一个作切点 (兼容VOFA+滑块发"kp:45")
 * 防护2: 切断后跳过":"等非数字前缀再 atoi; 全是非数字 → 拒绝
 *        (教训: atoi(":493")会返回0, 而0在合法范围内不报错, 曾把参数静默清零)
 * 防护3: 范围检查, 超范围 ERR range 钳位拒绝, 不写值 */
static void Tune_ApplyOne(char *line)
{
    static const int mul[4] = {1, 10, 100, 1000};
    char *sp;
    char *sc;
    char *val;
    char ack[40];
    int iv;
    uint8_t i;

    if (strcmp(line, "get") == 0) { Tune_SendAll(); return; }  /* 精确匹配, 防"getxx"误触发 */

    sp = strchr(line, ' ');
    sc = strchr(line, ':');
    if ((sp == NULL) || ((sc != NULL) && (sc < sp))) sp = sc;  /* 取更靠前的作切点 */
    if (sp == NULL) { Send_Bluetooth_Data("ERR fmt\r\n"); return; }
    *sp = '\0';                     /* 切断 → line="kp", sp+1="45..." */
    val = sp + 1;
    while (*val && !((*val >= '0' && *val <= '9') || *val == '-')) val++;  /* 跳过":"等非数字前缀 */
    if (*val == '\0') { Send_Bluetooth_Data("ERR fmt\r\n"); return; }
    iv = atoi(val);                 /* 整数解析, 避开 MicroLIB 不支持 sscanf %f 的坑 */

    for (i = 0; i < PARAM_NUM; i++) {
        if (strcmp(line, param_tab[i].name) != 0) continue;

        if ((iv < param_tab[i].min_i) || (iv > param_tab[i].max_i)) {
            Send_Bluetooth_Data("ERR range\r\n");   /* 超范围: 拒绝, 不写值 */
            return;
        }
        /* 直接赋值字段, 下一拍控制循环现读新值即生效。
         * 绝不调用 Midline_PD_Init/Speed_PID_Init —— 它们会清零误差状态 */
        *param_tab[i].ptr = (float)iv / mul[param_tab[i].scale];
        sprintf(ack, "OK %s=%d\r\n", line, iv);
        Send_Bluetooth_Data(ack);
        return;
    }
    Send_Bluetooth_Data("ERR name\r\n");
}

/* ======================== 【调参】主入口 ======================== */
/* 主循环每圈调用。流程:
 *   BLERX_FLAG==0 → 直接返回 (USART6 IDLE 中断置位后才处理)
 *   临界区整体拷出 BLERX_BUFF → 按 \n 分行逐条执行 → 半行尾巴留到下一帧拼 */
void BLE_Tune_Process(void)
{
    static char tail[BLERX_LEN_MAX];    /* 跨IDLE保留的半行尾巴:
                                           蓝牙空中分块交货, 一条命令可能被 IDLE
                                           切成两帧, 没见到\n的半行不能丢 */
    static uint8_t tail_len = 0;
    char *start, *end;
    uint8_t len, copy_n;

    if (BLERX_FLAG == 0) return;

    /* 关中断整体拷出: 防止解析期间 RX 中断又往里写。
     * Clear 也放进临界区, 避免开中断后新到的字节被误清 */
    __disable_irq();
    len    = BLERX_LEN;
    copy_n = (len > BLERX_LEN_MAX - 1 - tail_len) ? (uint8_t)(BLERX_LEN_MAX - 1 - tail_len) : len;
    memcpy(tail + tail_len, BLERX_BUFF, copy_n);
    tail_len += copy_n;
    Clear_BLERX_BUFF();
    __enable_irq();

    tail[tail_len] = '\0';

    /* 按 \n 分行: 支持一次发多条命令 */
    start = tail;
    while ((end = strchr(start, '\n')) != NULL) {
        *end = '\0';
        if (end > start && end[-1] == '\r') end[-1] = '\0';  /* 兼容串口助手"发送新行"带的CR */
        if (end != start) Tune_ApplyOne(start);              /* 空行跳过 */
        start = end + 1;
    }
    /* 没见到\n的半行留到下一帧继续拼 (与IDLE何时切帧无关) */
    tail_len = (uint8_t)strlen(start);
    memmove(tail, start, tail_len + 1);
}
