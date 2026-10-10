/**
 * @file    ble_tune.c
 * @brief   蓝牙实时调参模块 (串口助手/VOFA+ → HC-05 → USART6)
 *
 * 两条数据流:
 *   【调参】PC→车: 文本命令 "名字 整数\n" (名字后跟空格或冒号, 如 "kp 45"/"kp:45")。
 *          整数按 scale 缩放为浮点写入参数 (避开 MicroLIB 不支持 sscanf %f 的坑,
 *          用 atoi 解析)。命令以 \n 结尾, 解析层按 \n 分帧 (蓝牙空中分块交货,
 *          IDLE 分帧不可靠, 见 蓝牙实时调参方案.md §3.3)。
 *   【遥测】车→PC: float32小端 × 8通道 + 帧尾 00 00 80 7F = 36字节
 *          (VOFA+ JustFloat 引擎靠帧尾自动识别通道数, 见 §3.6)。
 *
 * 实时生效原理: 控制循环每拍现读 Servo_pd/Speed_pid/BLUE_* 字段,
 * 所以本模块解析后直接赋值字段即可下一拍生效。
 * 参数成功修改后增加 Diag_revision；转向模块据此重建 D 历史，避免假微分。
 *
 * 使用说明见 蓝牙调参说明书.md。
 */

#include "ble_tune.h"
#include "ble_diag.h"
#include "m10p.h"
#include "m10p_vehicle.h"
#include "timer.h"
#include "bsp_bluetooth.h" /* BLERX_BUFF/BLERX_FLAG/BLERX_LEN, 蓝牙收发函数 */
#include "centre_line.h"   /* 当前路径参数与独立速度 PI */
#include "LEIDA_DATA.h"    /* BLUE_ANGLE_LEFT_RIGHT */
#include "moto.h"          /* Speed_now, Speed_mubiao */
#include "stdio.h"
#include "string.h"
#include "stdlib.h" /* atoi (MicroLIB支持, 但sscanf %f不支持) */

/* ======================== 【调参】参数表 ========================
 * scale 含义: 发送值 / 10^scale = 实际写入的浮点值
 *   例: {"kp", ..., 3} → 手机发 "kp 45" 实际写入 45/1000 = 0.045
 * min_i/max_i 是"发送值"的合法范围, 超范围回 ERR range 且不写值 */
typedef struct
{
    const char *name; /* 命令名 */
    float *ptr;       /* 被调参数指针 */
    int16_t min_i;    /* 发送值下限 */
    int16_t max_i;    /* 发送值上限 */
    uint16_t key;     /* 稳定协议键；删除参数不改变其他键的含义 */
    uint8_t scale;    /* 缩放: 发送值/10^scale = 实际值 */
} Param_t;

static const Param_t param_tab[] = {
    /* 命令、参数、整数范围、协议键、十进制缩放。三组增益按路径来源使用。 */
    {"kp", &Servo_pd.kp, 0, 1000, 100, 3},
    {"kp2", &Servo_pd.kp_2, 0, 1000, 101, 3},
    {"kp3", &Servo_pd.kp_3, 0, 1000, 102, 3},
    {"kd", &Servo_pd.kd, 0, 1000, 103, 3},
    {"kd2", &Servo_pd.kd_2, 0, 1000, 104, 3},
    {"kd3", &Servo_pd.kd_3, 0, 1000, 105, 3},
    {"sp_kp", &Speed_pid.kp, 0, 5000, 106, 2},
    {"sp_ki", &Speed_pid.ki, 0, 5000, 107, 3},
    {"spd", &Speed_mubiao, 0, 50, 108, 0},
    {"angle", &BLUE_ANGLE_LEFT_RIGHT, 0, 90, 115, 0}, /* 仅开口诊断 */
    {"cx", &CENTER_X_TARGET_MM, -300, 300, 116, 0},
    {"eclamp", &MODE0_ERR_CLAMP_MM, 100, 500, 117, 0},
    {"dy", &duandian_MIN_Y, 0, 550, 118, 0}, /* 仅开口诊断 */
    {"preview", &PATH_PREVIEW_MM, 600, 1100, 121, 0},
    {"width", &PATH_WIDTH_MM, 400, 900, 122, 0},
};
#define PARAM_NUM (sizeof(param_tab) / sizeof(param_tab[0]))

/* ======================== 【遥测】二进制发送底层 ======================== */
void BLE_Tune_Telemetry(float err, float pwm, uint16_t mode)
{
    uint8_t packet[36];
    float values[8];
    uint32_t qnan = 0x7fc00000;
    if (Diag_mode != 1)
        return;
    values[0] = err;
    if (mode >= 10)
        memcpy(&values[0], &qnan, 4);
    values[1] = pwm;
    values[2] = Servo_pd.kp;
    values[3] = Servo_pd.kd;
    values[4] = mode;
    values[5] = Speed_now;
    values[6] = Speed_mubiao;
    values[7] = LEIDA_speed_dps;
    memcpy(packet, values, 32);
    packet[32] = 0;
    packet[33] = 0;
    packet[34] = 0x80;
    packet[35] = 0x7f;
    BLE_Queue(packet, 36, 0);
}
uint8_t Tune_ConfigValue(uint16_t i, uint16_t *key, float *value)
{
    if (i >= PARAM_NUM)
        return 0;
    *key = param_tab[i].key;
    *value = *param_tab[i].ptr;
    return 1;
}

/* ======================== 【调参】get 命令回显 ======================== */
/* 回发全部参数 "名字=整数\r\n", 整数按 scale 四舍五入 (如 kp → "kp=35")。
 * 逐行异步入队，完整消息之间可调度CONTROL；精确备份使用getcfg */
static uint8_t get_index = PARAM_NUM;
static void Tune_SendAll(void)
{
    get_index = 0;
}
static void Tune_GetPoll(void)
{
    static const int mul[4] = {1, 10, 100, 1000};
    char line[40];
    int iv;
    float fv;
    if (get_index >= PARAM_NUM || BLE_FreeCritical() < 3)
        return;
    /* 半整数远离零取整。原实现固定 +0.5f 再截断，对负值是错的:
     * cx=-27.0 会显示成 -26(int)(-26.5f)=-26, 实机已出现。
     * 与 Diag_Encode 的舍入规则保持一致。 */
    fv = *param_tab[get_index].ptr * mul[param_tab[get_index].scale];
    iv = (int)(fv >= 0 ? fv + 0.5f : fv - 0.5f);
    sprintf(line, "%s=%d\r\n", param_tab[get_index].name, iv);
    Send_Bluetooth_Data(line);
    get_index++;
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
    char ack[64];
    int iv;
    uint8_t i;

    if (BLE_FreeCritical() < 2)
        return;
    if (Diag_Command(line))
    {
        get_index = PARAM_NUM;
        return;
    }

    /* 快照来自上一完整处理帧：pd=计算候选，apply=确实写舵机。
     * ok 只表示全扇区覆盖；局部路径有效性见 path，src=0双侧/1左/2右/3无路径。
     * 无有效直道时 cal_valid=0，不能把旧误差当安装标定数据。 */
    if (strcmp(line, "perception") == 0)
    {
        char status[240];
        uint8_t cal_valid =
            Steering_command.applied && Steering_path.straight && Steering_path.source == PATH_DUAL &&
            Steering_path.avoid_offset == 0;
        sprintf(status,
                "perception seq=%lu ok=%u path=%u src=%u pd=%u apply=%u reason=%u bend=%d age_us=%lu "
                "err=%.1f ref=%.0f near=%.1f far=%.1f width=%.0f cal_valid=%u cal=%.1f\r\n",
                (unsigned long)M10P_control_seq, (unsigned)M10P_perception_ok, (unsigned)Steering_path.valid,
                (unsigned)Steering_path.source, (unsigned)Steering_command.computed,
                (unsigned)Steering_command.applied, (unsigned)Steering_command.reason,
                (int)Steering_command.bend, (unsigned long)M10P_build_age_us, (double)Steering_command.error,
                (double)Steering_path.ref_y, (double)Steering_path.near_x, (double)Steering_path.far_x,
                (double)Steering_path.width, (unsigned)cal_valid,
                (double)(cal_valid ? Steering_path.near_x : 0));
        Send_Bluetooth_Data(status);
        return;
    }

    /* 仅按需输出，不增加循环遥测负载；offset正值向右避让，y=0表示保持/释放阶段。 */
    if (strcmp(line, "avoid") == 0)
    {
        char status[128];
        sprintf(status, "avoid path=%u offset_mm=%.1f cone_y_mm=%.0f target_x_mm=%.1f pwm=%u\r\n",
                (unsigned)Steering_path.valid, (double)Steering_path.avoid_offset,
                (double)Steering_path.avoid_y, (double)Steering_path.target_x,
                (unsigned)Steering_command.pwm);
        Send_Bluetooth_Data(status);
        return;
    }

    /* 竞速模式不再输出已删除的启动许可、锁停和超时计数。 */
    if (strcmp(line, "motor") == 0)
    {
        char status[192];
        sprintf(status, "motor race=1 pwm=%u target_x100=%ld speed_x100=%ld frame_age_us=%lu\r\n",
                (unsigned)TIM2->CCR2, (long)(Radar_effective_target * 100), (long)(Speed_now * 100),
                (unsigned long)Radar_ControlAgeUs());
        Send_Bluetooth_Data(status);
        return;
    }
    if (strcmp(line, "radar") == 0)
    {
        char status[192];
        sprintf(status,
                "radar packets=%lu discontinuities=%lu rejected=%lu overflow=%lu raw=%u invalid=%lu\r\n",
                (unsigned long)M10P_stats.packets, (unsigned long)M10P_stats.discontinuities,
                (unsigned long)M10P_stats.rejected, (unsigned long)M10P_stats.scan_overflow,
                (unsigned)LEIDA_raw_count, (unsigned long)Radar_invalid_inputs);
        Send_Bluetooth_Data(status);
        return;
    }

    if (strcmp(line, "get") == 0)
    {
        Tune_SendAll();
        return;
    } /* 精确匹配, 防"getxx"误触发 */

    sp = strchr(line, ' ');
    sc = strchr(line, ':');
    if ((sp == NULL) || ((sc != NULL) && (sc < sp)))
        sp = sc; /* 取更靠前的作切点 */
    if (sp == NULL)
    {
        Diag_Reject("ERR fmt\r\n");
        return;
    }
    *sp = '\0'; /* 切断 → line="kp", sp+1="45..." */
    val = sp + 1;
    while (*val == ' ' || *val == ':')
        val++; /* 跳过":"等非数字前缀 */
    if (*val == '\0')
    {
        Diag_Reject("ERR fmt\r\n");
        return;
    }
    {
        char *end;
        long parsed = strtol(val, &end, 10);
        if (end == val || *end || parsed < -32768 || parsed > 32767)
        {
            Diag_Reject("ERR value\r\n");
            return;
        }
        iv = (int)parsed;
    }

    for (i = 0; i < PARAM_NUM; i++)
    {
        if (strcmp(line, param_tab[i].name) != 0)
            continue;

        if ((iv < param_tab[i].min_i) || (iv > param_tab[i].max_i))
        {
            Diag_Reject("ERR range\r\n"); /* 超范围: 拒绝, 不写值 */
            return;
        }
        /* 直接赋值字段, 下一拍控制循环现读新值即生效。
         * 参数修订号改变后，转向模块重建 D 历史；电机 PI 积分保留 */
        if (!Diag_Parameter(param_tab[i].key, param_tab[i].ptr, (float)iv / mul[param_tab[i].scale]))
        {
            Diag_Reject("ERR busy\r\n");
            return;
        }
        sprintf(ack, "OK %s=%d\r\n", line, iv);
        Send_Bluetooth_Data(ack);
        return;
    }
    Diag_Reject("ERR name\r\n");
}

/* ======================== 【调参】主入口 ======================== */
/* 主循环每圈调用。流程:
 *   BLERX_FLAG==0 → 只轮询待发get回复 (USART6 IDLE 中断置位后才处理)
 *   临界区整体拷出 BLERX_BUFF → 按 \n 分行逐条执行 → 半行尾巴留到下一帧拼 */
void BLE_Tune_Process(void)
{
    static char tail[BLERX_LEN_MAX];
    static uint8_t used, discard;
    uint8_t buf[BLERX_LEN_MAX], len, i, error;
    uint32_t p;
    Tune_GetPoll();
    if (!BLERX_FLAG)
        return;
    p = __get_PRIMASK();
    __disable_irq();
    len = BLERX_LEN;
    error = BLERX_FLAG == 2;
    memcpy(buf, BLERX_BUFF, len);
    Clear_BLERX_BUFF();
    __set_PRIMASK(p);
    if (error)
    {
        used = 0;
        discard = 1;
    }
    for (i = 0; i < len; i++)
    {
        if (buf[i] == '\n')
        {
            if (!discard)
            {
                if (used && tail[used - 1] == '\r')
                    used--;
                tail[used] = 0;
                if (used)
                    Tune_ApplyOne(tail);
            }
            else
                Diag_Reject("ERR overflow\r\n");
            used = 0;
            discard = 0;
        }
        else if (!discard)
        {
            if (buf[i] < 32 && buf[i] != '\r')
            {
                discard = 1;
                used = 0;
            }
            else if (used < sizeof(tail) - 1)
                tail[used++] = (char)buf[i];
            else
            {
                discard = 1;
                used = 0;
                Diag_rx_overflow++;
            }
        }
    }
}
