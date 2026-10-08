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
 * 绝不调用 Midline_PD_Init / Speed_PID_Init 改参数 —— 它们会清零 err/err_l,
 * 车在弯里时 D 项突降会猛抖一下。
 *
 * 使用说明见 蓝牙调参说明书.md。
 */

#include "ble_tune.h"
#include "ble_diag.h"
#include "m10p.h"
#include "m10p_vehicle.h"
#include "timer.h"
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
    {"angle", &BLUE_ANGLE_LEFT_RIGHT, 0,   90, 0},   /* 左右边界扫描范围(度) */
    /* 20261008 新增: 车体几何标定与直道修正权限(实车必须在赛道上重标) */
    {"cx",    &CENTER_X_TARGET_MM, -300, 300, 0},    /* 车体系里"通道中线"应在的横向位置(mm): 发 cx 0 */
    {"eclamp",&MODE0_ERR_CLAMP_MM,  100, 500, 0},    /* mode0 误差限幅(mm): 原200, 现350 */
};
#define PARAM_NUM (sizeof(param_tab) / sizeof(param_tab[0]))

/* ======================== 【遥测】二进制发送底层 ======================== */
typedef union {
    float   f;
    uint8_t b[4];
} FloatByte_t;

void BLE_Tune_Telemetry(float err,float pwm,uint16_t mode)
{
    uint8_t packet[36];float values[8];uint32_t qnan=0x7fc00000;
    if(Diag_mode!=1)return;
    values[0]=err;if(mode>=10)memcpy(&values[0],&qnan,4);
    values[1]=pwm;values[2]=Servo_pd.kp;values[3]=Servo_pd.kd;values[4]=mode;
    values[5]=Speed_now;values[6]=Speed_mubiao;values[7]=LEIDA_speed_dps;
    memcpy(packet,values,32);packet[32]=0;packet[33]=0;packet[34]=0x80;packet[35]=0x7f;
    BLE_Queue(packet,36,0);
}
uint8_t Tune_ConfigValue(uint16_t i,uint16_t*key,float*value)
{if(i>=PARAM_NUM)return 0;*key=100+i;*value=*param_tab[i].ptr;return 1;}

/* ======================== 【调参】get 命令回显 ======================== */
/* 回发全部参数 "名字=整数\r\n", 整数按 scale 四舍五入 (如 kp → "kp=35")。
 * 逐行异步入队，完整消息之间可调度CONTROL；精确备份使用getcfg */
static uint8_t get_index=PARAM_NUM;
static void Tune_SendAll(void){get_index=0;}
static void Tune_GetPoll(void){
    static const int mul[4]={1,10,100,1000};char line[40];int iv;
    if(get_index>=PARAM_NUM || BLE_FreeCritical()<3)return;
    iv=(int)(*param_tab[get_index].ptr*mul[param_tab[get_index].scale]+0.5f);
    sprintf(line,"%s=%d\r\n",param_tab[get_index].name,iv);Send_Bluetooth_Data(line);get_index++;
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

    if(BLE_FreeCritical()<2)return;
    if(Diag_Command(line)){get_index=PARAM_NUM;return;}

    /* 同一主循环读取上一完整处理帧的诊断，距离为前向投影，单位毫米。
     * clear_mm=M10P_MAX_MM表示未发现更近的走廊回波；gap按0.5度桶计数。
     * why 是判据失败原因位图(m10p_vehicle.h), src 是本帧实际使用的转向来源:
     *   0=双侧中线 1=仅左侧跟线 2=仅右侧跟线 3=降级(有回波但判据不全) 4=保持(无回波)
     * pd=1 才表示本帧真的执行了PD, 此时 mode 为 0/5/7 时的 err 才是"中线横向偏差"。
     * ★标定: 车摆在通道正中并摆平, 反复读 cal=, 直接发 "cx <cal>" 即可;
     *   标定后 err 应≈0。cal = cx - err = 当前常数下测出的真实中线横向位置。 */
    if (strcmp(line,"perception")==0) {
        char status[256];
        sprintf(status,"perception seq=%lu ok=%u src=%u why=0x%02x pd=%u points=%u front=%u left=%u right=%u gap=%u clear_mm=%lu age_us=%lu err=%.1f cx=%.1f cal=%.1f mode=%u\r\n",
            (unsigned long)M10P_control_seq,(unsigned)M10P_perception_ok,
            (unsigned)M10P_steer_source,(unsigned)M10P_perception_why,
            (unsigned)Servo_PD_valid,(unsigned)valid_couter,
            (unsigned)M10P_front_bins,(unsigned)M10P_left_bins,(unsigned)M10P_right_bins,
            (unsigned)M10P_front_gap_bins,(unsigned long)M10P_clearance_mm,
            (unsigned long)M10P_build_age_us,
            (double)Servo_pd.err,(double)CENTER_X_TARGET_MM,
            (double)(CENTER_X_TARGET_MM - Servo_pd.err),(unsigned)Diag_detail_u[6]);
        Send_Bluetooth_Data(status);return;
    }

    /* 竞速模式不再输出已删除的启动许可、锁停和超时计数。 */
    if (strcmp(line,"motor")==0) {
        char status[192];
        sprintf(status,"motor race=1 pwm=%u target_x100=%ld speed_x100=%ld frame_age_us=%lu\r\n",
            (unsigned)TIM2->CCR2,(long)(Radar_effective_target*100),
            (long)(Speed_now*100),(unsigned long)Radar_ControlAgeUs());
        Send_Bluetooth_Data(status);return;
    }
    if (strcmp(line,"radar")==0) {
        char status[192];
        sprintf(status,"radar packets=%lu discontinuities=%lu rejected=%lu overflow=%lu raw=%u invalid=%lu\r\n",
            (unsigned long)M10P_stats.packets,(unsigned long)M10P_stats.discontinuities,
            (unsigned long)M10P_stats.rejected,(unsigned long)M10P_stats.scan_overflow,
            (unsigned)LEIDA_raw_count,(unsigned long)Radar_invalid_inputs);
        Send_Bluetooth_Data(status);return;
    }

    if (strcmp(line, "get") == 0) { Tune_SendAll(); return; }  /* 精确匹配, 防"getxx"误触发 */

    sp = strchr(line, ' ');
    sc = strchr(line, ':');
    if ((sp == NULL) || ((sc != NULL) && (sc < sp))) sp = sc;  /* 取更靠前的作切点 */
    if (sp == NULL) { Diag_Reject("ERR fmt\r\n"); return; }
    *sp = '\0';                     /* 切断 → line="kp", sp+1="45..." */
    val = sp + 1;
    while (*val==' ' || *val==':') val++;  /* 跳过":"等非数字前缀 */
    if (*val == '\0') { Diag_Reject("ERR fmt\r\n"); return; }
    {
        char *end; long parsed=strtol(val,&end,10);
        if(end==val || *end || parsed< -32768 || parsed>32767){Diag_Reject("ERR value\r\n");return;}
        iv=(int)parsed;
    }

    for (i = 0; i < PARAM_NUM; i++) {
        if (strcmp(line, param_tab[i].name) != 0) continue;

        if ((iv < param_tab[i].min_i) || (iv > param_tab[i].max_i)) {
            Diag_Reject("ERR range\r\n");   /* 超范围: 拒绝, 不写值 */
            return;
        }
        /* 直接赋值字段, 下一拍控制循环现读新值即生效。
         * 绝不调用 Midline_PD_Init/Speed_PID_Init —— 它们会清零误差状态 */
        if(!Diag_Parameter(100+i,param_tab[i].ptr,(float)iv/mul[param_tab[i].scale])){Diag_Reject("ERR busy\r\n");return;}
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
    static char tail[BLERX_LEN_MAX];static uint8_t used,discard;
    uint8_t buf[BLERX_LEN_MAX],len,i,error;uint32_t p;
    Tune_GetPoll();
    if(!BLERX_FLAG)return;
    p=__get_PRIMASK();__disable_irq();len=BLERX_LEN;error=BLERX_FLAG==2;memcpy(buf,BLERX_BUFF,len);Clear_BLERX_BUFF();__set_PRIMASK(p);
    if(error){used=0;discard=1;}
    for(i=0;i<len;i++){
        if(buf[i]=='\n'){
            if(!discard){if(used && tail[used-1]=='\r')used--;tail[used]=0;if(used)Tune_ApplyOne(tail);}
            else Diag_Reject("ERR overflow\r\n");
            used=0;discard=0;
        }else if(!discard){
            if(buf[i]<32 && buf[i]!='\r'){discard=1;used=0;}
            else if(used<sizeof(tail)-1)tail[used++]=(char)buf[i];
            else {discard=1;used=0;Diag_rx_overflow++;}
        }
    }
}
