
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#define LEIDA_DATA_COUNTER 800
#define LEIDA_ANGLE_CENTER 90.0f
#define LEIDA_ANGLE_LEFT 180.0f
#define LEIDA_ANGLE_RIGHT 0.0f
#define LEIDA_ANGLE_yuliang 75.0f
#define LEIDA_ANGLE_piancha 0.5f
#define PI 3.14159265358979323846f
#define SERVO_PWM_MIN 1170
#define SERVO_PWM_MAX 1720
#define SERVO_PWM_MID 1445
#define arm_cos_f32 cosf
#define arm_sin_f32 sinf
typedef uint8_t u8;typedef uint16_t u16;
typedef struct{float angle,distance;} _LEIDA_DATA;
typedef struct{float _x,_y;} _LEIDA_DATA_plane;
typedef struct{float k,b;} Midline_type;
typedef struct{float kp,kp_2,kp_3,ki,kd,kd_2,kd_3,v_set,v_fb,err_ll,err_l,err,err_sum,erry,out,out_max,out_min;} pid_type;
static struct{uint16_t CCR1;} timer3={1445};
#define TIM3 (&timer3)
#define TIM4 4
static uint16_t encoder_counter;
static unsigned servo_writes;
uint16_t TIM_GetCounter(int ignored){return encoder_counter;}
void TIM_SetCounter(int ignored,uint16_t value){encoder_counter=value;}
uint32_t Diag_detail_u[24];float Diag_detail_f[16];
uint32_t LEIDA_parse_calls,LEIDA_sync_failures,LEIDA_short_inputs,LEIDA_missing_packets;
uint16_t LEIDA_speed_dps,LEIDA_raw_count;
static uint8_t lidar_packet[47];static uint16_t lidar_pending;
float zhongxian_junzhi,zhongxian_chuizhi;
uint8_t LEIDA_vertical_valid,Servo_PD_valid,Servo_reject_reason;
static uint8_t pd_history_valid;static uint16_t pd_previous_mode;static uint32_t pd_previous_us;
float BLUE_Y_RIGHT=1200,BLUE_Y_LEFT=1350,BLUE_Y_STRA_SEL=0,BLUE_Y_STRA=750;
float BLUE_DIS_RIGHT=50,BLUE_DIS_LEFT=50,paodao_distance=800;
static uint32_t clock_us;
uint32_t Diag_TimeUs(void){return clock_us;}
void Diag_Fit(const void *line,uint8_t valid){}
void Servo_ChangePwm(uint16_t value){timer3.CCR1=value;servo_writes++;}
float Encoder_cnt,Speed_now;int16_t Encoder_cnt_arr[5];uint16_t Encoder_cnt_temp;
uint16_t LEIDA_DATA_HANDLE10(_LEIDA_DATA_plane *,u16);
#define TURN_GUARD_US 350000u
#define TURN_EXIT_FRAMES 2u
#define TURN_EXIT_ERROR_MM 100.0f
#define TURN_ENTRY_PWM 75.0f /* 入弯附加量同时不超过本帧|P|，不放大小误差噪声 */
#define TURN_MIN_OFFSET 20  /* 接近中位的候选不能成为弯道保持依据 */
#define TURN_RETRACT_PWM 60 /* 同模式单次明显收舵，需要出弯确认或期限到达 */
typedef struct {
    uint32_t observed_us;
    uint32_t evidence_us, pending_evidence, pending_us; /* 独立观测时间，均支持回绕 */
    uint8_t pending_direction; /* 0无，1右，2左；等待期间必须限速 */
    uint16_t mode, pwm;
    uint8_t active, straight_frames;
} TurnGuard;

uint8_t Diag_RadarPacket(const uint8_t *a)
{
    uint8_t crc = 0, b;
    uint16_t i, start = (uint16_t)(a[4] | a[5] << 8), end = (uint16_t)(a[42] | a[43] << 8);
    Diag_detail_u[12]++;
    for (i = 0; i < 46; i++) {
        crc ^= a[i];
        for (b = 0; b < 8; b++)
            crc = (uint8_t)((crc << 1) ^ ((crc & 0x80) ? 0x4d : 0));
    }
    if (crc != a[46])
        Diag_detail_u[13]++;
    if (a[1] != 0x2c)
        Diag_detail_u[14]++;
    if (start >= 36000 || end >= 36000)
        Diag_detail_u[15]++;
    return a[0] == 0x54 && a[1] == 0x2c && start < 36000 && end < 36000 && crc == a[46];
}
void LEIDA_ParserReset(void)
{
    lidar_pending = 0;
}
uint16_t LEIDA_DATA_HANDLE1(_LEIDA_DATA data[], u8 arr[], u16 size)
{
    uint16_t i, j = 0, k, skip;
    float start, end, angle;
    LEIDA_parse_calls++;
    LEIDA_raw_count = 0;
    Diag_detail_u[4] |= 64;
    Diag_detail_u[17] = 0xffffffffu;
    /* 先全清: 同一个 800 槽数组被前后两块数据复用, 上一块写过的槽位这一块可能不再
     * 被写到, 残留旧点会被 HANDLE3_2 当成有效点混进 valid_couter。 */
    memset(data, 0, LEIDA_DATA_COUNTER * sizeof(*data));
    if (!size) {
        LEIDA_short_inputs++;
        return 0;
    }
    for (i = 0; i < size; i++) {
        if (!lidar_pending && arr[i] != 0x54)
            continue;
        lidar_packet[lidar_pending++] = arr[i];
        if (lidar_pending < 47)
            continue;
        if (Diag_RadarPacket(lidar_packet)) {
            if (Diag_detail_u[17] == 0xffffffffu)
                Diag_detail_u[17] = i >= 46 ? i - 46 : 0;
            LEIDA_speed_dps = (uint16_t)(lidar_packet[2] | lidar_packet[3] << 8);
            start = (lidar_packet[4] | lidar_packet[5] << 8) / 100.0f;
            end = (lidar_packet[42] | lidar_packet[43] << 8) / 100.0f;
            if (end < start)
                end += 360.0f;
            for (k = 0; k < 12 && j < LEIDA_DATA_COUNTER; k++, j++) {
                data[j].distance = (float)(lidar_packet[6 + 3 * k] | lidar_packet[7 + 3 * k] << 8);
                /* LD14P 一包 47 字节: [0]0x54 [1]0x2C [2..3]转速 [4..5]起始角
                 * [6..41]12 个点, 每点 3 字节(前 2 字节=距离, 小端; 第 3 字节本代码不用)
                 * [42..43]结束角 [44..45]时间戳 [46]CRC8。点角度不读包内的, 用起止角插值:
                 * 12 个点把首尾两端都算进去了, 所以除 11 而不是 12。 */
                angle = start + (end - start) * k / 11.0f;
                angle = 360.0f - angle + LEIDA_ANGLE_CENTER;
                while (angle >= 360.0f)
                    angle -= 360.0f;
                while (angle < 0)
                    angle += 360.0f;
                data[j].angle = angle;
                if (data[j].distance > 0)
                    Diag_detail_u[18] |= 1u << (uint16_t)(angle / 30.0f);
            }
            lidar_pending = 0;
        } else {
            LEIDA_missing_packets++;
            Diag_detail_u[16]++;
            /* Resynchronize bytewise; preserve a potential header inside a bad packet. */
            for (skip = 1; skip < 47 && lidar_packet[skip] != 0x54; skip++) {
            }
            lidar_pending = (uint16_t)(47 - skip);
            if (lidar_pending)
                memmove(lidar_packet, lidar_packet + skip, lidar_pending);
        }
    }
    if (!j && size >= 47)
        LEIDA_sync_failures++;
    LEIDA_raw_count = j;
    return j;
}
uint16_t LEIDA_DATA_HANDLE10(_LEIDA_DATA_plane arr[], u16 size)
{
    uint16_t i, n = 0;
    float lo = FLT_MAX, hi = -FLT_MAX, threshold, previous = 0, x;
    for (i = 0; i < size; i++) {
        if (!(arr[i]._x <= FLT_MAX && arr[i]._x >= -FLT_MAX && arr[i]._y <= FLT_MAX && arr[i]._y >= -FLT_MAX))
            continue;
        arr[n++] = arr[i];
    }
    size = n;
    if (size < 3)
        return size;
    for (i = 0; i < size; i++) {
        if (arr[i]._x < lo)
            lo = arr[i]._x;
        if (arr[i]._x > hi)
            hi = arr[i]._x;
    }
    threshold = 5 * (hi - lo) / size;
    if (threshold < 20)
        threshold = 20;
    /* Remove isolated jumps, preserve constant-x walls and endpoints. */
    for (i = 0, n = 0; i < size; i++) {
        x = arr[i]._x;
        if (i == 0 || i + 1 == size || fabs(x - previous) <= threshold || fabs(x - arr[i + 1]._x) <= threshold)
            arr[n++] = arr[i];
        previous = x;
    }
    return n;
}
uint16_t LEIDA_DATA_HANDLE4(_LEIDA_DATA_plane data_center[], _LEIDA_DATA arr[], u16 size)
{
    int16_t index[181];
    uint16_t i, bin, n = 0;
    int right, left;
    float x, y;
    /* 先按1度索引，再配对；不再为每个角度重复扫描全部点。 */
    for (i = 0; i <= 180; i++) index[i] = -1;
    for (i = 0; i < size; i++) {
        if (arr[i].distance < 100 || arr[i].distance > 2000 ||
            arr[i].angle < 0 || arr[i].angle > 180) continue;
        bin = (uint16_t)(arr[i].angle + 0.5f);
        if (index[bin] < 0 || arr[i].distance < arr[index[bin]].distance)
            index[bin] = i;
    }
    zhongxian_junzhi = 0;
    for (i = 0; i <= 75; i++) {
        right = index[i]; left = index[180 - i];
        if (right < 0 || left < 0) continue;
        x = (arr[right].distance * arm_cos_f32(arr[right].angle * PI / 180) +
             arr[left].distance * arm_cos_f32(arr[left].angle * PI / 180)) / 2;
        y = (arr[right].distance * arm_sin_f32(arr[right].angle * PI / 180) +
             arr[left].distance * arm_sin_f32(arr[left].angle * PI / 180)) / 2;
        if (y >= 0 && y <= 800) {
            data_center[n]._x = x;
            data_center[n++]._y = y;
        }
    }
    n = LEIDA_DATA_HANDLE10(data_center, n);
    for (i = 0; i < n; i++) zhongxian_junzhi += data_center[i]._x;
    if (n) zhongxian_junzhi /= n;
    return n;
}
float LEIDA_DATA_HANDLE11(_LEIDA_DATA_plane arr[], u16 start, u16 end)
{
    uint16_t i;
    float lo = FLT_MAX, hi = -FLT_MAX, sum = 0, x;
    LEIDA_vertical_valid = 0;
    if (end <= start || end - start < 2 || end > LEIDA_DATA_COUNTER / 2)
        return 0;
    for (i = start; i < end; i++) {
        x = arr[i]._x;
        if (!(x <= FLT_MAX && x >= -FLT_MAX))
            return 0;
        if (x < lo)
            lo = x;
        if (x > hi)
            hi = x;
        sum += x;
    }
    if (hi - lo < 10) {
        LEIDA_vertical_valid = 1;
        return sum / (end - start);
    }
    return 0;
}
uint8_t Midline_fit(_LEIDA_DATA_plane *points, int start, int end, Midline_type *line)
{
    int i, n = end - start;
    float sx = 0, sy = 0, xx = 0, xy = 0, x, y;
    line->k = line->b = 0;
    if (start < 0 || n < 2 || end > LEIDA_DATA_COUNTER / 2) {
        Diag_Fit(line, 0);
        return 0;
    }
    for (i = start; i < end; i++) {
        x = points[i]._x;
        y = points[i]._y;
        if (!(x <= FLT_MAX && x >= -FLT_MAX && y <= FLT_MAX && y >= -FLT_MAX)) {
            Diag_Fit(line, 0);
            return 0;
        }
        sx += x;
        sy += y;
    }
    sx /= n;
    sy /= n;
    for (i = start; i < end; i++) {
        x = points[i]._x - sx;
        xx += x * x;
        xy += x * (points[i]._y - sy);
    }
    if (xx <= 1e-6f) {
        line->b = sy;
        Diag_Fit(line, 0);
        return 0;
    }
    line->k = xy / xx;
    line->b = sy - line->k * sx;
    if (!(line->k <= FLT_MAX && line->k >= -FLT_MAX && line->b <= FLT_MAX && line->b >= -FLT_MAX)) {
        line->k = line->b = 0;
        Diag_Fit(line, 0);
        return 0;
    }
    Diag_Fit(line, 1);
    return 1;
}
void Midline_PD_Reset(void)
{
    pd_history_valid = 0;
    Servo_PD_valid = 0;
}
static uint16_t pd_reject(void)
{
    if (!Servo_reject_reason) Servo_reject_reason = 4;
    Diag_detail_u[4] &= ~1u;
    Diag_detail_u[4] |= 512u;
    Midline_PD_Reset();
    return (uint16_t)TIM3->CCR1;
}
static uint8_t turn_direction(uint16_t mode)
{
    return (mode == 1 || mode == 3 || mode == 8) ? 1 :
           (mode == 2 || mode == 4 || mode == 9) ? 2 : 0;
}
static uint8_t turn_rank(uint16_t mode)
{
    return (mode == 3 || mode == 4) ? 3 : (mode == 8 || mode == 9) ? 2 : 1;
}
static uint8_t wall_reference(_LEIDA_DATA_plane *points, uint16_t start, uint16_t end,
                              uint16_t mode, float target, float *reference)
{
    uint16_t i, n = 0;
    float sx = 0, sy = 0, yy = 0, xy = 0, residual = 0;
    float low = FLT_MAX, high = -FLT_MAX, a, b, x, y;
    for (i = start; i < end; i++) {
        x = points[i]._x; y = points[i]._y;
        if (!(fabs(x) <= FLT_MAX && y >= 150 && y <= 1800)) continue;
        if ((mode == 1 && x > -80) || (mode == 2 && x < 80)) continue;
        sx += x; sy += y; n++;
        if (y < low) low = y;
        if (y > high) high = y;
    }
    Diag_detail_u[9] = n;
    if (n < 6 || high - low < 250 || target < low - 250 || target > high + 250)
        return 0;
    sx /= n; sy /= n;
    for (i = start; i < end; i++) {
        x = points[i]._x; y = points[i]._y;
        if (!(fabs(x) <= FLT_MAX && y >= 150 && y <= 1800)) continue;
        if ((mode == 1 && x > -80) || (mode == 2 && x < 80)) continue;
        yy += (y - sy) * (y - sy); xy += (y - sy) * (x - sx);
    }
    if (yy < 1) return 0;
    a = xy / yy; b = sx - a * sy;
    if (fabs(a) > 2) return 0;
    for (i = start; i < end; i++) {
        x = points[i]._x; y = points[i]._y;
        if (!(fabs(x) <= FLT_MAX && y >= 150 && y <= 1800)) continue;
        if ((mode == 1 && x > -80) || (mode == 2 && x < 80)) continue;
        residual += (x - a * y - b) * (x - a * y - b);
    }
    if (residual / n > 60 * 60) return 0;
    *reference = a * target + b;
    Diag_detail_f[8] = *reference;
    Diag_detail_f[9] = target;
    /* ref_dy现在表示外推距离，0表示目标位于真实覆盖区间内。 */
    Diag_detail_f[10] = target < low ? low - target : target > high ? target - high : 0;
    Diag_detail_u[4] |= 2;
    return 1;
}
uint16_t Midline_PD_Calculate(_LEIDA_DATA_plane points[], pid_type *pid, Midline_type *line,
                    float mid, uint16_t start, uint16_t end, uint16_t mode)
{
    uint16_t i;
    uint32_t now = Diag_TimeUs(), dt = now - pd_previous_us;
    float e = 0, kp, kd, p, d, original_d, output, mag, x = 0, target, entry = 0;
    Servo_PD_valid = 0;
    Servo_reject_reason = 0;
    Diag_detail_u[6] = pd_previous_mode;
    Diag_detail_u[7] = start;
    Diag_detail_u[8] = end;
    Diag_detail_u[9] = end > start ? end - start : 0;
    Diag_detail_f[13] = pid->err_l;
    Diag_detail_f[11] = line->k;
    Diag_detail_f[12] = line->b;
    if (mode > 9 || end <= start || end > LEIDA_DATA_COUNTER / 2) {
        Servo_reject_reason = 1;
        if (mode == 1 || mode == 2)
            Diag_detail_u[4] |= 8;
        return pd_reject();
    }
    if (mode == 0) {
        if (!(fabs(line->k) > 0.1f && fabs(line->k) <= FLT_MAX && fabs(line->b) <= FLT_MAX)) {
            Servo_reject_reason = 2;
            return pd_reject();
        }
        target = BLUE_Y_STRA_SEL == 1 ? BLUE_Y_STRA : points[end - 1]._y;
        e = -((target - line->b) / line->k - 50);
        /* 保留全局±500保护，不再将正常中线纠偏永久压成±200。
         * 固定前视不许远超实际覆盖，否则应降级而非盲目外推。 */
        if (BLUE_Y_STRA_SEL == 1 && fabs(target - points[end - 1]._y) > 250) {
            Servo_reject_reason = 2;
            return pd_reject();
        }
    } else if (mode == 1 || mode == 2) {
        target = mode == 1 ? BLUE_Y_RIGHT : BLUE_Y_LEFT;
        if (!wall_reference(points, start, end, mode, target, &x)) {
            Servo_reject_reason = 3;
            Diag_detail_u[4] |= 8;
            return pd_reject();
        }
        /* 期望: 墙保持在车侧 paodao*BLUE_DIS/100 mm 处。x 是"离该墙的点"的横向坐标,
         * e<0 = 车离墙太远, 该往右打 (mode1); e>0 = 该往左打 (mode2)。 */
        e = mode == 1 ? -(x + paodao_distance * BLUE_DIS_RIGHT / 100) : -(x - paodao_distance * BLUE_DIS_LEFT / 100);
    } else if (mode == 3 || mode == 4 || mode == 8 || mode == 9) {
        if (!(fabs(line->k) <= FLT_MAX))
            return pd_reject();
        /* 转弯力度: 线越斜(弯越急)|k| 越小, 给的固定误差越大; |k|<0.35 直接顶到 500。
         * 方向由模式决定, 不看 k 的符号 —— 拟合退化时也不给反向指令。 */
        mag = fabs(line->k) < 0.35f ? 500.0f : 175.0f / fabs(line->k);
        e = (mode == 3 || mode == 8) ? -mag : mag;
    } else if (mode == 5) {
        if (!LEIDA_vertical_valid)
            return pd_reject();
        e = 50 - zhongxian_chuizhi;
    } else {
        if (end - start < 2)
            return pd_reject();
        for (i = start; i < end; i++) {
            if (!(fabs(points[i]._x) <= FLT_MAX))
                return pd_reject();
            x += points[i]._x;
        }
        x /= end - start;
        e = 50 - x;
        Diag_detail_f[14] = x;
        Diag_detail_u[4] |= 256;
    }
    if (!(fabs(e) <= FLT_MAX))
        return pd_reject();
    if (e > 500)
        e = 500;
    if (e < -500)
        e = -500;
    kp = (mode == 1 || mode == 2) ? pid->kp_3 : (mode == 0 || (mode >= 5 && mode <= 7)) ? pid->kp
                                                                                        : pid->kp_2;
    kd = (mode == 1 || mode == 2) ? pid->kd_3 : (mode == 0 || (mode >= 5 && mode <= 7)) ? pid->kd
                                                                                        : pid->kd_2;
    /* 不同模式的误差来自不同测量目标，不能直接相减当作运动变化。
     * 例如大左转500 -> 侧墙左转82，会产生假的负D并把舵机拉回中位。
     * 切模式首帧只用P；弯道延续由主循环的有界TurnGuard负责。 */
    if (!pd_history_valid || !dt || dt > 250000u || mode != pd_previous_mode) {
        pid->err_l = e;
        Diag_detail_u[4] |= 4;
    } else
        kd *= 115000.0f / dt;
    p = 10 * kp * e;
    d = 10 * kd * (e - pid->err_l);
    original_d = d;
    /* D 限幅 ±150: 舵机单边行程 275 (SERVO_PWM_MID 1445 → MIN 1170), 150 约半程,
     * 再小就会把入弯那记踢腿削掉。实测 mode 3 入弯 D ≈ -124, 原来的 ±60 砍掉一半。 */
    if (d > 150)
        d = 150;
    if (d < -150)
        d = -150;
    /* D 只能把 P 往中位拉, 不许把修正方向拽反 */
    if ((p >= 0 && p + d < 0) || (p <= 0 && p + d > 0))
        d = -p;
    if (d != original_d)
        Diag_detail_u[4] |= 2048;
    /* 入弯/换向/升级为更大弯时，单独给有界补偿，不使用跨测量目标的假D。
     * 只增强P已经指向目标弯向的指令；降级、同模式、无效后同模式恢复不重复加。
     * DETAIL中的P/D保持原义，补偿量=pwm_unclamped-pwm_mid-pd_p-pd_d。 */
    if (turn_direction(mode) &&
        (turn_direction(mode) != turn_direction(pd_previous_mode) || turn_rank(mode) > turn_rank(pd_previous_mode)) &&
        ((turn_direction(mode) == 1 && p < 0) || (turn_direction(mode) == 2 && p > 0))) {
        entry = fabs(p) < TURN_ENTRY_PWM ? fabs(p) : TURN_ENTRY_PWM;
        if (p < 0)
            entry = -entry;
    }
    output = 10 * mid + p + d + entry;
    /* 打方向的模式(1/3/8 往右, 2/4/9 往左)不许把舵机指到中位的另一边, 防反打 */
    if ((mode == 1 || mode == 3 || mode == 8) && output > 10 * mid) {
        output = 10 * mid;
        Diag_detail_u[4] |= 1024;
    }
    if ((mode == 2 || mode == 4 || mode == 9) && output < 10 * mid) {
        output = 10 * mid;
        Diag_detail_u[4] |= 1024;
    }
    if (!(fabs(output) <= FLT_MAX))
        return pd_reject();
    Diag_detail_u[4] |= 1;
    Diag_detail_f[0] = pid->err_l;
    Diag_detail_f[1] = e;
    Diag_detail_f[2] = p;
    Diag_detail_f[3] = d;
    Diag_detail_f[4] = kp;
    Diag_detail_f[5] = kd;
    Diag_detail_f[6] = output;
    Diag_detail_f[7] = 10 * mid;
    if (output < SERVO_PWM_MIN || output > SERVO_PWM_MAX)
        Diag_detail_u[4] |= 16;
    if (output < SERVO_PWM_MIN)
        output = SERVO_PWM_MIN;
    if (output > SERVO_PWM_MAX)
        output = SERVO_PWM_MAX;
    pid->err = pid->err_l = e;
    pd_previous_us = now;
    pd_previous_mode = mode;
    pd_history_valid = 1;
    Servo_PD_valid = 1;
    return (uint16_t)output;
}
uint16_t Midline_PD(_LEIDA_DATA_plane points[], pid_type *pid, Midline_type *line,
                    float mid, uint16_t start, uint16_t end, uint16_t mode)
{
    uint16_t pwm = Midline_PD_Calculate(points, pid, line, mid, start, end, mode);
    if (Servo_PD_valid)
        Servo_ChangePwm(pwm);
    return pwm;
}
uint16_t TurnGuard_Apply(TurnGuard *state, uint16_t mode, uint8_t valid,
                        uint8_t straight, uint16_t pwm, uint32_t now, uint8_t *held)
{
    uint8_t direction = turn_direction(mode);
    int offset, previous_offset;
    *held = 0;
    if (!valid) state->pending_direction = 0;
    if (valid && direction && turn_direction(state->mode) &&
        direction != turn_direction(state->mode)) {
        /* 两个独立的侧墙观测才能确认反向；重复缓存不会给确认计数。
         * 待确认只短时保舵，主循环同步限速；紧急障碍绕过此仲裁。 */
        if (state->pending_direction != direction) {
            state->pending_direction = direction;
            state->pending_us = now;
            state->pending_evidence = state->evidence_us;
            *held = 1;
            return state->pwm;
        }
        if (state->evidence_us == state->pending_evidence ||
            (uint32_t)(now - state->pending_us) < 60000u) {
            *held = 1;
            return state->pwm;
        }
        state->pending_direction = 0;
    } else if (valid) state->pending_direction = 0;
    /* unsigned差值允许微秒时钟回绕；HOLD/INVALID绝不刷新这个时刻。 */
    if (state->active && (uint32_t)(now - state->observed_us) >= TURN_GUARD_US) {
        state->active = 0;
        state->straight_frames = 0;
    }
    if (!valid) {
        state->straight_frames = 0;
        return pwm;
    }
    if (direction) {
        state->straight_frames = 0;
        offset = direction == 1 ? SERVO_PWM_MID - (int)pwm : (int)pwm - SERVO_PWM_MID;
        previous_offset = direction == 1 ? SERVO_PWM_MID - (int)state->pwm : (int)state->pwm - SERVO_PWM_MID;
        /* 回中/方向矛盾不能覆盖有效锚点，更不能用来刷新保持期限。 */
        if (offset < TURN_MIN_OFFSET) {
            if (state->active && direction == turn_direction(state->mode)) {
                *held = 1;
                return state->pwm;
            }
            state->active = 0;
            return pwm;
        }
        if (state->active && direction == turn_direction(state->mode) &&
            ((turn_rank(mode) < turn_rank(state->mode) && offset < previous_offset) ||
             previous_offset - offset > TURN_RETRACT_PWM)) {
            *held = 1;
            return state->pwm;
        }
        /* 同向增强立即执行；反向观测立即替换旧状态，不锁死旧方向。 */
        state->active = 1;
        state->mode = mode;
        state->pwm = pwm;
        state->observed_us = now;
        return pwm;
    }
    if (!state->active)
        return pwm;
    state->straight_frames = straight ? state->straight_frames + 1 : 0;
    if (state->straight_frames >= TURN_EXIT_FRAMES) {
        state->active = 0;
        state->straight_frames = 0;
        state->mode = 0;
        return pwm;
    }
    *held = 1;
    return state->pwm;
}
void Get_Encoder(void)
{
    uint16_t i,counter;static uint16_t previous_counter;
    int16_t signed_count;
    counter=TIM_GetCounter(TIM4);
    Encoder_cnt_temp=(uint16_t)(counter-previous_counter);previous_counter=counter;
    signed_count=(int16_t)(Encoder_cnt_temp<32768?(int32_t)Encoder_cnt_temp:(int32_t)Encoder_cnt_temp-65536);
    Encoder_cnt = signed_count;

    for (i = 0; i < 5 - 1; i++) {
        Encoder_cnt_arr[i] = Encoder_cnt_arr[i + 1];
        Encoder_cnt += Encoder_cnt_arr[i];
    }
    Encoder_cnt_arr[i] = signed_count;
    Encoder_cnt /= 5;
    Speed_now = (Encoder_cnt * 100) / (4 * 11 * 6.25);
}
#define NAV_CACHE_US 200000u
#define NAV_HALF_WIDTH_MM 125.0f
#define NAV_MARGIN_MM 50.0f
#define NAV_STOP_MM 400.0f
#define NAV_LOOK_MM 1000.0f
#define NAV_FRONT_MM 200.0f
#define NAV_ALLOW_AVOID 1 /* 已按用户尺寸配置；仍须低速验证实际转角/停止距离 */
#define NAV_CLEAR 0u
#define NAV_UNKNOWN 1u
#define NAV_OBSTACLE 2u
#define NAV_AVOID_LEFT 3u
#define NAV_AVOID_RIGHT 4u
typedef struct {
    uint16_t front_bins, left_bins, right_bins, max_age_ms;
    uint32_t left_stamp, right_stamp;
    float obstacle_y, obstacle_x, obstacle_width;
    uint8_t action; /* 上述NAV_*；0不是“没有数据”，而是前方覆盖足够 */
} NavigationObservation;
typedef struct {
    float speed_limit;
    uint16_t pwm;
    uint8_t reason, override_steering;
} NavigationCommand;
extern NavigationObservation Navigation;
NavigationCommand LEIDA_NavigationCommand(uint8_t valid, uint16_t mode,
                                          uint8_t pending, uint32_t pending_age);
void LEIDA_ScanReset(void);
void LEIDA_ScanUpdate(const _LEIDA_DATA *raw, uint16_t count, uint32_t input_us);
uint16_t LEIDA_ScanSnapshot(_LEIDA_DATA *out, uint32_t now);
void LEIDA_InspectNavigation(uint32_t now);

/* ===== 带年龄的角度缓存与独立近障碍检测 ===== */
typedef struct {
    _LEIDA_DATA point;
    float x, y;             /* 转换一次，多处复用 */
    uint32_t measured_us;   /* 用包内扫描角差估算；不是主机接收时刻 */
    uint8_t seen;
} NavigationBin;
static NavigationBin navigation_bins[360];
NavigationObservation Navigation;

void LEIDA_ScanReset(void)
{
    memset(navigation_bins, 0, sizeof navigation_bins);
    memset(&Navigation, 0, sizeof Navigation);
    Navigation.action = NAV_UNKNOWN;
}

void LEIDA_ScanUpdate(const _LEIDA_DATA *raw, uint16_t count, uint32_t input_us)
{
    uint16_t i, bin;
    float last_angle, delta;
    uint32_t measured;
    /* 异常转速下不能可靠估计点年龄，宁可清空，也不沿用旧空间。 */
    if (!count || LEIDA_speed_dps < 1500 || LEIDA_speed_dps > 3000) {
        LEIDA_ScanReset();
        return;
    }
    last_angle = raw[count - 1].angle;
    for (i = 0; i < count; i++) {
        if (!(raw[i].angle >= 0 && raw[i].angle < 360))
            continue;
        bin = (uint16_t)(raw[i].angle + 0.5f) % 360;
        /* 车体坐标角随扫描递减；最末点前的角差对应较早的测量。
         * 额外扣3ms覆盖DMA尾部未完成包；仅作为本机年龄估计。 */
        delta = raw[i].angle - last_angle;
        if (delta < 0) delta += 360;
        measured = input_us - (uint32_t)(delta * 1000000.0f / LEIDA_speed_dps) - 3000u;
        if (navigation_bins[bin].seen &&
            (int32_t)(measured - navigation_bins[bin].measured_us) < 0) continue;
        /* 同一角度只保留较新观测；近乎同时的点取最近，避免远背景盖住锥桶。 */
        if (navigation_bins[bin].seen &&
            (uint32_t)(measured - navigation_bins[bin].measured_us) < 2000u &&
            navigation_bins[bin].point.distance >= 100 &&
            navigation_bins[bin].point.distance < raw[i].distance)
            continue;
        navigation_bins[bin].point = raw[i];
        navigation_bins[bin].measured_us = measured;
        navigation_bins[bin].seen = raw[i].distance >= 100 && raw[i].distance <= 4000;
        if (navigation_bins[bin].seen) {
            navigation_bins[bin].x = raw[i].distance * arm_cos_f32(raw[i].angle * PI / 180);
            navigation_bins[bin].y = raw[i].distance * arm_sin_f32(raw[i].angle * PI / 180);
        }
    }
}

static uint8_t navigation_fresh(uint16_t bin, uint32_t now)
{
    return navigation_bins[bin].seen &&
           (uint32_t)(now - navigation_bins[bin].measured_us) <= NAV_CACHE_US;
}

uint16_t LEIDA_ScanSnapshot(_LEIDA_DATA *out, uint32_t now)
{
    uint16_t i, count = 0;
    for (i = 0; i < 360; i++)
        if (navigation_fresh(i, now))
            out[count++] = navigation_bins[i].point;
    return count;
}

/* 检查目标前后扫掠走廊。此处是低速几何可行性，不替代实车转角标定。 */
static uint8_t navigation_path_clear(float lateral, float target_y, uint32_t now)
{
    uint16_t i, bin;
    float y, x, angle, reach, path_x;
    if (fabs(lateral) > 450 || target_y < 450) return 0;
    /* 每100mm采样中心线及左右边缘；所有方向均须有更新的远端回波。 */
    for (y = 200; y <= target_y + NAV_FRONT_MM; y += 100) {
        path_x = lateral * (y < target_y ? y / target_y : 1);
        for (i = 0; i < 3; i++) {
            x = path_x + ((int)i - 1) * (NAV_HALF_WIDTH_MM + NAV_MARGIN_MM);
            angle = atan2f(y, x) * 180 / PI;
            bin = (uint16_t)(angle + 0.5f);
            reach = sqrtf(x * x + y * y);
            if (!navigation_fresh(bin, now) || navigation_bins[bin].point.distance < reach + 50)
                return 0;
        }
    }
    /* 同时检查走廊内部，防止只查三条射线漏掉窄锥桶。 */
    for (i = 0; i < 180; i++) {
        if (!navigation_fresh(i, now)) continue;
        y = navigation_bins[i].y;
        if (y < 100 || y > target_y + NAV_FRONT_MM) continue;
        path_x = lateral * (y < target_y ? y / target_y : 1);
        if (fabs(navigation_bins[i].x - path_x) < NAV_HALF_WIDTH_MM + NAV_MARGIN_MM)
            return 0;
    }
    return 1;
}

void LEIDA_InspectNavigation(uint32_t now)
{
    uint16_t i, age, nearest_bin = 0, lo, hi;
    uint32_t left_age = 0xffffffffu, right_age = 0xffffffffu;
    float closest = NAV_LOOK_MM + 1, x, y;
    uint8_t clustered;
    memset(&Navigation, 0, sizeof Navigation);
    for (i = 0; i < 360; i++) {
        if (!navigation_fresh(i, now)) continue;
        age = (uint16_t)((now - navigation_bins[i].measured_us) / 1000u);
        if (age > Navigation.max_age_ms) Navigation.max_age_ms = age;
        if (i >= 70 && i <= 110) Navigation.front_bins++;
        if (i >= 105 && i <= 180) {
            Navigation.left_bins++;
            if (now - navigation_bins[i].measured_us < left_age) {
                left_age = now - navigation_bins[i].measured_us;
                Navigation.left_stamp = navigation_bins[i].measured_us;
            }
        }
        if (i <= 75) {
            Navigation.right_bins++;
            if (now - navigation_bins[i].measured_us < right_age) {
                right_age = now - navigation_bins[i].measured_us;
                Navigation.right_stamp = navigation_bins[i].measured_us;
            }
        }
        x = navigation_bins[i].x;
        y = navigation_bins[i].y;
        if (y < 100 || y > NAV_LOOK_MM || fabs(x) > NAV_HALF_WIDTH_MM + NAV_MARGIN_MM +
            (fabs((float)TIM3->CCR1 - SERVO_PWM_MID) > 100 ? 75 : 0))
            continue;
        /* 两个相邻角度形成小簇；400mm以内单个可信回波也立即停车。
         * 这里不做“远点数量足够就清空近点”的判断。 */
        clustered = y <= NAV_STOP_MM;
        if (i > 0 && navigation_fresh(i - 1, now) &&
            fabs(navigation_bins[i - 1].x - x) < 120 &&
            fabs(navigation_bins[i - 1].y - y) < 120)
            clustered = 1;
        if (i < 359 && navigation_fresh(i + 1, now) &&
            fabs(navigation_bins[i + 1].x - x) < 120 &&
            fabs(navigation_bins[i + 1].y - y) < 120)
            clustered = 1;
        if (clustered && y < closest) {
            closest = y;
            nearest_bin = i;
            Navigation.obstacle_x = x;
            Navigation.obstacle_y = y;
        }
    }
    Navigation.action = Navigation.front_bins >= 29 ? NAV_CLEAR : NAV_UNKNOWN;
    if (closest > NAV_LOOK_MM) return;
    Navigation.action = NAV_OBSTACLE;
    /* 扩展相邻回波簇，宽墙交给弯道规划；小目标才尝试绕行。 */
    lo = hi = nearest_bin;
    while (lo > 0 && navigation_fresh(lo - 1, now) &&
           fabs(navigation_bins[lo - 1].x - navigation_bins[lo].x) < 120 &&
           fabs(navigation_bins[lo - 1].y - navigation_bins[lo].y) < 120) lo--;
    while (hi < 359 && navigation_fresh(hi + 1, now) &&
           fabs(navigation_bins[hi + 1].x - navigation_bins[hi].x) < 120 &&
           fabs(navigation_bins[hi + 1].y - navigation_bins[hi].y) < 120) hi++;
    Navigation.obstacle_width = fabs(navigation_bins[hi].x - navigation_bins[lo].x);
    if (!NAV_ALLOW_AVOID || closest <= NAV_STOP_MM || Navigation.obstacle_width > 250) return;
    /* 左右分别验证一条带车宽余量的折线路径，未知扇区直接否决。 */
    x = Navigation.obstacle_x + NAV_HALF_WIDTH_MM + NAV_MARGIN_MM +
        Navigation.obstacle_width * 0.5f + 50;
    if (navigation_path_clear(x, closest, now))
        Navigation.action = NAV_AVOID_RIGHT;
    else if (navigation_path_clear(Navigation.obstacle_x - NAV_HALF_WIDTH_MM - NAV_MARGIN_MM -
                                  Navigation.obstacle_width * 0.5f - 50, closest, now))
        Navigation.action = NAV_AVOID_LEFT;
}
/* ===== 导航缓存结束 ===== */

/* 导航只生成决策，不直接写电机/舵机；主循环与TIM5各自统一执行。 */
NavigationCommand LEIDA_NavigationCommand(uint8_t valid, uint16_t mode,
                                          uint8_t pending, uint32_t pending_age)
{
    NavigationCommand command = {0, SERVO_PWM_MID, 1, 0};
    uint8_t bend = mode == 1 || mode == 2 || mode == 3 || mode == 4 || mode == 8 || mode == 9;
    if (Navigation.action == NAV_AVOID_LEFT || Navigation.action == NAV_AVOID_RIGHT) {
        command.pwm = Navigation.action == NAV_AVOID_LEFT ? SERVO_PWM_MID + 150 : SERVO_PWM_MID - 150;
        command.speed_limit = 4;
        command.reason = 2;
        command.override_steering = 1;
    } else if (pending && pending_age >= 200000u) {
        command.reason = 3; /* 换向一直无法确认，撤驱动而非无限保舵 */
    } else if (!valid || Navigation.action == NAV_UNKNOWN) {
        command.reason = 1;
    } else if (Navigation.action == NAV_OBSTACLE) {
        command.reason = 2;
        /* 宽墙可能是弯道外墙，只有远处宽墙+有效弯道允许低速接近。 */
        if (Navigation.obstacle_width > 250 && Navigation.obstacle_y > NAV_STOP_MM && bend)
            command.speed_limit = 4;
    } else if (pending) {
        command.speed_limit = 4;
        command.reason = 3;
    } else {
        command.speed_limit = bend ? 6 : 8;
        command.reason = bend ? 4 : 0;
    }
    return command;
}

#define RADAR_TIMEOUT_TICKS 50
static uint32_t __get_PRIMASK(void){return 0;}
static void __disable_irq(void){}
static void __set_PRIMASK(uint32_t x){}
static float Speed_mubiao=8;
static pid_type Speed_pid;
static float Diag_motor_integral,Diag_motor_prelimit;
static float moto_pwm;
static uint16_t daoche_flag;
#define TIM5 5
#define TIM2 2
#define TIM_IT_Update 1
#define SET 1
static int TIM_GetITStatus(int t,int f){return SET;}
static void TIM_ClearITPendingBit(int t,int f){}
static float motor_output;
static void Moto_Speed(float v){motor_output=v;}
static void TIM_SetCompare1(int t,uint16_t v){motor_output=v;}
static void Diag_MotorTick(uint16_t raw,uint8_t f,uint8_t p){}

volatile uint8_t Radar_drive_enabled = 1; /* 独立上电循迹兼容；上位机协商时drive 0 */
volatile float Speed_effective = 0;
static volatile float navigation_speed_limit = 0;
volatile uint8_t Radar_limit_reason = 1;
/* reason：0正常，1感知不足，2障碍，3换向确认，4弯道，5旧观测超时。 */
void Radar_SetSpeedLimit(float limit, uint8_t reason)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    navigation_speed_limit = limit;
    Radar_limit_reason = reason;
    __set_PRIMASK(mask);
}

volatile uint16_t Radar_age_ticks = 0;
volatile uint8_t Radar_started = 0;
volatile uint8_t Radar_stop_latched = 0;
volatile uint32_t Radar_timeout_count = 0;
volatile uint32_t Radar_invalid_inputs = 0;

void Radar_ControlCompleted(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* Never clear a timeout by receiving another scan. */
    if (!Radar_stop_latched) {
        Radar_age_ticks = 0;
        Radar_started = 1;
    }
    __set_PRIMASK(primask);
}

void Radar_GuardTick(void)
{
    if (Radar_started && !Radar_stop_latched) {
        if (Radar_age_ticks < RADAR_TIMEOUT_TICKS) Radar_age_ticks++;
        if (Radar_age_ticks >= RADAR_TIMEOUT_TICKS) {
            Radar_stop_latched = 1;
            Radar_timeout_count++;
        }
    }
}


void Speed_PID_Reset(pid_type *pid)
{
    pid->err_sum = 0;
    pid->err_l = 0;
    Diag_motor_integral = 0;
    Diag_motor_prelimit = 0;
}
float PID_realize(float speed_now, float speed_mubiao, pid_type *speed_pid)
{
    float output, next_sum;
    speed_pid->err = speed_mubiao - speed_now;
    next_sum = speed_pid->err_sum + speed_pid->err;
    if (next_sum > 200) next_sum = 200;
    if (next_sum < -200) next_sum = -200;
    output = speed_pid->kp * speed_pid->err + speed_pid->ki * next_sum +
             speed_pid->kd * (speed_pid->err - speed_pid->err_l);
    /* 饱和时只允许有助于退出饱和的积分，避免长期贴墙仍积累驱动力。 */
    if (!((output > 100 && speed_pid->err > 0) || (output < 0 && speed_pid->err < 0)))
        speed_pid->err_sum = next_sum;
    speed_pid->err_l = speed_pid->err;
    Diag_motor_integral = speed_pid->err_sum;
    Diag_motor_prelimit = output;
    if (output > 100) output = 100;
    if (output < 0) output = 0;
    return output;
}
void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET) {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

        uint8_t encoder_fresh=0,pi_fresh=0;
        extern uint16_t Encoder_cnt_temp;
        Radar_GuardTick();
        /* 无新有效控制时100ms开始限速，200ms撤驱动；500ms锁停仍保留。 */
        {
            float next = Speed_mubiao < navigation_speed_limit ? Speed_mubiao : navigation_speed_limit;
            if (Radar_age_ticks >= 10 && next > 4) next = 4;
            if (Radar_age_ticks >= 20) { next = 0; Radar_limit_reason = 5; }
            if (!Radar_drive_enabled || next < 0) next = 0;
            if (next < Speed_effective - 0.5f) Speed_PID_Reset(&Speed_pid);
            Speed_effective = next;
        }
        if (!Radar_started || Radar_stop_latched || Speed_effective <= 0) {
            /* Zero duty removes propulsion; it is not an active brake.
             * Skip PI so its integral cannot accumulate during inhibition. */
            Get_Encoder();encoder_fresh=1;
            Speed_PID_Reset(&Speed_pid);
            moto_pwm = 0;
            Moto_Speed(0);
        } else if (daoche_flag == 1) {
            TIM_SetCompare1(TIM2, (uint16_t)(100 * 0.5));  /* 50%制动,不足以驱动小车 */
        } else {
            Get_Encoder();encoder_fresh=1;
            moto_pwm = PID_realize(Speed_now, Speed_effective, &Speed_pid);pi_fresh=1;
            Moto_Speed(moto_pwm);
        }
        Diag_MotorTick(Encoder_cnt_temp,encoder_fresh,pi_fresh);
    }
}
static int checks;
#define CHECK(c) do{checks++;if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);return 1;}}while(0)
static _LEIDA_DATA scan[800], snapshot[800];
static _LEIDA_DATA_plane wall[10];
static void full_scan(uint32_t now,float distance){
    int i;LEIDA_speed_dps=2160;
    for(i=0;i<360;i++){scan[i].angle=(float)(359-i);scan[i].distance=distance;}
    LEIDA_ScanUpdate(scan,360,now);
    LEIDA_InspectNavigation(now);
}
int main(void){
    int i;uint16_t count,pwm;float x;TurnGuard guard={0};uint8_t held;
    pid_type controller={0};Midline_type line={1,0};
    controller.kp=.035f;controller.kp_3=.0395f;
    LEIDA_ScanReset();LEIDA_InspectNavigation(1000000);
    CHECK(Navigation.action==NAV_UNKNOWN && !Navigation.front_bins);
    full_scan(1000000,2500);
    CHECK(LEIDA_ScanSnapshot(snapshot,1000000)==360);
    CHECK(Navigation.action==NAV_CLEAR && Navigation.front_bins==41);
    LEIDA_InspectNavigation(1200001);
    CHECK(Navigation.action==NAV_UNKNOWN && LEIDA_ScanSnapshot(snapshot,1200001)==0);
    /* 微秒时间回绕仍按实际年龄限龄。 */
    LEIDA_ScanReset();full_scan(0xfffffff0u,2500);
    CHECK(LEIDA_ScanSnapshot(snapshot,0x1000u)==360);
    CHECK(LEIDA_ScanSnapshot(snapshot,0x40000u)==0);
    /* 窄锥桶和远背景共存，不能误报空旷。 */
    LEIDA_ScanReset();full_scan(2000000,2500);
    for(i=0;i<360;i++)if(scan[i].angle>=89 && scan[i].angle<=92)scan[i].distance=700;
    LEIDA_ScanUpdate(scan,360,2110000);LEIDA_InspectNavigation(2110000);
    CHECK(Navigation.action>=NAV_OBSTACLE);
    CHECK(Navigation.obstacle_y>690 && Navigation.obstacle_y<=700);
    CHECK(Navigation.obstacle_width<100);
    /* 100～200mm近障碍仍响应，且单点足够停车。 */
    for(i=0;i<360;i++)if(scan[i].angle==90)scan[i].distance=150;
    LEIDA_ScanUpdate(scan,360,2220000);LEIDA_InspectNavigation(2220000);
    CHECK(Navigation.action==NAV_OBSTACLE && Navigation.obstacle_y==150);
    /* 零回波会使原有“空旷”失效，缺测不能沿用旧远点。 */
    for(i=0;i<360;i++)if(scan[i].angle>=70 && scan[i].angle<=110)scan[i].distance=0;
    LEIDA_ScanUpdate(scan,360,2330000);LEIDA_InspectNavigation(2330000);
    CHECK(Navigation.action==NAV_UNKNOWN && Navigation.front_bins==0);
    LEIDA_speed_dps=633;LEIDA_ScanUpdate(scan,360,2440000);
    CHECK(!LEIDA_ScanSnapshot(snapshot,2440000));
    /* 10月4日坏参考：只有17cm，不能冒充1.2m。 */
    for(i=0;i<8;i++){wall[i]._x=-350;wall[i]._y=172+i;}
    CHECK(!wall_reference(wall,0,8,1,1200,&x));
    clock_us=1000000;
    Midline_PD_Calculate(wall,&controller,&line,144.5f,0,8,1);
    CHECK(!Servo_PD_valid && Servo_reject_reason==3);
    for(i=0;i<8;i++){wall[i]._x=-350+.1f*i*100;wall[i]._y=500+i*100;}
    CHECK(wall_reference(wall,0,8,1,1200,&x) && fabs(x+280)<.01f);
    CHECK(!wall_reference(wall,0,8,1,1600,&x));
    wall[3]._x=-900;CHECK(!wall_reference(wall,0,8,1,1200,&x));
    /* 反向一次不执行，重复缓存也不能当成第二次。 */
    guard.evidence_us=100000;
    CHECK(TurnGuard_Apply(&guard,3,1,0,1170,100000,&held)==1170);
    guard.evidence_us=200000;
    CHECK(TurnGuard_Apply(&guard,2,1,0,1696,200000,&held)==1170 && held);
    CHECK(TurnGuard_Apply(&guard,2,1,0,1696,310000,&held)==1170 && held);
    guard.evidence_us=400000;
    CHECK(TurnGuard_Apply(&guard,2,1,0,1696,400000,&held)==1696 && !held);
    /* 实际TIM5撤驱动，不只是修改遥测目标；锁停无法被新感知解除。 */
    Speed_pid.kp=8.5f;Speed_pid.ki=.505f;
    Radar_ControlCompleted();Radar_SetSpeedLimit(8,0);
    TIM5_IRQHandler();CHECK(Speed_effective==8 && motor_output>0);
    Radar_drive_enabled=0;TIM5_IRQHandler();CHECK(motor_output==0);
    Radar_drive_enabled=1;
    Radar_SetSpeedLimit(0,2);Speed_pid.err_sum=150;
    TIM5_IRQHandler();CHECK(motor_output==0 && Speed_pid.err_sum==0);
    Radar_SetSpeedLimit(8,0);Radar_age_ticks=19;
    TIM5_IRQHandler();CHECK(Speed_effective==0 && motor_output==0);
    Radar_age_ticks=49;TIM5_IRQHandler();CHECK(Radar_stop_latched);
    Radar_ControlCompleted();CHECK(Radar_stop_latched);
    printf("PASS %d navigation/speed checks (production C)\n",checks);
    return 0;
}
