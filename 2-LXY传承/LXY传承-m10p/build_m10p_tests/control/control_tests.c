
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
uint8_t LEIDA_vertical_valid,Servo_PD_valid;
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
#define TURN_EXIT_MIN_US 100000u
#define TURN_EXIT_ERROR_MM 100.0f
#define TURN_ENTRY_PWM 75.0f /* 入弯附加量同时不超过本帧|P|，不放大小误差噪声 */
#define TURN_MIN_OFFSET 20  /* 接近中位的候选不能成为弯道保持依据 */
#define TURN_RETRACT_PWM 60 /* 同模式单次明显收舵，需要出弯确认或期限到达 */
typedef struct {
    uint32_t observed_us, straight_since_us;
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
    uint16_t i, n = 0;
    int right, left;
    float a, b, dr, dl, diff, x, y;
    zhongxian_junzhi = 0;
    for (a = LEIDA_ANGLE_RIGHT, b = LEIDA_ANGLE_LEFT;
         a <= LEIDA_ANGLE_RIGHT + LEIDA_ANGLE_yuliang; a += 0.6f, b -= 0.6f) {
        /* Each angular pair owns fresh indices; array index zero is valid. */
        right = left = -1;
        dr = dl = 2001.0f;
        for (i = 0; i < size; i++) {
            if (arr[i].distance < 100 || arr[i].distance > 2000)
                continue;
            diff = fabs(arr[i].angle - a);
            if (diff > 180)
                diff = 360 - diff;
            if (diff <= LEIDA_ANGLE_piancha && arr[i].distance < dr) {
                right = i;
                dr = arr[i].distance;
            }
            diff = fabs(arr[i].angle - b);
            if (diff > 180)
                diff = 360 - diff;
            if (diff <= LEIDA_ANGLE_piancha && arr[i].distance < dl) {
                left = i;
                dl = arr[i].distance;
            }
        }
        if (right < 0 || left < 0)
            continue;
        x = (arr[right].distance * arm_cos_f32(arr[right].angle * PI / 180) + arr[left].distance * arm_cos_f32(arr[left].angle * PI / 180)) / 2;
        y = (arr[right].distance * arm_sin_f32(arr[right].angle * PI / 180) + arr[left].distance * arm_sin_f32(arr[left].angle * PI / 180)) / 2;
        if (y >= 0 && y <= 800 && n < LEIDA_DATA_COUNTER / 2) {
            data_center[n]._x = x;
            data_center[n++]._y = y;
        }
    }
    n = LEIDA_DATA_HANDLE10(data_center, n);
    for (i = 0; i < n; i++)
        zhongxian_junzhi += data_center[i]._x;
    if (n)
        zhongxian_junzhi /= n;
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
uint16_t Midline_PD_Calculate(_LEIDA_DATA_plane points[], pid_type *pid, Midline_type *line,
                    float mid, uint16_t start, uint16_t end, uint16_t mode)
{
    uint16_t i;
    uint32_t now = Diag_TimeUs(), dt = now - pd_previous_us;
    float e = 0, kp, kd, p, d, original_d, output, mag, x = 0, best = FLT_MAX, target, entry = 0;
    Servo_PD_valid = 0;
    Diag_detail_u[6] = pd_previous_mode;
    Diag_detail_u[7] = start;
    Diag_detail_u[8] = end;
    Diag_detail_u[9] = end > start ? end - start : 0;
    Diag_detail_f[13] = pid->err_l;
    Diag_detail_f[11] = line->k;
    Diag_detail_f[12] = line->b;
    if (mode > 9 || end <= start || end > LEIDA_DATA_COUNTER / 2) {
        if (mode == 1 || mode == 2)
            Diag_detail_u[4] |= 8;
        return pd_reject();
    }
    if (mode == 0) {
        if (!(fabs(line->k) > 0.1f && fabs(line->k) <= FLT_MAX && fabs(line->b) <= FLT_MAX))
            return pd_reject();
        target = BLUE_Y_STRA_SEL == 1 ? BLUE_Y_STRA : points[end - 1]._y;
        e = -((target - line->b) / line->k - 50);
        if (BLUE_Y_STRA_SEL != 1) {
            if (e > 200)
                e = 200;
            if (e < -200)
                e = -200;
        }
    } else if (mode == 1 || mode == 2) {
        target = mode == 1 ? BLUE_Y_RIGHT : BLUE_Y_LEFT;
        for (i = start; i < end; i++) {
            float dy = fabs(points[i]._y - target);
            if (dy < best && fabs(points[i]._x) <= FLT_MAX) {
                best = dy;
                x = points[i]._x;
                Diag_detail_u[4] |= 2;
                Diag_detail_f[8] = x;
                Diag_detail_f[9] = points[i]._y;
                Diag_detail_f[10] = dy;
            }
        }
        if (!(Diag_detail_u[4] & 2)) {
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
    if (straight && !state->straight_frames) state->straight_since_us = now;
    state->straight_frames = straight ? (state->straight_frames < 255 ? state->straight_frames + 1 : 255) : 0;
    if (state->straight_frames >= TURN_EXIT_FRAMES && (uint32_t)(now-state->straight_since_us) >= TURN_EXIT_MIN_US) {
        state->active = 0;
        state->straight_frames = 0;
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
static int checks;
#define CHECK(c) do{checks++;if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);return 1;}}while(0)
static uint8_t packet[47]={0x54,0x2c,0x68,0x08,0xab,0x7e,0xe0,0x00,0xe4,0xdc,0x00,0xe2,0xd9,0x00,0xe5,0xd5,0x00,0xe3,0xd3,0x00,0xe4,0xd0,0x00,0xe9,0xcd,0x00,0xe4,0xca,0x00,0xe2,0xc7,0x00,0xe9,0xc5,0x00,0xe5,0xc2,0x00,0xe5,0xc0,0x00,0xe5,0xbe,0x82,0x3a,0x1a,0x50};
static _LEIDA_DATA points[800];
static _LEIDA_DATA_plane plane[400];
static pid_type pid;
static int deferred;
static void begin(void){memset(Diag_detail_u,0,sizeof Diag_detail_u);memset(Diag_detail_f,0,sizeof Diag_detail_f);clock_us+=115000;}
static uint16_t drive(uint16_t mode,float error){
    Midline_type line={1,650+error};
    begin();plane[0]._y=plane[1]._y=700;
    plane[0]._x=plane[1]._x=mode==1?-error-400:mode==2?400-error:50-error;
    if(mode==3 || mode==4 || mode==8 || mode==9)line.k=fabs(error)>0?175.0f/fabs(error):INFINITY;
    LEIDA_vertical_valid=1;zhongxian_chuizhi=50-error;
    return deferred ? Midline_PD_Calculate(plane,&pid,&line,144.5f,0,2,mode) :
                      Midline_PD(plane,&pid,&line,144.5f,0,2,mode);
}
static int run(void){
    unsigned split,k,total;uint8_t stream[2000],bad[47];Midline_type line;
    uint16_t output;float before;
    TurnGuard guard={0};uint8_t held;unsigned writes;
    pid.kp=.035f;pid.kd=.035f;pid.kp_2=.040f;pid.kd_2=.022f;pid.kp_3=.0395f;pid.kd_3=.020f;
    CHECK(Diag_RadarPacket(packet));
    for(split=1;split<47;split++){
        LEIDA_ParserReset();memset(points,0xff,sizeof points);
        CHECK(LEIDA_DATA_HANDLE1(points,packet,(uint16_t)split)==0);
        CHECK(points[799].distance==0);
        CHECK(LEIDA_DATA_HANDLE1(points,packet+split,(uint16_t)(47-split))==12);
        CHECK(LEIDA_speed_dps==2152);
        CHECK(fabs(points[11].angle-115.3f)<.001f);
    }
    memcpy(bad,packet,47);bad[6]^=1;memcpy(stream,bad,47);memcpy(stream+47,packet,47);
    LEIDA_ParserReset();CHECK(LEIDA_DATA_HANDLE1(points,stream,94)==12);
    CHECK(points[12].distance==0);
    /* Byte insertion and deletion recover later complete packets. */
    memcpy(stream,packet,20);stream[20]=0x11;memcpy(stream+21,packet+20,27);memcpy(stream+48,packet,47);
    LEIDA_ParserReset();CHECK(LEIDA_DATA_HANDLE1(points,stream,95)==12);
    memcpy(stream,packet,20);memcpy(stream+20,packet+21,26);memcpy(stream+46,packet,47);
    LEIDA_ParserReset();CHECK(LEIDA_DATA_HANDLE1(points,stream,93)==12);
    for(k=0;k<40;k++)memcpy(stream+k*47,packet,47);
    LEIDA_ParserReset();total=LEIDA_DATA_HANDLE1(points,stream,1798);total+=LEIDA_DATA_HANDLE1(points,stream+1798,82);
    CHECK(total==480); /* split DMA boundary loses no accepted slots */
    LEIDA_ParserReset();CHECK(!LEIDA_DATA_HANDLE1(points,packet,20));LEIDA_ParserReset();CHECK(!LEIDA_DATA_HANDLE1(points,packet+20,27));
    CHECK(LEIDA_DATA_HANDLE1(points,packet,47)==12);
    CHECK(LEIDA_DATA_HANDLE10(plane,0)==0);
    for(k=0;k<8;k++){plane[k]._x=50;plane[k]._y=(float)k*100;}
    CHECK(LEIDA_DATA_HANDLE10(plane,8)==8);
    CHECK(LEIDA_DATA_HANDLE11(plane,0,8)==50 && LEIDA_vertical_valid);
    for(k=0;k<4;k++)plane[k]._x=(float)k*100;
    CHECK(LEIDA_DATA_HANDLE11(plane,0,4)==0 && !LEIDA_vertical_valid);
    CHECK(LEIDA_DATA_HANDLE11(plane,0,0)==0 && !LEIDA_vertical_valid);
    for(k=0;k<4;k++)plane[k]._x=0;
    CHECK(LEIDA_DATA_HANDLE11(plane,0,4)==0 && LEIDA_vertical_valid);
    points[0].angle=30;points[0].distance=400;points[1].angle=150;points[1].distance=400;
    CHECK(LEIDA_DATA_HANDLE4(plane,points,2)>0); /* index zero usable */
    points[0].angle=0;points[1].angle=179;
    CHECK(LEIDA_DATA_HANDLE4(plane,points,2)==0); /* no cross-angle stale pairing */
    CHECK(!Midline_fit(plane,0,0,&line));
    plane[0]._x=plane[1]._x=20;CHECK(!Midline_fit(plane,0,2,&line));
    before=timer3.CCR1;begin();Midline_PD(plane,&pid,&line,144.5f,0,0,2);
    CHECK(!Servo_PD_valid && timer3.CCR1==before && (Diag_detail_u[4]&512));
    Midline_PD_Reset();output=drive(0,-200);CHECK(output==1375);
    output=drive(5,247.7f);CHECK(output>=1531 && output<=1532 && Diag_detail_f[3]==0);
    output=drive(0,-102.4f);CHECK(output>=1409 && output<=1410 && Diag_detail_f[3]==0);
    Midline_PD_Reset();drive(5,421.156f);output=drive(5,71.5988f);
    CHECK(output>=1445 && fabs(Diag_detail_f[3])<=60); /* old PWM was 1202 */
    Midline_PD_Reset();drive(2,500);output=drive(2,-500);CHECK(output>=1445 && (Diag_detail_u[4]&1024));
    output=drive(1,100);CHECK(output<=1445);
    output=drive(4,500);CHECK(output>=1445);
    before=timer3.CCR1;output=drive(4,0);CHECK(!Servo_PD_valid && output==before);
    /* A reset/invalid observation does not leave a stale D kick. */
    Midline_PD_Reset();output=drive(7,-20);CHECK(Diag_detail_f[3]==0);
    /* Real telemetry: large-left e=500 -> side-wall e=82.391 is not a derivative. */
    drive(4,500);output=drive(2,82.391f);
    CHECK(output>=1477 && output<=1478 && Diag_detail_f[3]==0);
    drive(3,-500);output=drive(1,-82.391f);
    CHECK(output>=1412 && output<=1413 && Diag_detail_f[3]==0);
    output=drive(1,-182.391f);CHECK(Diag_detail_f[3]<-19.9f); /* same-mode D retained */
    output=drive(2,200);CHECK(Diag_detail_f[3]==0); /* opposite direction */
    line.k=1;line.b=650;before=timer3.CCR1;
    begin();Midline_PD_Calculate(plane,&pid,&line,144.5f,0,2,0);
    CHECK(Servo_PD_valid && timer3.CCR1==before); /* candidate must not write hardware */
    /* Replay observed left-turn errors using real PD + real guard, not mocked PD. */
    deferred=1;drive(0,0);writes=servo_writes;
    output=drive(2,284.289f);CHECK(output==1632 && Diag_detail_f[3]==0);
    CHECK(TurnGuard_Apply(&guard,2,1,0,output,clock_us,&held)==1632 && !held);
    output=drive(2,-49.897f);CHECK(output==1445);
    CHECK(TurnGuard_Apply(&guard,2,1,0,output,clock_us,&held)==1632 && held);
    CHECK(guard.pwm==1632); /* 14:45:03 used to overwrite the anchor with 1445 */
    output=drive(0,50);
    CHECK(TurnGuard_Apply(&guard,0,1,1,output,clock_us,&held)==1632 && held);
    output=drive(0,50);
    CHECK(TurnGuard_Apply(&guard,0,1,1,output,clock_us,&held)==1462 && !held);
    memset(&guard,0,sizeof guard);
    output=drive(4,500);CHECK(output==1720 && Diag_detail_f[3]==0);
    CHECK(TurnGuard_Apply(&guard,4,1,0,output,clock_us,&held)==1720 && !held);
    for(k=0;k<3;k++){
        output=drive(4,500);CHECK(output==1645); /* entry compensation not repeated */
        CHECK(TurnGuard_Apply(&guard,4,1,0,output,clock_us,&held)==1720 && held);
    }
    output=drive(4,500);
    CHECK(TurnGuard_Apply(&guard,4,1,0,output,clock_us,&held)==1645 && !held);
    drive(0,0);output=drive(2,83.583f);CHECK(output==1511); /* small entry bounded by |P| */
    drive(0,0);output=drive(2,-50);CHECK(output==1445); /* contradictory error never boosted */
    drive(0,0);output=drive(3,-500);CHECK(output==1170);
    CHECK(servo_writes==writes);deferred=0; /* whole candidate replay leaves hardware untouched */
    /* Encoder forward, reverse and modulo wrap: no huge unsigned speed. */
    for(k=0;k<5;k++){encoder_counter+=28;Get_Encoder();}
    CHECK(fabs(Speed_now-10.181818f)<.001f);
    for(k=0;k<5;k++){encoder_counter-=1;Get_Encoder();}
    CHECK(Encoder_cnt_temp==65535 && Speed_now<0 && Speed_now> -1);
    for(k=0;k<5;k++){encoder_counter=(uint16_t)(encoder_counter+30000);Get_Encoder();}
    CHECK(Encoder_cnt_temp==30000);
    printf("PASS %d checks (production C, mocked hardware)\n",checks);
    return 0;
}
int main(void){return run();}
