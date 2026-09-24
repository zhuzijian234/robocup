
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
void Servo_ChangePwm(uint16_t value){timer3.CCR1=value;}
float Encoder_cnt,Speed_now;int16_t Encoder_cnt_arr[5];uint16_t Encoder_cnt_temp;
uint16_t LEIDA_DATA_HANDLE10(_LEIDA_DATA_plane *,u16);
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
uint16_t Midline_PD(_LEIDA_DATA_plane points[], pid_type *pid, Midline_type *line,
                    float mid, uint16_t start, uint16_t end, uint16_t mode)
{
    uint16_t i;
    uint32_t now = Diag_TimeUs(), dt = now - pd_previous_us;
    float e = 0, kp, kd, p, d, original_d, output, mag, x = 0, best = FLT_MAX, target;
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
    /* 换测量目标那一帧要不要清 err_l? 清 —— 否则 D 会把"上一帧的旧误差"当成
     * 跳变, 打一记假踢腿。但有一类例外: 进入转弯模式(1/2/3/4/8/9)时 err_l 保留,
     * 因为那一帧的误差跳变是车真的在拐, D 踢腿正是入弯要的力度。
     * 2026-09-22 实测: 原来无脑清, 入弯第一帧从打满(≈-324)掉到 -200, 表现为
     * "转弯力度小、反应迟钝"。
     * !dt / dt>250ms: 中间隔了太久(掉帧、切过测试模式), 旧误差没有参考意义, 重学。 */
    if (!pd_history_valid || !dt || dt > 250000u ||
        (mode != pd_previous_mode && !((mode == 1) || (mode == 2) || (mode == 3) || (mode == 4) || (mode == 8) || (mode == 9)))) {
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
    output = 10 * mid + p + d;
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
    Servo_ChangePwm((uint16_t)output);
    return (uint16_t)output;
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
void reverse(_LEIDA_DATA_plane a[], int sz)
{
    int left = 0;
    int right = sz - 1;
    float b;

    /* 反转 _x */
    while (left < right) {
        b = a[right]._x;
        a[right]._x = a[left]._x;
        a[left]._x = b;
        left++;
        right--;
    }

    /* 反转 _y */
    left = 0;
    right = sz - 1;
    while (left < right) {
        b = a[right]._y;
        a[right]._y = a[left]._y;
        a[left]._y = b;
        left++;
        right--;
    }
}
uint16_t LEIDA_DATA_HANDLE5(_LEIDA_DATA_plane data[], _LEIDA_DATA arr[], u16 size)
{
    uint16_t i, j;
    uint16_t counter = 0;
    uint16_t zhidao_counter = 0;
    Midline_type midline;
    float start_angle = 60;
    uint16_t cnt = 0;

    float y_max = -4000;
    float y_max_r = -4000;
    float y_max_r_r = -4000;
    float y_min = 4000;
    float y_min_r = 4000;
    float y_min_r_r = 4000;
    float jiange = 0;

    /* 空旷检测: 正前方(86°-94°)有>=5个点距离>1.5m，说明前方空旷(有缺口) */
    for (i = 0; i < size - 1; i++) {
        if ((fabs(arr[i].angle - 90) <= 4) && (arr[i].distance > 1500) && (arr[i].distance <= 4000)) {
            cnt++;
        }
        if (cnt >= 5)
            return 0; /* 前方无遮挡，不用前视数据 */
    }

    /* 前方有墙: 扫描前方弧70°-110°,得到前视数据 */
    for (start_angle = 70; start_angle <= 110; start_angle += 1) {
        for (i = 0; i < size - 1; i++) {
            if ((fabs(arr[i].angle - start_angle) <= 1)                             /*找角度匹配的那个点*/
                && (arr[i].distance * arm_sin_f32(arr[i].angle * PI / 180) <= 2000) /*太近的是噪声(≥200),且只看前方2米以内*/
                && (arr[i].distance >= 200)) {
                data[counter]._x = arr[i].distance * arm_cos_f32(arr[i].angle * PI / 180);
                data[counter]._y = arr[i].distance * arm_sin_f32(arr[i].angle * PI / 180);
                counter++;
                break;
            }
        }
    }
    /*得到data[0,counter-1]从70°到110°逐个方向取到的点*/
    /* 不足六点无法可靠地排除四个极值，保留观测交给调用方检查拟合。 */
    if (counter < 6)
        return counter;
    /* 拟合直线，若k>0则反转（确保近->远顺序） */
    Midline_fit(data, 1, counter - 1, &midline); /*掐头去尾(1-counter-2),去除边缘噪声*/
    if (midline.k > 0) {
        reverse(data, counter);
    } /*为了让650行的滤除循环在k>0,即y[i]<y[i+1]时有意义*/

    /* 找y极值 (最大/次大/次次大, 最小/次小/次次小) */
    for (i = 0; i < counter; i++) {
        if (data[i]._y > y_max)
            y_max = data[i]._y;
        if (data[i]._y < y_min)
            y_min = data[i]._y;
    }
    for (i = 0; i < counter; i++) {
        if ((data[i]._y > y_max_r) && (data[i]._y != y_max))
            y_max_r = data[i]._y;
        if ((data[i]._y < y_min_r) && (data[i]._y != y_min))
            y_min_r = data[i]._y;
    }
    for (i = 0; i < counter; i++) {
        if ((data[i]._y > y_max_r_r) && (data[i]._y != y_max) && (data[i]._y != y_max_r))
            y_max_r_r = data[i]._y;
        if ((data[i]._y < y_min_r_r) && (data[i]._y != y_min) && (data[i]._y != y_min_r))
            y_min_r_r = data[i]._y;
    }

    /* 水平墙或重复高度可能没有三个不同的极值；不可用哨兵值计算阈值。
     * 中间跨度为零时也保留点，避免把微小测量差全部判成离群点。 */
    if (y_max_r_r <= y_min_r_r)
        return counter;
    jiange = (y_max_r_r - y_min_r_r) / (counter - 4); /* 排除4个极值点 */

    /* 按y间距一致性滤除离群点 */
    for (i = 0, j = 0; i < counter - 1; i++) {
        if (fabs(data[i]._y - data[i + 1]._y) <= 5 * jiange) {
            data[j]._x = data[i]._x;
            data[j]._y = data[i]._y;
            j++;
        }
    }

    return j;
}
#define BLE_MODE_HOLD 10
static uint16_t pid_select, pid_select_last, pid_select_last_last;
static uint16_t state_left_cnt, state_right_cnt, state_left_cnt_2, state_right_cnt_2;
static uint16_t RIGHT_duandian, LEFT_duandian, duandian_DIStance=550, duandian_distance=450;
static uint16_t LEFT_cnt=20, RIGHT_cnt=20, Forward_cnt=20, ref_start, ref_end, telemetry_mode;
static uint8_t forward_fit_ok, danbian_flag;
static float servo_midpwm=144.5f;
static pid_type Servo_pd;
static Midline_type Midline, Midline_forward, Midline_forward_2, Midline_forward_3;
static _LEIDA_DATA_plane LEIDA_DATA_LEFT_Plane[400], LEIDA_DATA_RIGHT_Plane[400], LEIDA_DATA_Forward[400];
static _LEIDA_DATA LEIDA_DATA_LEFT[800], LEIDA_DATA_RIGHT[800];
static int calls, selected;
static void convert(_LEIDA_DATA_plane *p,_LEIDA_DATA *a,int n){}
static uint16_t keep(_LEIDA_DATA_plane *p,uint16_t n){return n;}
static uint16_t command(_LEIDA_DATA_plane *p,pid_type *pid,Midline_type *l,float mid,uint16_t s,uint16_t e,uint16_t mode){
    calls++;selected=mode;Servo_PD_valid=1;return 1445;
}
#define LEIDA_DATA_HANDLE2 convert
#define LEIDA_DATA_HANDLE10 keep
#define Midline_PD command
static void step(void){
    telemetry_mode=11;Servo_PD_valid=0;
            pid_select_last_last = pid_select_last;
            pid_select_last = pid_select;
            if ((RIGHT_duandian > 0 && RIGHT_duandian < duandian_DIStance) ||
                (LEFT_duandian > 0 && LEFT_duandian < duandian_DIStance)) {
                uint8_t right = (RIGHT_duandian > 0 && RIGHT_duandian < duandian_DIStance);
                uint16_t breakpoint = right ? RIGHT_duandian : LEFT_duandian;
                /* 新弯道观测优先，取消上一弯尚未执行的出弯补打。 */
                state_left_cnt = state_right_cnt = 0;
                state_left_cnt_2 = state_right_cnt_2 = 0;
                if (forward_fit_ok && breakpoint < duandian_distance && fabs(Midline_forward.k) < 0.35f) {
                    pid_select = right ? 3 : 4;
                    (void)Midline_PD(LEIDA_DATA_Forward, &Servo_pd, &Midline_forward, servo_midpwm,
                                     (uint16_t)(Forward_cnt * 0.1f), (uint16_t)(Forward_cnt * 0.9f), pid_select);
                } else if (forward_fit_ok && breakpoint < duandian_distance && fabs(Midline_forward.k) < 0.7f &&
                           danbian_flag && (fabs(Midline_forward_2.k) < 0.25f || fabs(Midline_forward_3.k) < 0.25f)) {
                    pid_select = right ? 8 : 9;
                    (void)Midline_PD(LEIDA_DATA_Forward, &Servo_pd, &Midline_forward, servo_midpwm,
                                     (uint16_t)(Forward_cnt * 0.1f), (uint16_t)(Forward_cnt * 0.9f), pid_select);
                } else {
                    _LEIDA_DATA_plane *boundary = right ? LEIDA_DATA_LEFT_Plane : LEIDA_DATA_RIGHT_Plane;
                    uint16_t count = right ? LEFT_cnt : RIGHT_cnt;
                    if (right && (pid_select_last == 3) && (pid_select_last_last == 3))
                        state_left_cnt_2 = 1;
                    if (!right && (pid_select_last == 4) && (pid_select_last_last == 4))
                        state_right_cnt_2 = 1;
                    /* 即使本帧 HOLD，也消耗掉“大弯连续两帧”的历史。
                     * 否则 pid_select 留在 3/4，下一帧会重新触发 HOLD。 */
                    pid_select = right ? 1 : 2;
                    if (state_left_cnt_2 > 0 || state_right_cnt_2 > 0) {
                        /* 前方拟合刚丢(减速带/车头扫过弯心): 这一帧只保舵机不动,
                         * 别拿一帧残缺的边界点去重算, 否则车头会抖一下。 */
                        telemetry_mode = BLE_MODE_HOLD;
                        Servo_ChangePwm((uint16_t)TIM3->CCR1);
                        Servo_PD_valid = 1;
                        if (state_left_cnt_2 > 0)
                            state_left_cnt_2--;
                        else
                            state_right_cnt_2--;
                    } else {
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
                        (void)Midline_PD(boundary, &Servo_pd, &Midline, servo_midpwm, ref_start, ref_end, pid_select);
                    }
                }

}

}
#undef Midline_PD
#undef LEIDA_DATA_HANDLE10

static int checks;
#define CHECK(c) do{checks++;if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);return 1;}}while(0)
int main(void){
    int direction,i;uint16_t n;
    _LEIDA_DATA scan[50];_LEIDA_DATA_plane wall[400];
    for(direction=0;direction<2;direction++){
        RIGHT_duandian=direction?0:300;LEFT_duandian=direction?300:0;
        pid_select=pid_select_last=direction?4:3;
        forward_fit_ok=0;calls=0;
        step();CHECK(telemetry_mode==10 && calls==0);
        for(i=0;i<4;i++){step();CHECK(calls==i+1 && selected==(direction?2:1));}
        /* Opposite turn must act immediately, not hold the previous direction. */
        pid_select=pid_select_last=direction?3:4;calls=0;
        step();CHECK(calls==1 && selected==(direction?2:1));
        /* A newly observed bend cancels a pending forced turn from the old bend. */
        state_left_cnt=state_right_cnt=2;forward_fit_ok=1;Midline_forward.k=.2f;
        step();CHECK(!state_left_cnt && !state_right_cnt);
        CHECK(selected==(direction?4:3));
    }
    memset(scan,0,sizeof scan);
    CHECK(LEIDA_DATA_HANDLE5(wall,scan,0)==0);
    CHECK(LEIDA_DATA_HANDLE5(wall,scan,1)==0);
    /* One accepted scan measurement creates a sparse window: keep its points. */
    scan[0].angle=80;scan[0].distance=600;
    n=LEIDA_DATA_HANDLE5(wall,scan,2);CHECK(n>0 && n<6);
    /* A wall perpendicular to the vehicle has nearly constant y. */
    for(i=0;i<41;i++){
        scan[i].angle=70.0f+i;
        scan[i].distance=700.0f/sinf(scan[i].angle*PI/180);
    }
    n=LEIDA_DATA_HANDLE5(wall,scan,42);CHECK(n>=30);
    CHECK(Midline_fit(wall,0,n,&Midline_forward));
    CHECK(fabs(Midline_forward.k)<.01f);
    printf("PASS %d turn/filter regression checks (production C, mocked hardware)\n",checks);
    return 0;
}
