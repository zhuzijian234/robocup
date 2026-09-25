/**
 * @brief M10P工作点云的几何处理：坐标转换、边界、中线、断点和前方墙面。
 * 字节接收由DMA.c负责；协议解析和整圈所有权由m10p.c负责。
 * m10p_vehicle.c生成LEIDA_DATA2后，本模块仅使用有效点数范围内的数据。
 * 坐标约定：0度向右、90度向前；距离和x/y均以毫米计。
 */

#include "LEIDA_DATA.h"
#include "ble_diag.h"
#include <string.h>
#include <float.h>
#include "centre_line.h"

_LEIDA_DATA LEIDA_DATA2[LEIDA_DATA_COUNTER];

/* 共享的几何/遥测接口：main从M10P整圈复制转速及真实点数。
 * M10P健康统计统一由M10P_stats提供。 */
uint16_t LEIDA_speed_dps = 0;
volatile uint16_t LEIDA_raw_count = 0;

/* ======================== HANDLE2: 极坐标转笛卡尔坐标 ======================== */

/**
 * @brief  极坐标 (角度, 距离) -> 笛卡尔坐标 (x, y)
 *
 * x = distance * cos(angle)
 * y = distance * sin(angle)
 */
void LEIDA_DATA_HANDLE2(_LEIDA_DATA_plane data[], _LEIDA_DATA arr[], u16 size)
{
    int i;
    for (i = 0; i < size; i++) {
        data[i]._x = arr[i].distance * arm_cos_f32(arr[i].angle * PI / 180);
        data[i]._y = arr[i].distance * arm_sin_f32(arr[i].angle * PI / 180);
    }
}


uint16_t valid_couter;

/* ======================== 边界/中线数据处理数组 ======================== */

_LEIDA_DATA LEIDA_DATA_LEFT[LEIDA_DATA_COUNTER / 2];
_LEIDA_DATA LEIDA_DATA_RIGHT[LEIDA_DATA_COUNTER / 2];
_LEIDA_DATA_plane LEIDA_DATA_LEFT_Plane[LEIDA_DATA_COUNTER / 2];
_LEIDA_DATA_plane LEIDA_DATA_RIGHT_Plane[LEIDA_DATA_COUNTER / 2];
_LEIDA_DATA_plane LEIDA_DATA_CENTER[LEIDA_DATA_COUNTER / 2];
uint16_t LEFT_cnt;
uint16_t RIGHT_cnt;
uint16_t LEFT_cnt_2;
uint16_t RIGHT_cnt_2;
uint16_t CENTER_cnt;
float zhongxian_junzhi;

/* Main-context scratch index; rebuilt for each caller, never caches pointers.
 * Lists retain every input point, so nearest/tie and width semantics are exact. */
static int16_t angle_heads[720], angle_next[LEIDA_DATA_COUNTER];
static void angle_index_build(const _LEIDA_DATA *points, uint16_t size)
{
    int i;
    for (i = 0; i < 720; ++i) angle_heads[i] = -1;
    for (i = (int)size - 1; i >= 0; --i) {
        int bin;
        float a = points[i].angle;
        angle_next[i] = -1;
        if (!(a >= 0.0f && a <= 360.0f)) continue;
        bin = (int)(a * 2.0f);
        if (bin == 720) bin = 0;
        angle_next[i] = angle_heads[bin];
        angle_heads[bin] = (int16_t)i;
    }
}
static int angle_nearest(const _LEIDA_DATA *points, float angle)
{
    int b, i, best = -1, center = (int)(angle * 2.0f);
    float nearest = 2001.0f;
    for (b = center - 2; b <= center + 2; ++b) {
        int bin = (b + 720) % 720;
        for (i = angle_heads[bin]; i >= 0; i = angle_next[i]) {
            float d = points[i].distance, diff = fabsf(points[i].angle - angle);
            if (diff > 180.0f) diff = 360.0f - diff;
            if (d >= 100.0f && d <= 2000.0f && diff <= LEIDA_ANGLE_piancha &&
                (d < nearest || (d == nearest && (best < 0 || i < best)))) {
                nearest = d; best = i;
            }
        }
    }
    return best;
}

/* ======================== HANDLE4: 中线点计算 ======================== */

/**
 * @brief  配对左右边界点，计算中线点
 *
 * 算法:
 *   1. 从右边界(0°)到左边界(180°)以0.6°步进扫描
 *   2. 在每个角度对(angle_right, angle_left)处，找到距离50-2000mm内
 *      满足角度容差的最近点
 *   3. 计算中点: (x_left + x_right)/2, (y_left + y_right)/2
 *   4. 排除x极端离群点（去掉最大最小各两个）
 *   5. 按相邻点x间距一致性滤除离群点
 *   6. 只保留y <= 800mm的近区点
 *几何含义：跑道左边界一个点 + 右边界对称角度的点 → 两点连线的中点就在跑道中线上。
 * 这是判断车现在偏离跑道中线多少的关键函数
 * @return 有效中线点数量
 */
uint16_t LEIDA_DATA_HANDLE4(_LEIDA_DATA_plane data_center[], _LEIDA_DATA arr[], u16 size)
{
    uint16_t i, n = 0;
    uint8_t used[(LEIDA_DATA_COUNTER + 7) / 8] = {0};
    int right, left;
    float a, b, x, y;
    zhongxian_junzhi = 0;
    if (size > LEIDA_DATA_COUNTER) return 0;
    angle_index_build(arr, size);
    for (a = LEIDA_ANGLE_RIGHT, b = LEIDA_ANGLE_LEFT;
         a <= LEIDA_ANGLE_RIGHT + LEIDA_ANGLE_yuliang; a += 0.6f, b -= 0.6f) {
        right = angle_nearest(arr, a);
        left = angle_nearest(arr, b);
        if (right < 0 || left < 0)
            continue;
        /* Overlapping query windows must not count one physical return twice
         * as independent support for a straight line or turn-exit decision. */
        if ((used[right / 8] & (1u << (right % 8))) ||
            (used[left / 8] & (1u << (left % 8)))) continue;
        x = (arr[right].distance * arm_cos_f32(arr[right].angle * PI / 180) + arr[left].distance * arm_cos_f32(arr[left].angle * PI / 180)) / 2;
        y = (arr[right].distance * arm_sin_f32(arr[right].angle * PI / 180) + arr[left].distance * arm_sin_f32(arr[left].angle * PI / 180)) / 2;
        if (y >= 0 && y <= 800 && n < LEIDA_DATA_COUNTER / 2) {
            used[right / 8] |= (uint8_t)(1u << (right % 8));
            used[left / 8] |= (uint8_t)(1u << (left % 8));
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

/* ======================== HANDLE6/7: 左右边界提取 ======================== */

float BLUE_ANGLE_LEFT_RIGHT = 90;

/**
 * @brief  提取左边界点 (180° -> 90°)
 *
 * 从LEIDA_ANGLE_LEFT(180°)向下扫描BLUE_ANGLE_LEFT_RIGHT度，
 * 每个角度步进LEIDA_ANGLE_resolution度，找到该角度下距离<=4000mm的最近有效点。
 * 实际上知识对每个角度找该角度的第一个有效点
 *
 * @return 左边界点数量
 */
static uint16_t boundary_extract(_LEIDA_DATA *out, _LEIDA_DATA *in, uint16_t count, uint8_t left)
{
    static int16_t nearest[181];
    uint16_t i, n = 0, span;
    if (count > LEIDA_DATA_COUNTER) count = LEIDA_DATA_COUNTER;
    span = BLUE_ANGLE_LEFT_RIGHT < 0 ? 0 : BLUE_ANGLE_LEFT_RIGHT > 90 ? 90 : (uint16_t)BLUE_ANGLE_LEFT_RIGHT;
    for (i = 0; i <= 180; ++i) nearest[i] = -1;
    for (i = 0; i < count; ++i) {
        int degree = (int)(in[i].angle + 0.5f);
        float diff;
        if (degree == 360) degree = 0;
        if (degree < 0 || degree > 180 || in[i].distance < 100 || in[i].distance > 4000) continue;
        diff = fabsf(in[i].angle - degree);
        if (diff > 180) diff = 360 - diff;
        if (diff <= 0.5f && (nearest[degree] < 0 || in[i].distance < in[nearest[degree]].distance))
            nearest[degree] = (int16_t)i;
    }
    for (i = 0; i <= span && n < LEIDA_DATA_COUNTER / 2; ++i) {
        int16_t ix = nearest[left ? 180-i : i];
        if (ix >= 0) out[n++] = in[ix];
    }
    return n;
}

uint16_t LEIDA_DATA_HANDLE6(_LEIDA_DATA out[], _LEIDA_DATA in[], u16 size)
{ return boundary_extract(out, in, size, 1); }

/**
 * @brief  提取右边界点 (0° -> 90°)
 *
 * 从0°向上扫描BLUE_ANGLE_LEFT_RIGHT度，处理0°/360°回绕。
 *
 * @return 右边界点数量
 */
uint16_t LEIDA_DATA_HANDLE7(_LEIDA_DATA out[], _LEIDA_DATA in[], u16 size)
{ return boundary_extract(out, in, size, 0); }

/* ======================== HANDLE8/9: 边界突变点检测（弯道入口） ======================== */

/**
 * @brief  检测左边界突变点（弯道入口）
 *
 * 使用6点滑动窗口。突变判定条件:
 *   |dist[i]   - dist[i+1]| >= 400mm  AND
 *   |dist[i-1] - dist[i+2]| >= 400mm  AND
 *   |dist[i-2] - dist[i+3]| >= 400mm
 *
 * 三重条件防止单点噪声误触发。
 * 仅考虑角度>=100°的点（左侧）。
 * 返回距离突变点较近一侧的y投影距离（1m以内）。
 *
 * @return 断点y坐标(mm), 无突变返回0
 */
uint16_t LEIDA_DATA_HANDLE8(_LEIDA_DATA arr[], u16 size)
{
    uint16_t i = 0;
    float min = 0;

    if (size < 8 || size > LEIDA_DATA_COUNTER / 2) return 0;
    for (i = 3; i + 4 < size; i++) {
        uint16_t k;
        uint8_t continuous = 1;
        for (k = i - 2; k < i + 3; ++k) {
            float gap = fabsf(arr[k+1].angle - arr[k].angle);
            if (gap < 0.25f || gap > 2.0f) continuous = 0;
        }
        if (!continuous) continue;
        if (arr[i].angle >= (180 - 80)) /* 仅左侧 (角度>=100°) */
            if (fabs(arr[i].distance - arr[i + 1].distance) >= fmaxf(400.0f, 0.25f * fminf(arr[i].distance, arr[i+1].distance)))
                if (fabs(arr[i - 1].distance - arr[i + 2].distance) >= fmaxf(400.0f, 0.25f * fminf(arr[i].distance, arr[i+1].distance)))
                    if (fabs(arr[i - 2].distance - arr[i + 3].distance) >= fmaxf(400.0f, 0.25f * fminf(arr[i].distance, arr[i+1].distance))) {
                        /* 取突变两侧中距离较近的点 */
                        if (arr[i].distance > arr[i + 2].distance)
                            min = arr[i + 2].distance * arm_sin_f32(arr[i + 2].angle * PI / 180);
                        else
                            min = arr[i - 1].distance * arm_sin_f32(arr[i - 1].angle * PI / 180);

                        if (min < 1000) /* 仅报告1米以内的突变 */
                            return min;
                    }
    }
    return 0;
}

/**
 * @brief  检测右边界突变点（同HANDLE8算法）
 *
 * 仅考虑角度<=80°的点（右侧）。
 *
 * @return 断点y坐标(mm), 无突变返回0
 */
uint16_t LEIDA_DATA_HANDLE9(_LEIDA_DATA arr[], u16 size)
{
    uint16_t i = 0;
    float min = 0;

    if (size < 8 || size > LEIDA_DATA_COUNTER / 2) return 0;
    for (i = 3; i + 4 < size; i++) {
        uint16_t k;
        uint8_t continuous = 1;
        for (k = i - 2; k < i + 3; ++k) {
            float gap = fabsf(arr[k+1].angle - arr[k].angle);
            if (gap < 0.25f || gap > 2.0f) continuous = 0;
        }
        if (!continuous) continue;
        if (arr[i].angle <= (80)) /* 仅右侧 (角度<=80°) */
            if (fabs(arr[i].distance - arr[i + 1].distance) >= fmaxf(400.0f, 0.25f * fminf(arr[i].distance, arr[i+1].distance)))
                if (fabs(arr[i - 1].distance - arr[i + 2].distance) >= fmaxf(400.0f, 0.25f * fminf(arr[i].distance, arr[i+1].distance)))
                    if (fabs(arr[i - 2].distance - arr[i + 3].distance) >= fmaxf(400.0f, 0.25f * fminf(arr[i].distance, arr[i+1].distance))) {
                        if (arr[i - 1].distance > arr[i + 2].distance)
                            min = arr[i + 2].distance * arm_sin_f32(arr[i + 2].angle * PI / 180);
                        else
                            min = arr[i - 1].distance * arm_sin_f32(arr[i - 1].angle * PI / 180);

                        if (min < 1000)
                            return min;
                    }
    }
    return 0;
}

/* ======================== LEIDA_Distance: 跑道宽度计算 ======================== */

/**
 * @brief  通过左右边界距离对计算跑道宽度
 *
 * 对每个左边界点(角度150°-210°)，找到对侧对应点(angle_left+180°)，
 * 将两者的距离求和。使用6元素最小值级联，取第6小的距离和作为跑道宽度。
 * 这样做可以排除噪点的影响。
 *
 * @return 第6小的左右距离和 (跑道宽度, mm)
 */
float LEIDA_Distance(_LEIDA_DATA data[], u16 size)
{
    uint16_t i;
    int j, bucket, center;

    float angle_left;
    float angle_right;

    float distance_temp;
    float distance_min = 5000;
    float distance_min_2 = 5000;
    float distance_min_3 = 5000;
    float distance_min_4 = 5000;
    float distance_min_5 = 5000;
    float distance_min_6 = 5000;

    if (size > LEIDA_DATA_COUNTER) return distance_min_6;
    angle_index_build(data, size);
    for (i = 0; i < size; i++) {
        if ((data[i].angle >= 180 - 30) && (data[i].angle <= 180 + 30)) { /* 左半: 150°~210° */
            angle_left = data[i].angle;
            /* 对侧角度: 左+180°=右, 归一化到 [0, 360) */
            angle_right = angle_left + 180;
            if (angle_right >= 360)
                angle_right -= 360;

            center = (int)(angle_right * 2.0f);
            for (bucket = center - 3; bucket <= center + 3; ++bucket) {
              for (j = angle_heads[(bucket + 720) % 720]; j >= 0; j = angle_next[j]) {
                /* 角度差（含环绕处理）：0°和359°物理上只差1°，需归一化到[-180,180] */
                float angle_diff = fabsf(data[j].angle - angle_right);
                if (angle_diff > 180)
                    angle_diff = 360 - angle_diff;
                if (angle_diff <= 1.0f) {
                    distance_temp = data[i].distance + data[j].distance;

                    /* 6元素最小级联: 逐个槽位比较，将distance_temp插入合适位置 */
                    if (distance_temp <= distance_min) {
                        distance_min_6 = distance_min_5;
                        distance_min_5 = distance_min_4;
                        distance_min_4 = distance_min_3;
                        distance_min_3 = distance_min_2;
                        distance_min_2 = distance_min;
                        distance_min = distance_temp;
                    } else if (distance_temp <= distance_min_2) {
                        distance_min_6 = distance_min_5;
                        distance_min_5 = distance_min_4;
                        distance_min_4 = distance_min_3;
                        distance_min_3 = distance_min_2;
                        distance_min_2 = distance_temp;
                    } else if (distance_temp <= distance_min_3) {
                        distance_min_6 = distance_min_5;
                        distance_min_5 = distance_min_4;
                        distance_min_4 = distance_min_3;
                        distance_min_3 = distance_temp;
                    } else if (distance_temp <= distance_min_4) {
                        distance_min_6 = distance_min_5;
                        distance_min_5 = distance_min_4;
                        distance_min_4 = distance_temp;
                    } else if (distance_temp <= distance_min_5) {
                        distance_min_6 = distance_min_5;
                        distance_min_5 = distance_temp;
                    } else if (distance_temp <= distance_min_6) {
                        distance_min_6 = distance_temp;
                    }
                }
              }
            }
        }
    }

    return distance_min_6;
}

/* ======================== HANDLE5: 前方路径扫描 ======================== */

_LEIDA_DATA_plane LEIDA_DATA_Forward[200];
_LEIDA_DATA_plane LEIDA_DATA_Forward_2[200];
_LEIDA_DATA_plane LEIDA_DATA_Forward_3[200];
uint16_t Forward_cnt;
uint16_t Forward_cnt_2;
uint16_t Forward_cnt_3;
uint32_t Forward_Distance;

/**
 * @brief  扫描前方路径 (70°-110°)，检测前方墙壁生成前视数据
 *
 * 算法:
 *   1. 空旷检测: 86°-94°范围内若有>=5个点距离>1500mm，
 *      认为前方空旷/有缺口，返回0
 *   2. 在70°-110°以1°步进扫描，每度找最近的符合条件点
 *      (距离>=200mm, y投影<=2000mm)
 *   3. 对前方点拟合直线
 *   4. 若斜率k>0，反转点序（确保由近到远排列）
 *   5. 按y间距一致性滤除离群点
 *
 * @return 有效前方路径点数量 (0=前方空旷)
 */
uint16_t LEIDA_DATA_HANDLE5(_LEIDA_DATA_plane data[], _LEIDA_DATA arr[], u16 size)
{
    uint16_t i, j;
    uint16_t counter = 0;
    uint16_t zhidao_counter = 0;
    Midline_type midline;
    float start_angle = 60;
    uint16_t cnt = 0, near_count = 0;

    float y_max = -4000;
    float y_max_r = -4000;
    float y_max_r_r = -4000;
    float y_min = 4000;
    float y_min_r = 4000;
    float y_min_r_r = 4000;
    float jiange = 0;

    if (size < 2 || size > LEIDA_DATA_COUNTER) return 0;
    /* 空旷检测: 正前方(86°-94°)有>=5个点距离>1.5m，说明前方空旷(有缺口) */
    for (i = 0; i < size; i++) {
        if ((fabs(arr[i].angle - 90) <= 4) && (arr[i].distance > 1500) && (arr[i].distance <= 4000)) {
            cnt++;
        }
        if (fabsf(arr[i].angle - 90) <= 4 && arr[i].distance >= 100 && arr[i].distance <= 1500)
            near_count++;

    }

    if (cnt >= 12 && !near_count) return 0;
    /* 前方有墙: 扫描前方弧70°-110°,得到前视数据 */
    for (start_angle = 70; start_angle <= 110; start_angle += 1) {
        for (i = 0; i < size; i++) {
            if ((fabs(arr[i].angle - start_angle) <= 1)                             /*找角度匹配的那个点*/
                && (arr[i].distance * arm_sin_f32(arr[i].angle * PI / 180) <= 2000) /*太近的是噪声(≥200),且只看前方2米以内*/
                && (arr[i].distance >= 100)) {
                if (counter >= 200) return counter;
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

/**
 * @brief  指定角度范围的前方路径扫描 [start_angle, end_angle]
 *
 * 与HANDLE5类似，但无障碍物检测和离群点滤除步骤。
 * 用于S弯检测时的侧前方扫描 (70°-90° 和 90°-110°)。
 *
 * @return 该角度范围内的有效点数量
 */
uint16_t LEIDA_DATA_HANDLE5_2(_LEIDA_DATA_plane data[], _LEIDA_DATA arr[], u16 size,
                              float start_angle, float end_angle)
{
    uint16_t i, j;
    uint16_t counter = 0;
    uint16_t zhidao_counter = 0;
    Midline_type midline;
    uint16_t cnt = 0, near_count = 0;

    float y_max = -4000;
    float y_max_r = -4000;
    float y_max_r_r = -4000;
    float y_min = 4000;
    float y_min_r = 4000;
    float y_min_r_r = 4000; /* 修复: 原为-4000, 同类初值错误(此处jiange未使用, 仅保持一致) */
    float jiange = 0;

    if (size < 2 || size > LEIDA_DATA_COUNTER || start_angle < 0 || end_angle > 180 || start_angle > end_angle) return 0;
    for (; start_angle <= end_angle; start_angle += 1) {
        for (i = 0; i < size; i++) {
            if ((fabs(arr[i].angle - start_angle) <= 1) && (arr[i].distance * arm_sin_f32(arr[i].angle * PI / 180) <= 2000) && (arr[i].distance >= 100)) {
                if (counter >= 200) return counter;
                data[counter]._x = arr[i].distance * arm_cos_f32(arr[i].angle * PI / 180);
                data[counter]._y = arr[i].distance * arm_sin_f32(arr[i].angle * PI / 180);
                counter++;
                break;
            }
        }
    }
    return counter;
}

/* ======================== 工具函数: 数组反转 ======================== */

/**
 * @brief  原地反转 _LEIDA_DATA_plane 数组 (_x和_y同时交换)
 */
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


/* ======================== HANDLE10: 边界离群点滤除 ======================== */

/**
 * @brief  通过x坐标间距滤除边界离群点
 *
 * 与HANDLE4中的滤除算法相同: 排除两侧最极端的两个点，
 * 然后按相邻点x间距在5倍平均间距范围内进行滤除。
 *
 * @return 滤除后的点数
 */
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

/* ======================== HANDLE11: 中线垂直度判断 ======================== */

float zhongxian_chuizhi;

/**
 * @brief  通过x坐标跨度判断中线是否垂直（是否垂直于x轴方向,即直道）
 *
 * 如果第3级x极值跨度 (x_max_r_r - x_min_r_r) < 10mm，
 * 则认为中线垂直，返回中点x坐标。否则返回0。
 *
 * @return 垂直中线的x坐标, 不垂直返回0
 */
uint8_t LEIDA_vertical_valid;
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
