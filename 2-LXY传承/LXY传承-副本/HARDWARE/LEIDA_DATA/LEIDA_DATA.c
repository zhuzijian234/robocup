/**
 * @file    LEIDA_DATA.c
 * @brief   激光雷达数据处理流水线 — 解析、转换、边界提取、突变检测、中线计算、前方扫描
 *
 * 本模块是循线小车的感知核心，将雷达原始串口数据帧转换为结构化几何信息，
 * 供转向和速度控制器使用。
 *
 * ======================== 数据流水线 ========================
 *
 *   USART6 DMA缓冲区 (原始字节)
 *        |
 *        v
 *   HANDLE1: 解析47字节数据包 -> 极坐标 (角度, 距离) LEIDA_DATA[]
 *        |
 *        v
 *   HANDLE3_2: 筛选有效点 (距离>=100mm) -> LEIDA_DATA2[]
 *        |
 *        +---> HANDLE6/7: 提取左/右边界点
 *        |         |
 *        |         +---> HANDLE2: 极坐标 -> 笛卡尔坐标转换
 *        |         +---> HANDLE8/9: 边界突变检测（弯道入口）
 *        |         +---> HANDLE10: 离群点滤除
 *        |
 *        +---> HANDLE4: 左右配对 -> 中线中点
 *        |         |
 *        |         +---> HANDLE11: 中线垂直度判断（直道检测）
 *        |
 *        +---> HANDLE5: 前方路径扫描 (70-110度)，障碍物检测
 *                  |
 *                  +---> HANDLE5_2: 侧前方扫描（S弯检测用）
 *
 *   LEIDA_Distance: 左右边界距离对 -> 跑道宽度
 */

#include "LEIDA_DATA.h"
#include "ble_diag.h"
#include <string.h>
#include <float.h>
#include "centre_line.h"

_LEIDA_DATA LEIDA_DATA[LEIDA_DATA_COUNTER];
_LEIDA_DATA LEIDA_DATA2[LEIDA_DATA_COUNTER];

u8 tiaoshi = 0; /* 调试标志 */

/* 雷达实时转速 (度/秒), 每帧由HANDLE1从数据包Byte2~3读出
 * 6Hz → 2160, 8Hz → 2880 (手册示例帧: 68 08 → 0x0868 = 2152 ≈ 5.98Hz)
 * 手册提示: 电机个体差异, 占空比设典型值时实际转速有差异, 需依此字段闭环 */
uint16_t LEIDA_speed_dps = 0;
volatile uint32_t LEIDA_parse_calls = 0;
volatile uint32_t LEIDA_sync_failures = 0;
volatile uint32_t LEIDA_short_inputs = 0;
volatile uint32_t LEIDA_missing_packets = 0;
volatile uint16_t LEIDA_raw_count = 0;

/* ======================== HANDLE1: 原始串口帧解析 ======================== */

/**
 * @brief  将雷达原始串口字节流解析为极坐标数据点
 *
 * 雷达数据包格式 (LD14P, 每包47字节, 见LD14P开发手册§4):
 *   Byte 0:     帧头 (0x54)
 *   Byte 1:     VerLen (0x2C)
 *   Byte 2-3:   转速 (低字节在前), 单位 度/秒
 *   Byte 4-5:   起始角度 (低字节在前) / 100 = 度
 *   Byte 6-7:   数据点0距离 (低字节在前) mm 8:信号强度,每个数据点3字节
 *   Byte 9-10:  数据点1距离 11:信号强度
 *   ...         每包12个数据点, 每点3字节
 *   Byte 42-43: 结束角度 (低字节在前) / 100 = 度
 *   Byte 44-45: 时间戳 (ms, 0~30000循环)
 *   Byte 46:    CRC-8 (本函数未校验)
 *
 * 解析后角度旋转+90度: 0°=右侧(x+), 90°=前方(y+), 180°=左侧(x-)
 *
 * 本帧未写出的槽位(帧头缺失的整包、块尾未覆盖区间)会被清零, 避免上一帧的旧点残留;
 * 所有未更新槽位（包括同步失败）清零；distance==0表示无可用距离，
 * 可能是清零占位或雷达原始零距离，不能仅靠数值区分二者。
 *
 * @return 成功=本次成功解析出的数据点数(每个命中帧头+12), 0=未找到有效帧头
 */
/* 流式收包: 一包 47 字节, 但 DMA 一块是 1798 字节(= 47*38+12), 包会跨块 —— 所以
 * 没凑满 47 的尾巴必须留到下一次调用, 不能每块都从头找。lidar_pending = 已缓冲字节数。 */
static uint8_t lidar_packet[47];
static uint16_t lidar_pending;
/* 丢块(seq 不连续)后调用: 扔掉半包, 强制下次从帧头重新同步 */
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

/* ======================== HANDLE3: 有效点筛选 ======================== */

uint16_t valid_couter;

/**
 * @brief  筛选[LEIDA_ANGLE_RIGHT, LEIDA_ANGLE_LEFT]范围内距离非零的点
 * @return 有效点个数
 */
uint16_t LEIDA_DATA_HANDLE3(_LEIDA_DATA data[], _LEIDA_DATA arr[], u16 size)
{
    int i, j;
    j = 0;
    for (i = 0; i < size; i++) { /*掐头去尾10个噪点*/
        if (arr[i].distance != 0) {
            if ((arr[i].angle >= LEIDA_ANGLE_RIGHT) && (arr[i].angle <= LEIDA_ANGLE_LEFT)) {
                data[j].angle = arr[i].angle;
                data[j].distance = arr[i].distance;
                j++;
            }
        }
    }
    return j;
}

/**
 * @brief  筛选所有距离>=100mm的有效点
 * @return 有效点个数
 */
uint16_t LEIDA_DATA_HANDLE3_2(_LEIDA_DATA data[], _LEIDA_DATA arr[], u16 size)
{
    int i, j;
    j = 0;
    for (i = 0; i < size; i++) {
        if ((arr[i].distance >= 100)) {
            data[j].angle = arr[i].angle;
            data[j].distance = arr[i].distance;
            j++;
        }
    }
    return j;
}

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
uint16_t LEIDA_DATA_HANDLE6(_LEIDA_DATA data_left[], _LEIDA_DATA arr[], u16 size)
{
    int i = 0;
    int left_cnt = 0;
    float angle_left;
    int angle_left_cnt = -1;
    int flag = 0;

    for (angle_left = LEIDA_ANGLE_LEFT;
         angle_left >= LEIDA_ANGLE_LEFT - BLUE_ANGLE_LEFT_RIGHT;
         angle_left -= LEIDA_ANGLE_resolution) {
        for (i = 0; i < size; i++) {
            if ((fabs(arr[i].angle - angle_left) <= LEIDA_ANGLE_piancha) && (arr[i].distance <= 4000)) {
                angle_left_cnt = i;
            }

            if ((angle_left_cnt != -1)) {
                left_cnt++;
                flag = 1;
                break;
            }
        }
        if (flag == 1) {
            data_left[left_cnt - 1].angle = arr[angle_left_cnt].angle;
            data_left[left_cnt - 1].distance = arr[angle_left_cnt].distance;
            angle_left_cnt = -1;
            flag = 0;
        }
    }
    return left_cnt;
}

/**
 * @brief  提取右边界点 (0° -> 90°)
 *
 * 从0°向上扫描BLUE_ANGLE_LEFT_RIGHT度，处理0°/360°回绕。
 *
 * @return 右边界点数量
 */
uint16_t LEIDA_DATA_HANDLE7(_LEIDA_DATA data_right[], _LEIDA_DATA arr[], u16 size)
{
    int i = 0;
    int right_cnt = 0;
    float angle_right, angle_right_r;
    int angle_right_cnt = -1;
    int flag = 0;

    for (angle_right_r = 0;
         angle_right_r <= LEIDA_ANGLE_RIGHT + BLUE_ANGLE_LEFT_RIGHT;
         angle_right_r += LEIDA_ANGLE_resolution) {
        if (angle_right_r <= 0)
            angle_right = angle_right_r + 360;
        else
            angle_right = angle_right_r;

        for (i = 0; i < size; i++) {
            if ((fabs(arr[i].angle - angle_right) <= LEIDA_ANGLE_piancha) && (arr[i].distance <= 4000)) {
                angle_right_cnt = i;
            }

            if ((angle_right_cnt != -1)) {
                right_cnt++;
                flag = 1;
                break;
            }
        }
        if (flag == 1) {
            data_right[right_cnt - 1].angle = arr[angle_right_cnt].angle;
            data_right[right_cnt - 1].distance = arr[angle_right_cnt].distance;
            angle_right_cnt = -1;
            flag = 0;
        }
    }
    return right_cnt;
}

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

    for (i = 3; i < size - 4; i++) {
        if (arr[i].angle >= (180 - 80)) /* 仅左侧 (角度>=100°) */
            if (fabs(arr[i].distance - arr[i + 1].distance) >= 400)
                if (fabs(arr[i - 1].distance - arr[i + 2].distance) >= 400)
                    if (fabs(arr[i - 2].distance - arr[i + 3].distance) >= 400) {
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

    for (i = 3; i < size - 4; i++) {
        if (arr[i].angle <= (80)) /* 仅右侧 (角度<=80°) */
            if (fabs(arr[i].distance - arr[i + 1].distance) >= 400)
                if (fabs(arr[i - 1].distance - arr[i + 2].distance) >= 400)
                    if (fabs(arr[i - 2].distance - arr[i + 3].distance) >= 400) {
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
    uint16_t i, j;

    float angle_left;
    float angle_right;

    float distance_temp;
    float distance_min = 5000;
    float distance_min_2 = 5000;
    float distance_min_3 = 5000;
    float distance_min_4 = 5000;
    float distance_min_5 = 5000;
    float distance_min_6 = 5000;

    for (i = 0; i < size; i++) {
        if ((data[i].angle >= 180 - 30) && (data[i].angle <= 180 + 30)) { /* 左半: 150°~210° */
            angle_left = data[i].angle;
            /* 对侧角度: 左+180°=右, 归一化到 [0, 360) */
            angle_right = angle_left + 180;
            if (angle_right >= 360)
                angle_right -= 360;

            for (j = 0; j < size; j++) {
                /* 角度差（含环绕处理）：0°和359°物理上只差1°，需归一化到[-180,180] */
                float angle_diff = fabs(data[j].angle - angle_right);
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
    uint16_t cnt = 0;

    float y_max = -4000;
    float y_max_r = -4000;
    float y_max_r_r = -4000;
    float y_min = 4000;
    float y_min_r = 4000;
    float y_min_r_r = 4000; /* 修复: 原为-4000, 同类初值错误(此处jiange未使用, 仅保持一致) */
    float jiange = 0;

    for (; start_angle <= end_angle; start_angle += 1) {
        for (i = 0; i < size - 1; i++) {
            if ((fabs(arr[i].angle - start_angle) <= 1) && (arr[i].distance * arm_sin_f32(arr[i].angle * PI / 180) <= 2000) && (arr[i].distance >= 200)) {
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

/* ======================== 雷达角度校准 ======================== */

/**
 * @brief  利用左右边界对称性校准雷达角度偏移
 *
 * 分别在左边界(135°-225°)和右边界(-45°-45°)找最近点，
 * 计算角度偏离预期位置(180°/0°)的偏差，对所有数据点施加修正。
 *
 * @return 计算得到的角度修正量（度）
 */
float LEIDA_ANGLE_jiuzheng(_LEIDA_DATA data[], u16 size)
{
    uint16_t i;
    uint16_t left_counter = 0;
    uint16_t right_counter = 0;

    float angle_piancha_left;
    float angle_piancha_right;
    float angle_piancha;

    float distance_min_left = 5000;
    float distance_min_right = 5000;

    for (i = 0; i < size; i++) {
        if ((data[i].angle >= 180 - 45) && (data[i].angle <= 180 + 45) && (data[i].distance < distance_min_left)) {
            distance_min_left = data[i].distance;
            angle_piancha_left = data[i].angle;
        }

        if ((data[i].angle >= 180 - 45) && (data[i].angle <= 180 + 45)) {
            left_counter++;
        }

        if (((data[i].angle >= 360 - 45) || (data[i].angle <= 0 + 45)) && (data[i].distance < distance_min_right)) {
            distance_min_right = data[i].distance;
            angle_piancha_right = data[i].angle;
        }

        if ((data[i].angle >= 360 - 45) || (data[i].angle <= 0 + 45)) {
            right_counter++;
        }
    }

    if (angle_piancha_right >= 180)
        angle_piancha_right -= 360; /* 归一化 */

    /* 计算修正量: 取左右偏差的平均 */
    if ((left_counter > 15) && (right_counter > 15)) {
        angle_piancha_left = angle_piancha_left - 180;
        angle_piancha_right = angle_piancha_right - 0;
        angle_piancha = (angle_piancha_left + angle_piancha_right) / 2;
    } else if (left_counter > 15) {
        angle_piancha_left = angle_piancha_left - 180;
        angle_piancha = angle_piancha_left;
    } else if (right_counter > 15) {
        angle_piancha_right = angle_piancha_right - 0;
        angle_piancha = angle_piancha_right;
    }

    /* 对所有点施加修正 */
    for (i = 0; i < size; i++) {
        data[i].angle -= angle_piancha;
        if (data[i].angle > 360)
            data[i].angle -= 360;
        if (data[i].angle < 0)
            data[i].angle += 360;
    }

    return angle_piancha;
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

/* ======================== HANDLE12: 占位 ======================= */

uint16_t LEIDA_DATA_HANDLE12(_LEIDA_DATA arr[], u16 size)
{
    uint16_t i;
    for (i = 0; i < size; i++) {
        /* 预留，未实现 */
    }
    return 0;
}

/* ======================== HANDLE13: 边界直线度判断 ======================== */

/**
 * @brief  判断指定角度范围内的边界点是否接近直线
 *
 * 计算每个点投影到x轴的坐标，找第5级极值跨度。
 * 若跨度<100mm，则认为是直线（用于判断是否即将进入弯道）。
 *
 * @return 1=直线(x跨度<100mm), 0=非直线
 */
uint16_t LEIDA_DATA_HANDLE13(_LEIDA_DATA arr[], u16 size, float start_angle, float end_angle)
{
    uint16_t i = 0;
    float x_now;
    float x_min = 4000;
    float x_min_r = 4000;
    float x_min_r_r = 4000;
    float x_min_r_r_r = 4000;
    float x_min_r_r_r_r = 4000;
    float x_max = -4000;
    float x_max_r = -4000;
    float x_max_r_r = -4000;
    float x_max_r_r_r = -4000;
    float x_max_r_r_r_r = -4000;

    /* 5级极值查找（逐步排除前N个离群点） */
    for (i = 0; i < size; i++) {
        if ((arr[i].angle >= start_angle) && (arr[i].angle <= end_angle)) {
            x_now = arr[i].distance * arm_cos_f32(arr[i].angle * PI / 180);
            if (x_now > x_max)
                x_max = x_now;
            if (x_now < x_min)
                x_min = x_now;
        }
    }
    /* 次大/次小 */
    for (i = 0; i < size; i++) {
        if ((arr[i].angle >= start_angle) && (arr[i].angle <= end_angle)) {
            x_now = arr[i].distance * arm_cos_f32(arr[i].angle * PI / 180);
            if ((x_now > x_max_r) && (x_now != x_max))
                x_max_r = x_now;
            if ((x_now < x_min_r) && (x_now != x_min))
                x_min_r = x_now;
        }
    }
    /* 次次大/次次小 */
    for (i = 0; i < size; i++) {
        if ((arr[i].angle >= start_angle) && (arr[i].angle <= end_angle)) {
            x_now = arr[i].distance * arm_cos_f32(arr[i].angle * PI / 180);
            if ((x_now > x_max_r_r) && (x_now != x_max) && (x_now != x_max_r))
                x_max_r_r = x_now;
            if ((x_now < x_min_r_r) && (x_now != x_min) && (x_now != x_min_r))
                x_min_r_r = x_now;
        }
    }
    /* 第4级 */
    for (i = 0; i < size; i++) {
        if ((arr[i].angle >= start_angle) && (arr[i].angle <= end_angle)) {
            x_now = arr[i].distance * arm_cos_f32(arr[i].angle * PI / 180);
            if ((x_now > x_max_r_r_r) && (x_now != x_max) && (x_now != x_max_r) && (x_now != x_max_r_r))
                x_max_r_r_r = x_now;
            if ((x_now < x_min_r_r_r) && (x_now != x_min) && (x_now != x_min_r) && (x_now != x_min_r_r))
                x_min_r_r_r = x_now;
        }
    }
    /* 第5级 */
    for (i = 0; i < size; i++) {
        if ((arr[i].angle >= start_angle) && (arr[i].angle <= end_angle)) {
            x_now = arr[i].distance * arm_cos_f32(arr[i].angle * PI / 180);
            if ((x_now > x_max_r_r_r_r) && (x_now != x_max) && (x_now != x_max_r) && (x_now != x_max_r_r) && (x_now != x_max_r_r_r))
                x_max_r_r_r_r = x_now;
            if ((x_now < x_min_r_r_r_r) && (x_now != x_min) && (x_now != x_min_r) && (x_now != x_min_r_r) && (x_now != x_min_r_r_r))
                x_min_r_r_r_r = x_now;
        }
    }

    /* 第5级x跨度<100mm -> 直线 */
    return fabs(x_max_r_r_r_r - x_min_r_r_r_r) < 100 ? 1 : 0;
}

/* ======================== 调试打印工具函数 ======================== */

void LEIDA_PrintAll(const _LEIDA_DATA *pts, uint16_t count)
{
    if (!pts || count == 0) {
        printf("[LEIDA] 数据为空\r\n");
        return;
    }
    printf("[LEIDA] 全量数据=%u\r\n", count);
    for (uint16_t i = 0; i < count; i++) {
        printf("%03u:(ang=%6.1f deg, dist=%6.1f mm)\r\n", i, pts[i].angle, pts[i].distance);
    }
}

void LEIDA_PrintSample(const _LEIDA_DATA *pts, uint16_t count, uint16_t step)
{
    if (!pts || count == 0) {
        printf("[LEIDA] 数据为空\r\n");
        return;
    }
    if (step == 0)
        step = 1;
    printf("[LEIDA] 采样(step=%u) 总数=%u\r\n", step, count);
    for (uint16_t i = 0; i < count; i += step) {
        printf("%03u:(ang=%6.1f, dist=%6.1f)\r\n", i, pts[i].angle, pts[i].distance);
    }
}

void LEIDA_PrintHead(const _LEIDA_DATA *pts, uint16_t count, uint16_t n)
{
    if (!pts || count == 0) {
        printf("[LEIDA] 数据为空\r\n");
        return;
    }
    if (n > count)
        n = count;
    printf("[LEIDA] 前 %u/%u 个点:\r\n", n, count);
    for (uint16_t i = 0; i < n; i++) {
        printf("%03u:(ang=%6.1f, dist=%6.1f)\r\n", i, pts[i].angle, pts[i].distance);
    }
}
