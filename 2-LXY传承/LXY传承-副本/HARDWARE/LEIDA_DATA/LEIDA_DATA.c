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
#include "centre_line.h"

_LEIDA_DATA LEIDA_DATA[LEIDA_DATA_COUNTER];
_LEIDA_DATA LEIDA_DATA2[LEIDA_DATA_COUNTER];
_LEIDA_DATA_plane LEIDA_DATA_plane[LEIDA_DATA_COUNTER];

u8 tiaoshi = 0;  /* 调试标志 */

/* ======================== HANDLE1: 原始串口帧解析 ======================== */

/**
 * @brief  将雷达原始串口字节流解析为极坐标数据点
 *
 * 雷达数据包格式 (M10系列, 每包47字节):
 *   Byte 0:     帧头 (0x54)
 *   Byte 1-3:   保留
 *   Byte 4-5:   起始角度 (低字节在前) / 100 = 度
 *   Byte 6-7:   数据点0距离 (低字节在前) mm 8:保留,每个数据点3字节
 *   Byte 9-10:  数据点1距离 11:保留
 *   ...         每包12个数据点, 每点3字节
 *   Byte 42-43: 结束角度 (低字节在前) / 100 = 度
 *   Byte 44-45: CRC校验 (未校验)
 *
 * 解析后角度旋转+90度: 0°=右侧(x+), 90°=前方(y+), 180°=左侧(x-)
 *
 * @return 1=成功, 0=未找到有效帧头
 */
uint16_t LEIDA_DATA_HANDLE1(_LEIDA_DATA data[], u8 arr[], u16 size)
{
    int i, j, k;
    float start_angle;
    float end_angle;

    /* 寻找同步模式: 间隔47字节的三个连续0x54帧头 */
    for (i = 0; i < size - 47 - 47 - 1; i++) {
        if (arr[i] == 0x54) {
            if (arr[i + 47] == 0x54) {
                if (arr[i + 94] == 0x54) {
                    break;
                }
            }
        }
    }
    if (i == size - 47 - 47 - 1) return 0;  /* 未找到有效同步 */

    /* 解析数据包: 每包47字节 -> 12个数据点 */
    for (j = 0; i < size - 47 - 1 && j < LEIDA_DATA_COUNTER - 12; i += 47, j += 12) {
        if (arr[i] == 0x54) {
            start_angle = (((u16)arr[i + 5] << 8) + (u16)arr[i + 4]) / 100.0f;
            end_angle   = (((u16)arr[i + 43] << 8) + (u16)arr[i + 42]) / 100.0f;

            /* 处理角度回绕: 结束角度<起始角度，说明扫描跨过了0度 */
            if (start_angle > end_angle)
                end_angle += 360;
 /*数据是怎么存的

arr[i+7+3*k]              arr[i+6+3*k]
  u8 (1字节)                u8 (1字节)
  例: 0x02                 例: 0x64
     │                        │
     ▼ 强转 (u16)             ▼ 强转 (u16)
  0x0002                   0x0064
  (2字节)                  (2字节)
     │                        │
     ▼ << 8                   │
  0x0200                      │
  (512)                       │
     │                        │
     └───────── + ────────────┘
                 │
                 ▼
          u16: 0x0264 = 612
          (2字节, 最大值 65535)
                 │
                 ▼ × 1.0f
          float: 612.0f
          (4字节, IEEE 754)
                 │
                 ▼
    data[j+k].distance = 612.0f*/
            for (k = 0; k < 12; k++) {
                /* 距离: byte 6+3k, 7+3k (低字节在前) 转为16字节->左移八位,不然溢出,低字节直接转为16字节 */
                data[j + k].distance = 1.0f * (((u16)arr[i + 7 + 3 * k] << 8) + (u16)arr[i + 6 + 3 * k]);
                /*i:包的帧头数组下标,每次+47;j:data数组/包索引,每个包12个数据点;size:字节总数 ,k:,data数组/数据点索引,每3字节一个数据点*/
                /* 角度: 起始和结束之间线性插值 */
                data[j + k].angle = start_angle + (end_angle - start_angle) / 12 * k;

                /* 角度归一化到 [0, 360) */
                if (data[j + k].angle > 360.0f) data[j + k].angle -= 360.0f;
                if (data[j + k].angle < 0.0f)   data[j + k].angle += 360.0f;

                /* 坐标系旋转: 0°->右侧(x+), 90°->前方(y+) ,原本:0°->正前,90°->右侧*/
                data[j + k].angle = -1.0f * data[j + k].angle + 360.0f + LEIDA_ANGLE_CENTER;

                if (data[j + k].angle > 360.0f) data[j + k].angle -= 360.0f;
                if (data[j + k].angle < 0.0f)   data[j + k].angle += 360.0f;
            }
        }
    }
    return 1;
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
    for (i = 10; i < size - 10; i++) {/*掐头去尾10个噪点*/
        if (arr[i].distance != 0) {
            if ((arr[i].angle >= LEIDA_ANGLE_RIGHT) && (arr[i].angle <= LEIDA_ANGLE_LEFT)) {
                data[j].angle    = arr[i].angle;
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
    for (i = 10; i < size - 10; i++) {
        if ((arr[i].distance >= 100)) {
            data[j].angle    = arr[i].angle;
            data[j].distance = arr[i].distance;
            j++;
        }
    }
    return j;
}

/* ======================== 边界/中线数据处理数组 ======================== */

_LEIDA_DATA LEIDA_DATA_LEFT[LEIDA_DATA_COUNTER / 2];
_LEIDA_DATA LEIDA_DATA_RIGHT[LEIDA_DATA_COUNTER / 2];
_LEIDA_DATA LEIDA_DATA_LEFT_2[LEIDA_DATA_COUNTER / 2];
_LEIDA_DATA LEIDA_DATA_RIGHT_2[LEIDA_DATA_COUNTER / 2];
_LEIDA_DATA_plane LEIDA_DATA_LEFT_Plane[LEIDA_DATA_COUNTER / 2];
_LEIDA_DATA_plane LEIDA_DATA_RIGHT_Plane[LEIDA_DATA_COUNTER / 2];
_LEIDA_DATA_plane LEIDA_DATA_LEFT_Plane_2[LEIDA_DATA_COUNTER / 2];
_LEIDA_DATA_plane LEIDA_DATA_RIGHT_Plane_2[LEIDA_DATA_COUNTER / 2];
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
    int i            = 0;
    uint16_t j       = 0;
    int center_cnt   = 0;
    int center_cnt_r = 0;
    float angle_right, angle_left;
    int angle_right_cnt = 0;
    int angle_left_cnt  = 0;
    int flag            = 0;
    _LEIDA_DATA_plane temp;

    float x_max     = -4000;
    float x_max_r   = -4000;
    float x_max_r_r = -4000;
    float x_min     = 4000;
    float x_min_r   = 4000;
    float x_min_r_r = 4000;
    float jiange    = 0;

    /* 角度扫描: 左右对称角度配对 */
    for (angle_right = LEIDA_ANGLE_RIGHT, angle_left = LEIDA_ANGLE_LEFT;
         angle_right <= LEIDA_ANGLE_RIGHT + LEIDA_ANGLE_yuliang; /*各扫 75°（覆盖 0°~75°左 + 105°~180°右）*/
         angle_right += 0.6, angle_left -= 0.6) {
/*匹配左右对称角度点*/
        for (i = 0; i < size; i++) {
            /* 匹配右侧点: 角度容差0.5°内, 距离50-2000mm */
            if ((fabs(arr[i].angle - angle_right) <= LEIDA_ANGLE_piancha)
                && (arr[i].distance <= 2000) && (arr[i].distance >= 50)) {
                angle_right_cnt = i;
            }

            /* 匹配左侧点: 角度容差0.5°内, 距离50-2000mm */
            if ((fabs(arr[i].angle - angle_left) <= LEIDA_ANGLE_piancha)
                && (arr[i].distance <= 2000) && (arr[i].distance >= 50)) {
                angle_left_cnt = i;
            }

            /* 左右都匹配到 -> 计算中点 */
            if ((angle_left_cnt != 0) && (angle_right_cnt != 0)) {
                center_cnt++;
                flag = 1;
                break;
            }
        }

        if (flag == 1) {
            /* 中点 = 左右笛卡尔坐标的平均值 */
            data_center[center_cnt - 1]._x =
                (arr[angle_right_cnt].distance * arm_cos_f32(arr[angle_right_cnt].angle * PI / 180)
                 + arr[angle_left_cnt].distance * arm_cos_f32(arr[angle_left_cnt].angle * PI / 180)) / 2;
            data_center[center_cnt - 1]._y =
                (arr[angle_right_cnt].distance * arm_sin_f32(arr[angle_right_cnt].angle * PI / 180)
                 + arr[angle_left_cnt].distance * arm_sin_f32(arr[angle_left_cnt].angle * PI / 180)) / 2;
            angle_right_cnt = 0;
            angle_left_cnt  = 0;
            flag            = 0;
        }
    }

    /* 寻找x的最大、次大、次次大 和 最小、次小、次次小（排除极端离群点） */
    for (i = 0; i < center_cnt; i++) {
        if (data_center[i]._x > x_max) x_max = data_center[i]._x;
        if (data_center[i]._x < x_min) x_min = data_center[i]._x;
    }
    for (i = 0; i < center_cnt; i++) {
        if ((data_center[i]._x != x_max) && (data_center[i]._x > x_max_r)) x_max_r = data_center[i]._x;
        if ((data_center[i]._x != x_min) && (data_center[i]._x < x_min_r)) x_min_r = data_center[i]._x;
    }
    for (i = 0; i < center_cnt; i++) {
        if ((data_center[i]._x != x_max) && (data_center[i]._x != x_max_r) && (data_center[i]._x > x_max_r_r))
            x_max_r_r = data_center[i]._x;
        if ((data_center[i]._x != x_min) && (data_center[i]._x != x_min_r) && (data_center[i]._x < x_min_r_r))
            x_min_r_r = data_center[i]._x;
    }

    /* 平均间距（使用第3级极值，排除顶部各2个离群点） */
    jiange           = (x_max_r_r - x_min_r_r) / center_cnt;
    zhongxian_junzhi = (x_max_r_r + x_min_r_r) / 2;

    /* 滤除x坐标与相邻点不一致的点 + 只保留y<=800mm的近点 */
    for (i = 0, j = 0; i < center_cnt - 3; i++) {
        if ((fabs(data_center[i]._x - data_center[i + 1]._x) <= 5 * jiange)
            && (fabs(data_center[i]._x - data_center[i + 2]._x) <= 10 * jiange)
            && (fabs(data_center[i]._x - data_center[i + 3]._x) <= 15 * jiange)) {
            if (data_center[i]._y <= 800) {
                data_center[j]._x = data_center[i]._x;
                data_center[j]._y = data_center[i]._y;
                j++;
            }
        }
    }

    return j; /*返回新索引,后面的不用了*/
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
    int i           = 0;
    int left_cnt    = 0;
    float angle_left;
    int angle_left_cnt = -1;
    int flag           = 0;

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
            data_left[left_cnt - 1].angle    = arr[angle_left_cnt].angle;
            data_left[left_cnt - 1].distance = arr[angle_left_cnt].distance;
            angle_left_cnt                    = -1;
            flag                              = 0;
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
    int i            = 0;
    int right_cnt    = 0;
    float angle_right, angle_right_r;
    int angle_right_cnt = -1;
    int flag            = 0;

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
            data_right[right_cnt - 1].angle    = arr[angle_right_cnt].angle;
            data_right[right_cnt - 1].distance = arr[angle_right_cnt].distance;
            angle_right_cnt = -1;
            flag            = 0;
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
    float min  = 0;

    for (i = 3; i < size - 4; i++) {
        if (arr[i].angle >= (180 - 80))  /* 仅左侧 (角度>=100°) */
            if (fabs(arr[i].distance - arr[i + 1].distance) >= 400)
                if (fabs(arr[i - 1].distance - arr[i + 2].distance) >= 400)
                    if (fabs(arr[i - 2].distance - arr[i + 3].distance) >= 400) {
                        /* 取突变两侧中距离较近的点 */
                        if (arr[i].distance > arr[i + 2].distance)
                            min = arr[i + 2].distance * arm_sin_f32(arr[i + 2].angle * PI / 180);
                        else
                            min = arr[i - 1].distance * arm_sin_f32(arr[i - 1].angle * PI / 180);

                        if (min < 1000)  /* 仅报告1米以内的突变 */
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
    float min  = 0;

    for (i = 3; i < size - 4; i++) {
        if (arr[i].angle <= (80))  /* 仅右侧 (角度<=80°) */
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
    float distance_min   = 5000;
    float distance_min_2 = 5000;
    float distance_min_3 = 5000;
    float distance_min_4 = 5000;
    float distance_min_5 = 5000;
    float distance_min_6 = 5000;

    for (i = 0; i < size; i++) {
        if ((data[i].angle >= 180 - 30) && (data[i].angle <= 180 + 30)) {  /* 左半: 150°~210° */
            angle_left  = data[i].angle;
            /* 对侧角度: 左+180°=右, 归一化到 [0, 360) */
            angle_right = angle_left + 180;
            if (angle_right >= 360) angle_right -= 360;

            for (j = 0; j < size; j++) {
                /* 角度差（含环绕处理）：0°和359°物理上只差1°，需归一化到[-180,180] */
                float angle_diff = fabs(data[j].angle - angle_right);
                if (angle_diff > 180) angle_diff = 360 - angle_diff;
                if (angle_diff <= 1.0f) {
                    distance_temp = data[i].distance + data[j].distance;

                    /* 6元素最小级联: 逐个槽位比较，将distance_temp插入合适位置 */
                    if (distance_temp <= distance_min) {
                        distance_min_6 = distance_min_5;
                        distance_min_5 = distance_min_4;
                        distance_min_4 = distance_min_3;
                        distance_min_3 = distance_min_2;
                        distance_min_2 = distance_min;
                        distance_min   = distance_temp;
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
    uint16_t counter        = 0;
    uint16_t zhidao_counter = 0;
    Midline_type midline;
    float start_angle = 60;
    uint16_t cnt      = 0;

    float y_max     = -4000;
    float y_max_r   = -4000;
    float y_max_r_r = -4000;
    float y_min     = 4000;
    float y_min_r   = 4000;
    float y_min_r_r = 4000;
    float jiange    = 0;

    /* 空旷检测: 正前方(86°-94°)有>=5个点距离>1.5m，说明前方空旷(有缺口) */
    for (i = 0; i < size - 1; i++) {
        if ((fabs(arr[i].angle - 90) <= 4) && (arr[i].distance > 1500) && (arr[i].distance <= 4000)) {
            cnt++;
        }
        if (cnt >= 5) return 0;  /* 前方无遮挡，不用前视数据 */
    }

    /* 前方有墙: 扫描前方弧70°-110°,得到前视数据 */
    for (start_angle = 70; start_angle <= 110; start_angle += 1) {
        for (i = 0; i < size - 1; i++) {
            if ((fabs(arr[i].angle - start_angle) <= 1) /*找角度匹配的那个点*/
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
    }/*为了让650行的滤除循环在k>0,即y[i]<y[i+1]时有意义*/

    /* 找y极值 (最大/次大/次次大, 最小/次小/次次小) */
    for (i = 0; i < counter; i++) {
        if (data[i]._y > y_max) y_max = data[i]._y;
        if (data[i]._y < y_min) y_min = data[i]._y;
    }
    for (i = 0; i < counter; i++) {
        if ((data[i]._y > y_max_r) && (data[i]._y != y_max)) y_max_r = data[i]._y;
        if ((data[i]._y < y_min_r) && (data[i]._y != y_min)) y_min_r = data[i]._y;
    }
    for (i = 0; i < counter; i++) {
        if ((data[i]._y > y_max_r_r) && (data[i]._y != y_max) && (data[i]._y != y_max_r)) y_max_r_r = data[i]._y;
        if ((data[i]._y < y_min_r_r) && (data[i]._y != y_min) && (data[i]._y != y_min_r)) y_min_r_r = data[i]._y;
    }

    jiange = (y_max_r_r - y_min_r_r) / (counter - 4);  /* 排除4个极值点 */

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
    uint16_t counter        = 0;
    uint16_t zhidao_counter = 0;
    Midline_type midline;
    uint16_t cnt = 0;

    float y_max     = -4000;
    float y_max_r   = -4000;
    float y_max_r_r = -4000;
    float y_min     = 4000;
    float y_min_r   = 4000;
    float y_min_r_r = -4000;
    float jiange    = 0;

    for (; start_angle <= end_angle; start_angle += 1) {
        for (i = 0; i < size - 1; i++) {
            if ((fabs(arr[i].angle - start_angle) <= 1)
                && (arr[i].distance * arm_sin_f32(arr[i].angle * PI / 180) <= 2000)
                && (arr[i].distance >= 200)) {
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
    int left  = 0;
    int right = sz - 1;
    float b;

    /* 反转 _x */
    while (left < right) {
        b           = a[right]._x;
        a[right]._x = a[left]._x;
        a[left]._x  = b;
        left++;
        right--;
    }

    /* 反转 _y */
    left  = 0;
    right = sz - 1;
    while (left < right) {
        b           = a[right]._y;
        a[right]._y = a[left]._y;
        a[left]._y  = b;
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
    uint16_t left_counter  = 0;
    uint16_t right_counter = 0;

    float angle_piancha_left;
    float angle_piancha_right;
    float angle_piancha;

    float distance_min_left  = 5000;
    float distance_min_right = 5000;

    for (i = 0; i < size; i++) {
        if ((data[i].angle >= 180 - 45) && (data[i].angle <= 180 + 45)
            && (data[i].distance < distance_min_left)) {
            distance_min_left  = data[i].distance;
            angle_piancha_left = data[i].angle;
        }

        if ((data[i].angle >= 180 - 45) && (data[i].angle <= 180 + 45)) {
            left_counter++;
        }

        if (((data[i].angle >= 360 - 45) || (data[i].angle <= 0 + 45))
            && (data[i].distance < distance_min_right)) {
            distance_min_right  = data[i].distance;
            angle_piancha_right = data[i].angle;
        }

        if ((data[i].angle >= 360 - 45) || (data[i].angle <= 0 + 45)) {
            right_counter++;
        }
    }

    if (angle_piancha_right >= 180) angle_piancha_right -= 360;  /* 归一化 */

    /* 计算修正量: 取左右偏差的平均 */
    if ((left_counter > 15) && (right_counter > 15)) {
        angle_piancha_left  = angle_piancha_left - 180;
        angle_piancha_right = angle_piancha_right - 0;
        angle_piancha = (angle_piancha_left + angle_piancha_right) / 2;
    } else if (left_counter > 15) {
        angle_piancha_left = angle_piancha_left - 180;
        angle_piancha      = angle_piancha_left;
    } else if (right_counter > 15) {
        angle_piancha_right = angle_piancha_right - 0;
        angle_piancha       = angle_piancha_right;
    }

    /* 对所有点施加修正 */
    for (i = 0; i < size; i++) {
        data[i].angle -= angle_piancha;
        if (data[i].angle > 360) data[i].angle -= 360;
        if (data[i].angle < 0)   data[i].angle += 360;
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
    uint16_t i, j;

    float x_max     = -4000;
    float x_max_r   = -4000;
    float x_max_r_r = -4000;
    float x_min     = 4000;
    float x_min_r   = 4000;
    float x_min_r_r = 4000;
    float jiange    = 0;

    /* 找x极值 */
    for (i = 0; i < size; i++) {
        if (arr[i]._x > x_max) x_max = arr[i]._x;
        if (arr[i]._x < x_min) x_min = arr[i]._x;
    }
    for (i = 0; i < size; i++) {
        if ((arr[i]._x != x_max) && (arr[i]._x > x_max_r)) x_max_r = arr[i]._x;
        if ((arr[i]._x != x_min) && (arr[i]._x < x_min_r)) x_min_r = arr[i]._x;
    }
    for (i = 0; i < size; i++) {
        if ((arr[i]._x != x_max) && (arr[i]._x != x_max_r) && (arr[i]._x > x_max_r_r)) x_max_r_r = arr[i]._x;
        if ((arr[i]._x != x_min) && (arr[i]._x != x_min_r) && (arr[i]._x < x_min_r_r)) x_min_r_r = arr[i]._x;
    }

    jiange = (x_max_r_r - x_min_r_r) / size;

    /* 按间距一致性滤除 */
    for (i = 0, j = 0; i < size - 1; i++) {
        if (fabs(arr[i]._x - arr[i + 1]._x) <= 5 * jiange) {
            arr[j]._x = arr[i]._x;
            arr[j]._y = arr[i]._y;
            j++;
        }
    }

    return j;
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
float LEIDA_DATA_HANDLE11(_LEIDA_DATA_plane arr[], u16 size_start, u16 size_end)
{
    uint16_t i;
    float x_max     = -4000;
    float x_max_r   = -4000;
    float x_max_r_r = -4000;
    float x_min     = 4000;
    float x_min_r   = 4000;
    float x_min_r_r = -4000;
    float jiange    = 0;

    /* 在指定区域内找x极值 */
    for (i = size_start; i < size_end; i++) {
        if (arr[i]._x > x_max) x_max = arr[i]._x;
        if (arr[i]._x < x_min) x_min = arr[i]._x;
    }
    for (i = size_start; i < size_end; i++) {
        if ((arr[i]._x != x_max) && (arr[i]._x > x_max_r)) x_max_r = arr[i]._x;
        if ((arr[i]._x != x_min) && (arr[i]._x < x_min_r)) x_min_r = arr[i]._x;
    }
    for (i = size_start; i < size_end; i++) {
        if ((arr[i]._x != x_max) && (arr[i]._x != x_max_r) && (arr[i]._x > x_max_r_r)) x_max_r_r = arr[i]._x;
        if ((arr[i]._x != x_min) && (arr[i]._x != x_min_r) && (arr[i]._x < x_min_r_r)) x_min_r_r = arr[i]._x;
    }

    jiange = (x_max_r_r - x_min_r_r); /*x几乎不变,理解为直道*/

    /* x跨度<10mm -> 垂直，返回中点x */
    if (jiange < 10)
        return (x_max_r_r + x_min_r_r) / 2;
    else
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
    float x_min         = 4000;
    float x_min_r       = 4000;
    float x_min_r_r     = 4000;
    float x_min_r_r_r   = 4000;
    float x_min_r_r_r_r = 4000;
    float x_max         = -4000;
    float x_max_r       = -4000;
    float x_max_r_r     = -4000;
    float x_max_r_r_r   = -4000;
    float x_max_r_r_r_r = -4000;

    /* 5级极值查找（逐步排除前N个离群点） */
    for (i = 0; i < size; i++) {
        if ((arr[i].angle >= start_angle) && (arr[i].angle <= end_angle)) {
            x_now = arr[i].distance * arm_cos_f32(arr[i].angle * PI / 180);
            if (x_now > x_max) x_max = x_now;
            if (x_now < x_min) x_min = x_now;
        }
    }
    /* 次大/次小 */
    for (i = 0; i < size; i++) {
        if ((arr[i].angle >= start_angle) && (arr[i].angle <= end_angle)) {
            x_now = arr[i].distance * arm_cos_f32(arr[i].angle * PI / 180);
            if ((x_now > x_max_r) && (x_now != x_max)) x_max_r = x_now;
            if ((x_now < x_min_r) && (x_now != x_min)) x_min_r = x_now;
        }
    }
    /* 次次大/次次小 */
    for (i = 0; i < size; i++) {
        if ((arr[i].angle >= start_angle) && (arr[i].angle <= end_angle)) {
            x_now = arr[i].distance * arm_cos_f32(arr[i].angle * PI / 180);
            if ((x_now > x_max_r_r) && (x_now != x_max) && (x_now != x_max_r)) x_max_r_r = x_now;
            if ((x_now < x_min_r_r) && (x_now != x_min) && (x_now != x_min_r)) x_min_r_r = x_now;
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
            if ((x_now > x_max_r_r_r_r) && (x_now != x_max) && (x_now != x_max_r)
                && (x_now != x_max_r_r) && (x_now != x_max_r_r_r))
                x_max_r_r_r_r = x_now;
            if ((x_now < x_min_r_r_r_r) && (x_now != x_min) && (x_now != x_min_r)
                && (x_now != x_min_r_r) && (x_now != x_min_r_r_r))
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
    if (step == 0) step = 1;
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
    if (n > count) n = count;
    printf("[LEIDA] 前 %u/%u 个点:\r\n", n, count);
    for (uint16_t i = 0; i < n; i++) {
        printf("%03u:(ang=%6.1f, dist=%6.1f)\r\n", i, pts[i].angle, pts[i].distance);
    }
}
