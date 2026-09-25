/**
 * @brief M10P工作点云的几何处理：坐标转换、边界、中线、断点和前方墙面。
 * 字节接收由DMA.c负责；协议解析和整圈所有权由m10p.c负责。
 * m10p_vehicle.c生成LEIDA_DATA2后，本模块仅使用有效点数范围内的数据。
 * 坐标约定：0度向右、90度向前；距离和x/y均以毫米计。
 */

#ifndef __LEIDA_DATA_H
#define __LEIDA_DATA_H

#include "sys.h"
#include "usart.h"
#include "arm_math.h"

/* 极坐标数据点（雷达原始输出） */
typedef struct {
    float angle;     /* 角度（度），0=右侧，90=正前方，180=左侧 */
    float distance;  /* 距离（毫米） */
    // u8    Quality;
} _LEIDA_DATA;

/* 笛卡尔坐标数据点（极坐标转平面后） */
typedef struct {
    float _x;  /* X坐标（毫米），正=右 */
    float _y;  /* Y坐标（毫米），正=前 */
} _LEIDA_DATA_plane;

#define LEIDA_DATA_COUNTER 800   /* 每帧最大数据点数 */
extern uint16_t valid_couter;
extern uint8_t LEIDA_vertical_valid;
extern volatile uint16_t LEIDA_raw_count; /* 当前M10P整圈的真实点数，不是工作数组容量 */
extern uint16_t LEIDA_speed_dps;   extern _LEIDA_DATA LEIDA_DATA2[];       /* 筛选后的有效极坐标数据 */

/* 雷达角度配置
 * LEIDA_ANGLE_CENTER = 90度（正前方）
 * LEIDA_ANGLE_LEFT   = 180度（左侧）
 * LEIDA_ANGLE_RIGHT  = 0度（右侧）
 */
#define LEIDA_ANGLE_PIANCHA    (0)
#define LEIDA_ANGLE_CENTER     90.0f + LEIDA_ANGLE_PIANCHA
#define LEIDA_ANGLE_LEFT       LEIDA_ANGLE_CENTER + 90
#define LEIDA_ANGLE_RIGHT      LEIDA_ANGLE_CENTER - 90

#define LEIDA_ANGLE_resolution 1     /* 角度扫描步长 */
#define LEIDA_ANGLE_yuliang    75    /* 左右边界角度余量 */
#define LEIDA_ANGLE_piancha    0.5   /* 角度匹配容差 */

/* 边界数据数组 */
extern _LEIDA_DATA LEIDA_DATA_LEFT[];
extern _LEIDA_DATA LEIDA_DATA_RIGHT[];
extern _LEIDA_DATA_plane LEIDA_DATA_LEFT_Plane[];
extern _LEIDA_DATA_plane LEIDA_DATA_RIGHT_Plane[];
extern _LEIDA_DATA_plane LEIDA_DATA_CENTER[];       /* 中线点 */
extern uint16_t LEFT_cnt;
extern uint16_t RIGHT_cnt;
extern uint16_t LEFT_cnt_2;
extern uint16_t RIGHT_cnt_2;
extern uint16_t CENTER_cnt;

extern float BLUE_ANGLE_LEFT_RIGHT;

/* 前方路径数据 */
extern _LEIDA_DATA_plane LEIDA_DATA_Forward[];
extern _LEIDA_DATA_plane LEIDA_DATA_Forward_2[];
extern _LEIDA_DATA_plane LEIDA_DATA_Forward_3[];
extern uint16_t Forward_cnt;
extern uint16_t Forward_cnt_2;
extern uint16_t Forward_cnt_3;
extern uint32_t Forward_Distance;
extern float zhongxian_chuizhi;   /* 中线垂直时的x坐标 */
extern float zhongxian_junzhi;    /* 中线均值x坐标 */

/* ============ 雷达数据处理流水线 ============ */

/* HANDLE2: 极坐标转笛卡尔坐标 */
void LEIDA_DATA_HANDLE2(_LEIDA_DATA_plane data[], _LEIDA_DATA arr[], u16 size);

/* HANDLE4: 配对左右边界点计算中线点，并滤除离群点 */
uint16_t LEIDA_DATA_HANDLE4(_LEIDA_DATA_plane data_center[], _LEIDA_DATA arr[], u16 size);

/* HANDLE5: 提取前方墙面点集供拟合；86~94度至少12个远点且无近点时返回0。
 * 返回0只表示此函数未提取墙面，不代表允许驱动，许可由适配层和TIM5共同决定。 */
uint16_t LEIDA_DATA_HANDLE5(_LEIDA_DATA_plane data[], _LEIDA_DATA arr[], u16 size);

/* HANDLE5_2: 同HANDLE5，但角度范围可配置 [start_angle, end_angle] */
uint16_t LEIDA_DATA_HANDLE5_2(_LEIDA_DATA_plane data[], _LEIDA_DATA arr[], u16 size,
                               float start_angle, float end_angle);

/* HANDLE6: 提取左边界点（角度180->90度） */
uint16_t LEIDA_DATA_HANDLE6(_LEIDA_DATA data_left[], _LEIDA_DATA arr[], u16 size);

/* HANDLE7: 提取右边界点（角度0->90度） */
uint16_t LEIDA_DATA_HANDLE7(_LEIDA_DATA data_right[], _LEIDA_DATA arr[], u16 size);

/* HANDLE8: 检测左边界突变点（弯道入口）
 * 6点滑动窗口，检测距离跳变>=400mm的突变 */
uint16_t LEIDA_DATA_HANDLE8(_LEIDA_DATA arr[], u16 size);

/* HANDLE9: 检测右边界突变点（同HANDLE8算法） */
uint16_t LEIDA_DATA_HANDLE9(_LEIDA_DATA arr[], u16 size);

/* HANDLE10: 左右边界离群点滤除（按x坐标间距） */
uint16_t LEIDA_DATA_HANDLE10(_LEIDA_DATA_plane arr[], u16 size);

/* HANDLE11: 判断中线是否垂直（直道）
 * x跨度<10mm视为垂直，返回中点x坐标；否则返回0 */
float LEIDA_DATA_HANDLE11(_LEIDA_DATA_plane arr[], u16 size_start, u16 size_end);

/* 计算跑道宽度：左右边界距离对的第6小值 */
float LEIDA_Distance(_LEIDA_DATA data[], u16 size);

/* 反转_LEIDA_DATA_plane数组 */
void reverse(_LEIDA_DATA_plane a[], int sz);

#endif
