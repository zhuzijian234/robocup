/**
 * @file    LEIDA_DATA.h
 * @brief   激光雷达数据结构与处理函数声明
 *
 * 本模块处理雷达原始极坐标数据，包含以下功能：
 * - 解析原始串口数据包（每包47字节，12个数据点）
 * - 极坐标(角度,距离) -> 笛卡尔坐标(x,y) 转换
 * - 提取左/右边界点
 * - 检测边界突变点（弯道入口）
 * - 计算跑道宽度（左右边界距离对）
 * - 前方路径扫描与障碍物检测
 *
 * 数据处理流水线：
 *   DMA缓冲区 -> HANDLE1解析 -> LEIDA_DATA[] (极坐标)
 *     -> HANDLE3_2筛选有效点 -> LEIDA_DATA2[]
 *     -> HANDLE2转笛卡尔 -> LEIDA_DATA_plane[]
 *     -> HANDLE6/7提取左右边界 -> LEIDA_DATA_LEFT[] / LEIDA_DATA_RIGHT[]
 *     -> HANDLE4计算中线 -> LEIDA_DATA_CENTER[]
 *     -> HANDLE5扫描前方路径 -> LEIDA_DATA_Forward[]
 *     -> HANDLE8/9检测突变点 -> 断点距离
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
extern _LEIDA_DATA LEIDA_DATA[];        /* 雷达原始极坐标数据 */
extern _LEIDA_DATA LEIDA_DATA2[];       /* 筛选后的有效极坐标数据 */
extern _LEIDA_DATA_plane LEIDA_DATA_plane[];  /* 笛卡尔平面坐标数据 */
extern u8 tiaoshi;                      /* 调试标志 */

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
extern _LEIDA_DATA LEIDA_DATA_LEFT_2[];
extern _LEIDA_DATA LEIDA_DATA_RIGHT_2[];
extern _LEIDA_DATA_plane LEIDA_DATA_LEFT_Plane[];
extern _LEIDA_DATA_plane LEIDA_DATA_RIGHT_Plane[];
extern _LEIDA_DATA_plane LEIDA_DATA_LEFT_Plane_2[];
extern _LEIDA_DATA_plane LEIDA_DATA_RIGHT_Plane_2[];
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

/* HANDLE1: 解析原始字节流为极坐标数据点
 * 每包47字节，12个数据点，同步头0x54
 * 返回1成功，0失败 */
uint16_t LEIDA_DATA_HANDLE1(_LEIDA_DATA data[], u8 arr[], u16 size);

/* HANDLE2: 极坐标转笛卡尔坐标 */
void LEIDA_DATA_HANDLE2(_LEIDA_DATA_plane data[], _LEIDA_DATA arr[], u16 size);

/* HANDLE3: 筛选左右90度内距离非零的有效点 */
uint16_t LEIDA_DATA_HANDLE3(_LEIDA_DATA data[], _LEIDA_DATA arr[], u16 size);

/* HANDLE3_2: 筛选距离>=100mm的所有有效点 */
uint16_t LEIDA_DATA_HANDLE3_2(_LEIDA_DATA data[], _LEIDA_DATA arr[], u16 size);

/* HANDLE4: 配对左右边界点计算中线点，并滤除离群点 */
uint16_t LEIDA_DATA_HANDLE4(_LEIDA_DATA_plane data_center[], _LEIDA_DATA arr[], u16 size);

/* HANDLE5: 扫描前方路径(70-110度)，检测无障碍的直行点集
 * 如果在86-94度有>=5个点距离>1500mm，则认为有障碍物，返回0 */
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

/* HANDLE12: 占位/预留 */
uint16_t LEIDA_DATA_HANDLE12(_LEIDA_DATA arr[], u16 size);

/* HANDLE13: 判断指定角度范围内的边界点是否接近直线
 * 第5级x跨度<100mm返回1，否则返回0 */
uint16_t LEIDA_DATA_HANDLE13(_LEIDA_DATA arr[], u16 size, float start_angle, float end_angle);

/* 计算跑道宽度：左右边界距离对的第6小值 */
float LEIDA_Distance(_LEIDA_DATA data[], u16 size);

/* 雷达角度校准：利用左右边界对称性计算角度偏差 */
float LEIDA_ANGLE_jiuzheng(_LEIDA_DATA data[], u16 size);

/* 反转_LEIDA_DATA_plane数组 */
void reverse(_LEIDA_DATA_plane a[], int sz);

/* ============ 调试打印接口 ============ */
void LEIDA_PrintAll(const _LEIDA_DATA *pts, uint16_t count);
void LEIDA_PrintSample(const _LEIDA_DATA *pts, uint16_t count, uint16_t step);
void LEIDA_PrintHead(const _LEIDA_DATA *pts, uint16_t count, uint16_t n);

#endif
