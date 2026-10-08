/**
 * @file    m10p_vehicle.h
 * @brief   M10P 适配层对外接口 — 取数据(M10P_Poll)与转换打包(M10P_Build)
 *
 * 谁在用: main.c 主循环。典型顺序:
 *     M10P_Poll();                         // 收 + 解析成扫描帧
 *     scan = M10P_Acquire();               // 取一帧(m10p.c)
 *     valid_couter = M10P_Build(scan, LEIDA_DATA2, LEIDA_DATA_COUNTER);
 *     ... 循线计算 ...
 *     M10P_Release(scan);                  // 还回去
 */
#ifndef M10P_VEHICLE_H
#define M10P_VEHICLE_H
#include "m10p.h"
#include "LEIDA_DATA.h"
/* 本帧来源标记: 帧号 / 正前方首个有效点的时刻 / 链路代号 —— 遥测里用来对上"这帧是哪来的" */
extern uint32_t M10P_control_seq, M10P_control_front_us, M10P_control_epoch;
/* 三个扇区的点数(按 0.5° 桶计): 正前 70°~110° / 左 110°~170° / 右 10°~70° */
extern uint16_t M10P_front_bins, M10P_left_bins, M10P_right_bins;
/* 最近一次感知检查的快照；age不是发送命令时计算，避免等待命令造成误判。 */
extern uint16_t M10P_front_gap_bins;
extern uint32_t M10P_build_age_us;
extern uint8_t M10P_front_seen, M10P_build_epoch_ok;
/* 正前方走廊(半宽180mm→130mm)内最近的障碍距离(mm)，仅供诊断，不参与电机控制。 */
extern float M10P_clearance_mm;
extern uint8_t M10P_perception_ok; /* 0=跳过转向更新并清D历史；保持舵角，电机PI继续运行 */
/* ==================== 感知分级 (20261008 起) ====================
 * 原来只有"有效/无效"两档, 无效就整帧不写舵机 → 锥桶赛道上两侧稀疏点会导致
 * 舵角被冻结在最后一次的值(实测冻结在右打满 1170)。现在拆成分级判据, 由 main.c
 * 决定用哪种转向来源, 保证只要有可用回波就仍然输出转向。 */
extern uint8_t M10P_front_ok, M10P_left_ok, M10P_right_ok; /* 三个扇区各自的达标情况 */
extern uint8_t M10P_perception_why;  /* 判据失败原因位图, 用来定位卡在哪一条 */
extern uint8_t M10P_steer_source;    /* 本帧实际使用的转向来源, 见下 */
/* M10P_steer_source: 0=双侧中线 1=仅左侧跟线 2=仅右侧跟线 3=降级(有回波但不可信) 4=保持(无回波) */
#define M10P_SRC_DUAL   0u
#define M10P_SRC_LEFT   1u
#define M10P_SRC_RIGHT  2u
#define M10P_SRC_WEAK   3u
#define M10P_SRC_HOLD   4u
/* M10P_perception_why 位 */
#define M10P_WHY_FRONT_SEEN 1u   /* 正前方没见过有效点 */
#define M10P_WHY_FRONT_BINS 2u   /* 正前方非空桶不足 */
#define M10P_WHY_FRONT_GAP  4u   /* 正前方连续盲区过大 */
#define M10P_WHY_LEFT_BINS  8u   /* 左侧非空桶不足 */
#define M10P_WHY_RIGHT_BINS 16u  /* 右侧非空桶不足 */
#define M10P_WHY_EPOCH      32u  /* 接收代次与当前不一致 */
#define M10P_WHY_AGE        64u  /* 前方点年龄超限 */
#define M10P_WHY_CAPACITY   128u /* 点云装不下整帧作废 */
void M10P_Poll(void); /* 主循环每圈调一次: 收块 + 喂解析器 + 处理断流/过期/跳变 */
uint16_t M10P_Build(const M10P_Scan *scan, _LEIDA_DATA *out, uint16_t capacity);
/* 纯转换 + 安全判据都在本模块里; 扫描帧由 main 一直持有到用完为止。 */
#endif
