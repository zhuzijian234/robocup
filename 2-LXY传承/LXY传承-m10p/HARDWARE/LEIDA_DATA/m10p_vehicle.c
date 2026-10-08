/**
 * @file    m10p_vehicle.c
 * @brief   M10P 与整车之间的适配层 — 取数据、转数据、判"这帧能不能信"
 *
 * ======================== 这个模块干嘛的 ========================
 * 上游是 m10p.c(纯协议解析, 只认识字节), 下游是 LXY 老流水线(只认识"角度+距离"的
 * 极坐标数组和一堆 HANDLE 函数)。中间这一层做三件事:
 *
 *   1) M10P_Poll()  把 DMA 环形队列里的块取出来喂给解析器, 顺手处理断流/过期/跳变
 *   2) M10P_Build() 把一帧扫描转成 LEIDA_DATA2[](算法角系: 0=右, 90=前, 180=左)
 *   3) 判健康度: 正前方有没有盲区、左右侧够不够点数、正前方走廊净空多少
 *      -> M10P_perception_ok(是否更新转向)，运行速度由固定目标决定
 *
 * main.c 的用法: M10P_Poll() -> M10P_Acquire() -> M10P_Build() -> ... -> M10P_Release()。
 * 扫描帧在 Build 期间一直由 main 持有, 所以这里的转换逻辑是"只读"的, 不碰缓冲状态。
 */

#include "m10p_vehicle.h"
#include "DMA.h"
#include "ble_diag.h"
#include "timer.h"
#include <float.h>
static uint16_t bins[M10P_BINS];   /* 0.5° 分桶结果: 桶内是点下标, 空桶 0xFFFF */
static uint8_t rx[LIDAR_RX_BLOCK]; /* 从 DMA 队列取块的暂存区(一块 512B) */
static uint32_t seen_epoch; /* 接收代次变化时丢弃未完成扫描 */
uint32_t M10P_control_seq, M10P_control_front_us, M10P_control_epoch; /* 本帧的来源标记, 给遥测对账用 */
uint16_t M10P_front_bins, M10P_left_bins, M10P_right_bins; /* 三个扇区的点数(按桶数算) */
float M10P_clearance_mm; /* 正前方走廊净空(mm)，仅供诊断，不控制车速 */
uint8_t M10P_perception_ok;                /* 本帧感知是否可信(0=不更新转向，电机继续闭环) */
uint16_t M10P_front_gap_bins; /* 前方最大连续空桶数，每桶0.5度 */
uint32_t M10P_build_age_us;   /* 感知检查时，前方观测已经过去的微秒数 */
uint8_t M10P_front_seen, M10P_build_epoch_ok;
uint8_t M10P_front_ok, M10P_left_ok, M10P_right_ok; /* 三个扇区各自达标情况 */
uint8_t M10P_perception_why = M10P_WHY_CAPACITY;    /* 判据失败原因位图 */
uint8_t M10P_steer_source = M10P_SRC_HOLD;          /* 本帧实际使用的转向来源(main.c 写) */

/**
 * @brief  取数据 + 喂解析器(主循环每圈开头调一次)
 *
 * 接收代次变化或块过期时丢弃不完整扫描；每次最多处理两个DMA块。
 * 时间戳取接收时刻，避免积压数据被当成新数据。此处不影响电机。
 */
void M10P_Poll(void)
{
    unsigned budget = 2;
    LidarRxStamp stamp;
    uint16_t n;
    if (seen_epoch != LidarRx_epoch) {
        seen_epoch = LidarRx_epoch;
        M10P_Lost(seen_epoch);
    }
    LidarRx_Service();
    while (budget-- && (n = LidarRx_Read(rx, sizeof rx, &stamp)) != 0) {
        if (stamp.epoch != seen_epoch) {
            seen_epoch = stamp.epoch; M10P_Lost(seen_epoch);
        }
        if ((uint32_t)(Diag_TimeUs() - stamp.start_us) > M10P_MAX_AGE_US) {
            M10P_Lost(seen_epoch); continue;
        }
        M10P_Feed(rx, n, stamp.start_us, stamp.end_us);
    }

}

/**
 * @brief  把一帧扫描转成 LXY 的极坐标数组, 同时算健康度
 * @param  scan     已取的扫描帧(只读)
 * @param  out      输出数组, 元素是 _LEIDA_DATA{角度(度), 距离(mm)}
 * @param  capacity out 能装多少点(一般是 LEIDA_DATA_COUNTER = 800)
 * @return 实际写入的点数; 装不下时返回 0(整帧作废, 不截断)
 *
 * 三个扇区(桶下标 -> 算法角):
 *   右侧 20~140   = 10°~70°
 *   正前 140~220  = 70°~110°  (还要单独查"最长的连续空桶", 即盲区弧长)
 *   左侧 220~340  = 110°~170°
 *
 * 输出顺序按桶下标递增, 也就是按角度从小到大排好, 下游的 HANDLE 直接按顺序扫。
 * 注意: 装满 capacity 时返回 0 —— 宁可这帧不用, 也不要半个数组喂给控制。
 *
 * 全部原始点另用于计算前向走廊净空，仅供诊断，不参与转向有效性或电机控制。
 */
uint16_t M10P_Build(const M10P_Scan *scan, _LEIDA_DATA *out, uint16_t capacity)
{
    uint16_t i, n = 0, missing = 0, max_missing = 0;
    M10P_front_bins = M10P_left_bins = M10P_right_bins = 0;
    M10P_clearance_mm = (float)M10P_MAX_MM; /* 先当"啥也没看见", 下面扫到更近的再改 */
    M10P_perception_ok = 0;
    M10P_front_gap_bins = 0;
    /* 提前 return(装不下)时留下确定的原因位, 不沿用上一帧的结果。 */
    M10P_front_ok = M10P_left_ok = M10P_right_ok = 0;
    M10P_perception_why = M10P_WHY_CAPACITY;
    M10P_front_seen = scan->front_seen;
    M10P_build_epoch_ok = scan->epoch == LidarRx_epoch;
    M10P_build_age_us = (uint32_t)(Diag_TimeUs() - scan->front_us);
    M10P_control_seq = scan->seq;
    M10P_control_front_us = scan->front_us;
    M10P_control_epoch = scan->epoch;
    M10P_Index(scan, bins);
    /* 总点数够多不代表正前方没瞎 —— 单独量一下最长的连续空桶。 */
    for (i = 140; i <= 220; ++i) {
        if (bins[i] == 0xffffu) {
            if (++missing > max_missing) max_missing = missing;
        } else missing = 0;
    }
    M10P_front_gap_bins = max_missing;
    for (i = 0; i < M10P_BINS; ++i) if (bins[i] != 0xffffu) {
        const M10P_Point *p = &scan->points[bins[i]];
        float a = M10P_AlgorithmAngle(p->angle_cdeg) / 100.0f; /* 0.01° -> 度 */
        if (n >= capacity) return 0; /* 装不下就整帧作废, 不喂半截数据 */
        out[n].angle = a; out[n++].distance = p->range_mm;
        if (i >= 140 && i <= 220) M10P_front_bins++;
        if (i >= 220 && i <= 340) M10P_left_bins++;
        if (i >= 20 && i <= 140) M10P_right_bins++;
    }
    /* 诊断净空：包含100mm以内的非零原始回波，不作为停车条件。 */
    for (i = 0; i < scan->count; ++i) {
        const M10P_Point *p = &scan->points[i];
        uint16_t theta;
        float angle, x, y;
        if (!p->range_mm || p->range_mm > M10P_MAX_MM) continue;
        theta = M10P_AlgorithmAngle(p->angle_cdeg);
        /* 后半圈 y <= 0, 不可能落进正前方走廊, 直接跳过。
         * 两侧保留完整判断, 让浮点边界行为偏保守。 */
        if (theta > 18000u) continue;
        angle = theta * (PI / 18000.0f); /* 0.01° -> 弧度 */
        x = p->range_mm * arm_cos_f32(angle);
        y = p->range_mm * arm_sin_f32(angle);
        if (y > 0 && fabsf(x) < M10P_CORRIDOR_HALF_MM && y < M10P_clearance_mm)
            M10P_clearance_mm = y;
    }
    /* 感知分级: 三个扇区分别判定, 再合成总判据。
     * 分级的意义: 锥桶赛道经常只有单侧可见, 旧的二值判据会整帧作废并冻结舵角;
     * 现在把"能不能用哪种来源"告诉 main.c, 由它决定双侧中线 / 单侧跟线 / 降级。 */
    M10P_build_age_us = (uint32_t)(Diag_TimeUs() - scan->front_us);
    M10P_build_epoch_ok = scan->epoch == LidarRx_epoch;
    M10P_front_ok = (M10P_front_bins >= M10P_FRONT_MIN_BINS) &&
                    (max_missing <= M10P_FRONT_MAX_MISSING_BINS);
    M10P_left_ok = M10P_left_bins >= M10P_SIDE_MIN_BINS;
    M10P_right_ok = M10P_right_bins >= M10P_SIDE_MIN_BINS;
    {
        uint8_t why = 0;
        if (!scan->front_seen) why |= M10P_WHY_FRONT_SEEN;
        if (M10P_front_bins < M10P_FRONT_MIN_BINS) why |= M10P_WHY_FRONT_BINS;
        if (max_missing > M10P_FRONT_MAX_MISSING_BINS) why |= M10P_WHY_FRONT_GAP;
        if (!M10P_left_ok) why |= M10P_WHY_LEFT_BINS;
        if (!M10P_right_ok) why |= M10P_WHY_RIGHT_BINS;
        if (scan->epoch != LidarRx_epoch) why |= M10P_WHY_EPOCH;
        if (M10P_build_age_us > M10P_MAX_AGE_US) why |= M10P_WHY_AGE;
        M10P_perception_why = why;
        M10P_perception_ok = (uint8_t)(why == 0u);
    }
    return n;
}
