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
 *      -> M10P_perception_ok(能不能信) / M10P_speed_scale(该跑多快)
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
static uint32_t seen_epoch, seen_discontinuities, seen_rejected; /* 本地快照, 用来发现"统计又涨了" */
uint32_t M10P_control_seq, M10P_control_front_us, M10P_control_epoch; /* 本帧的来源标记, 给遥测对账用 */
uint16_t M10P_front_bins, M10P_left_bins, M10P_right_bins; /* 三个扇区的点数(按桶数算) */
float M10P_clearance_mm, M10P_speed_scale; /* 正前方走廊净空(mm) / 限速系数 0.25~1 */
uint8_t M10P_perception_ok;                /* 本帧感知是否可信(0=不可信, 控制与电机都会让路) */

/**
 * @brief  取数据 + 喂解析器(主循环每圈开头调一次)
 *
 * 四件事, 按顺序:
 * 1) 接收链路换代(断流/DMA 故障/复位) -> 通知解析器丢半帧, 并作废电机许可
 * 2) LidarRx_Service() 处理 DMA 故障(重启流、清队列)
 * 3) 每圈最多取 2 块喂进去 —— 主循环没数据时是空转的, 调得足够勤, 不必一次吃空队列
 * 4) 链路丢块、解析器丢帧(discontinuities/rejected 涨了) -> 作废电机许可, 宁可停也不赌
 *
 * 过期判断用的是 stamp.start_us(字节到达时刻)而不是处理时刻: 队列里压了太久的块
 * 直接扔掉, 免得拿旧数据去驱动车。
 */
void M10P_Poll(void)
{
    unsigned budget = 2;
    LidarRxStamp stamp;
    uint16_t n;
    if (seen_epoch != LidarRx_epoch) {
        seen_epoch = LidarRx_epoch;
        M10P_Lost(seen_epoch); Radar_Invalidate();
    }
    LidarRx_Service();
    while (budget-- && (n = LidarRx_Read(rx, sizeof rx, &stamp)) != 0) {
        if (stamp.epoch != seen_epoch) {
            seen_epoch = stamp.epoch; M10P_Lost(seen_epoch); Radar_Invalidate();
        }
        if ((uint32_t)(Diag_TimeUs() - stamp.start_us) > M10P_MAX_AGE_US) {
            M10P_Lost(seen_epoch); Radar_Invalidate(); continue;
        }
        M10P_Feed(rx, n, stamp.start_us, stamp.end_us);
    }
    if (seen_discontinuities != M10P_stats.discontinuities || seen_rejected != M10P_stats.rejected) {
        /* 解析器只要丢过帧/丢过圈, 就认为眼前这帧的几何关系不可信:
         * 作废一次电机许可, 逼主循环重新用新鲜数据把许可挣回来。 */
        seen_discontinuities = M10P_stats.discontinuities;
        seen_rejected = M10P_stats.rejected;
        Radar_Invalidate();
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
 * 第二段是另一条独立的安全判据: 用**全部原始点**(不受 100mm 下限过滤影响, 因为
 * 贴在车头的东西更要命)算正前方走廊(|x| < 180mm, y > 0)里最近的障碍距离,
 * 存进 M10P_clearance_mm。后半圈的点 y <= 0, 不可能落进走廊, 直接跳过省时间。
 * 最后综合出感知可信标志与限速系数: 净空 350mm 以下判不可信(= 停车线),
 * 350~1000mm 之间线性降速, 最慢保底 0.25 倍。
 */
uint16_t M10P_Build(const M10P_Scan *scan, _LEIDA_DATA *out, uint16_t capacity)
{
    uint16_t i, n = 0, missing = 0, max_missing = 0;
    M10P_front_bins = M10P_left_bins = M10P_right_bins = 0;
    M10P_clearance_mm = (float)M10P_MAX_MM; /* 先当"啥也没看见", 下面扫到更近的再改 */
    M10P_perception_ok = 0; M10P_speed_scale = 0;
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
    for (i = 0; i < M10P_BINS; ++i) if (bins[i] != 0xffffu) {
        const M10P_Point *p = &scan->points[bins[i]];
        float a = M10P_AlgorithmAngle(p->angle_cdeg) / 100.0f; /* 0.01° -> 度 */
        if (n >= capacity) return 0; /* 装不下就整帧作废, 不喂半截数据 */
        out[n].angle = a; out[n++].distance = p->range_mm;
        if (i >= 140 && i <= 220) M10P_front_bins++;
        if (i >= 220 && i <= 340) M10P_left_bins++;
        if (i >= 20 && i <= 140) M10P_right_bins++;
    }
    /* 独立的近障碍判据: 用全部原始回波, 不走"墙点"那套过滤。
     * 近处(<100mm)的非零点按"可能挡路"保守计入。 */
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
    /* 感知可信的全部条件, 缺一不可 */
    M10P_perception_ok = scan->front_seen && M10P_front_bins >= 40 &&
        max_missing <= M10P_FRONT_MAX_MISSING_BINS &&
        M10P_left_bins >= 16 && M10P_right_bins >= 16 &&
        scan->epoch == LidarRx_epoch &&
        (uint32_t)(Diag_TimeUs() - scan->front_us) <= M10P_MAX_AGE_US &&
        M10P_clearance_mm > M10P_STOP_Y_MM;
    if (M10P_perception_ok) {
        /* 净空越近越慢: 350mm -> 0.25 倍, 1000mm 及以上 -> 满速 */
        M10P_speed_scale = (M10P_clearance_mm - M10P_STOP_Y_MM) / (M10P_SLOW_Y_MM - M10P_STOP_Y_MM);
        if (M10P_speed_scale > 1) M10P_speed_scale = 1;
        if (M10P_speed_scale < 0.25f) M10P_speed_scale = 0.25f;
    }
    return n;
}
