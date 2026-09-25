/**
 * @file    m10p.h
 * @brief   M10P 20K 激光雷达驱动 — 协议常量、数据结构与对外接口
 *
 * ======================== 这个模块干嘛的 ========================
 * 把 USART2 + DMA 收到的 M10P 原始字节流, 解析成"一圈一圈的扫描帧"(M10P_Scan),
 * 再交给 m10p_vehicle.c 转成 LXY 老流水线认识的极坐标数组。
 *
 * 分工:
 *   DMA.c          只管搬字节(中断里拷贝 + 打时间戳)
 *   m10p.c         只管解析(Framing / 解码 / 整圈切帧 / 双缓冲), 全在主循环上下文
 *   m10p_vehicle.c 只管适配(扫描帧 -> LEIDA_DATA2[] + 健康度判定)
 *   main.c         只管用(M10P_Poll -> Acquire -> Build -> ... -> Release)
 *
 * M10P 20K 串口 V2.0 版协议(只认这一种): 每包 160 字节, 70 个测距槽。
 * 雷达原生的角度系: 0° = 正前方, 顺时针为正。
 * 本工程对外(LXY 老流水线)用的是另一套: 0° = 右侧, 90° = 正前方, 180° = 左侧。
 * 两套之间的换算统一走 M10P_AlgorithmAngle(), 别在别处自己转。
 */
#ifndef M10P_H
#define M10P_H
#include <stdint.h>
#include <stddef.h>

#include "m10p_config.h" /* 协议与车辆包络参数集中维护 */

/* ======================== 数据结构 ======================== */

/* 单个测距点(协议里一个槽 2 字节解出来的东西) */
typedef struct {
    uint16_t angle_cdeg, range_mm, offset_100us;
    /* angle_cdeg   角度, 单位 0.01°, 雷达原生系(0=正前, 顺时针为正), 范围 0~36000
     * range_mm     距离, 单位 mm(bits15 的高反光标志已剥掉, 存这里的就是纯距离)
     * offset_100us 本点相对"本圈起点"的到达时刻, 单位 100us, 用于发现帧内时间跳变 */
    uint8_t quality, flags; /* quality=0: 协议没给质量字段, 恒为 0; flags bit0 = 高反光 */
} M10P_Point;

/* 一圈完整的扫描帧(解析器的产物, 主循环一次只吃一帧) */
typedef struct {
    M10P_Point points[M10P_SCAN_CAPACITY]; /* 点云, 按收到顺序追加, 有效长度看 count */
    uint32_t seq, start_us, end_us, period_us, front_us, epoch;
    /* seq        递增帧号, 用来判"有没有新证据"(不连续说明中间掉过帧)
     * start_us   本圈第一个字节的到达时刻(含 DMA 中断延迟, 只作保守下界)
     * end_us     收满本圈、切帧那一刻的时刻
     * period_us  本圈周期 = start_us - 上一圈的 start_us, 60~120ms 才算合法
     * front_us   本圈"正前方第一次看到有效点"的时刻, 感知新鲜度就看它
     * epoch      接收链路的"代"号, DMA 故障/复位会 +1, 换代即作废 */
    uint16_t count, coverage_cdeg, dps, invalid_slots;
    /* count         本圈实际存下的点数
     * coverage_cdeg 本圈累计覆盖角度(0.01°); 36000 附近才算整圈
     * dps           本圈最后读到的转速(度/秒), 6Hz = 2160, 10Hz = 3600
     * invalid_slots 本圈协议判为空槽的个数(0xFFFF), 衡量丢点程度 */
    uint8_t front_seen, overflow, unstable;
    /* front_seen 本圈正前方有没有见过有效点
     * overflow   点数超容 或 本圈拖过 250ms -> 置 1, 该帧不发布
     * unstable   本圈内出现过超范围转速 -> 置 1, 该帧不发布 */
} M10P_Scan;

/* 诊断计数(只累加, 不参与控制; 出问题时看哪个在涨就知道坏在哪一环) */
typedef struct {
    uint32_t bytes, packets, bad_length, bad_tail, bad_angle, bad_speed;
    /* bytes 收到的总字节; packets 通过校验的包数
     * bad_length 长度字段不对; bad_tail 尾字节不是 FA FB
     * bad_angle  角度字段 > 36000; bad_speed 转速字段过小(会让 dps 溢出 uint16) */
    uint32_t invalid_slots, high_reflect, discontinuities, scans, rejected;
    /* invalid_slots 累计空槽数; high_reflect 累计高反光点
     * discontinuities 角度/时间跳变次数(= 丢圈或乱码, 会丢半帧重新同步)
     * scans 成功发布的扫描帧数; rejected 因为不满足整圈/周期/转速条件被丢掉的帧数 */
    uint32_t scan_overflow, ready_drop, epoch;
    /* scan_overflow 点数或时长超限的次数; ready_drop 双缓冲都被占、被迫丢帧的次数
     * epoch 最近一次接收故障时的代次 */
} M10P_Stats;
extern M10P_Stats M10P_stats;

/* ======================== 对外接口 ======================== */
void M10P_Init(void);                 /* 上电初始化: 清统计、清双缓冲状态机 */
void M10P_Lost(uint32_t epoch);       /* 接收链路出问题(断流/DMA故障/换代)时调用: 丢半帧 + 记一次不连续 */
/* 时间戳是"字节到达"的时刻, 不是解析时刻; start_us 还含 DMA 中断延迟, 只当保守下界用。 */
void M10P_Feed(const uint8_t *data, size_t count, uint32_t start_us, uint32_t end_us);
const M10P_Scan *M10P_Acquire(void);  /* 取一帧 ready 的扫描(没有就返回 0); 用完必须 Release */
void M10P_Release(const M10P_Scan *scan); /* 归还扫描帧, 让缓冲能被下一圈复用 */
uint16_t M10P_AlgorithmAngle(uint16_t native_cdeg); /* 原生角(0.01°) -> 算法角(0.01°, 0=右/9000=前/18000=左) */
/* 按 0.5° 把一帧扫进 bins[]: 桶里存的是点在 scan->points[] 里的下标, 空桶 = 0xFFFF;
 * 返回非空桶个数。桶下标 i 对应的角度 = i*0.5°(算法系)。 */
uint16_t M10P_Index(const M10P_Scan *scan, uint16_t bins[M10P_BINS]);
#endif
