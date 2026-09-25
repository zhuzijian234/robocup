/**
 * @file    m10p.c
 * @brief   M10P 20K 激光雷达解析器 — 字节流 -> 整圈扫描帧
 *
 * ======================== 数据怎么流 ========================
 *
 *   USART2 @512000 (DMA1_Stream5, 循环模式)
 *        |  半传输/全传输中断: 拷贝 512B 到环形队列 + 打时间戳 (DMA.c)
 *        v
 *   M10P_Poll()  取一块 -> M10P_Feed()          [m10p_vehicle.c 调用]
 *        |
 *        v
 *   M10P_Feed: 找 A5 5A 帧头 / 凑满 160B / 校验长度·尾·角度·转速
 *        |
 *        v
 *   decode(): 单包 70 槽 -> 每点角度按 15° 窗口线性摊开 -> append()
 *        |
 *        v
 *   角度回绕到车尾时 boundary(): 结掉本圈 -> 检查(覆盖≈360° / 周期 60~120ms / 转速 3000~6000dps) -> 发布
 *        |
 *        v
 *   M10P_Acquire()/M10P_Release(): 主循环取走用, 用完归还
 *
 * ======================== 各函数速查 ========================
 *   M10P_Feed    字节级组包: A5 找头、A5 5A 校验、Byte2~3==160 校验长度、尾 FA FB、
 *                转速合法(speed<229 拒), 坏了就按"保留后缀里的下一个 A5"重新同步
 *   decode       单包解码: 读角度(Byte4~5)和转速(Byte6~7), 算 dps = 15000000/speed,
 *                70 个槽里 0xFFFF 的算空槽, 其余按 pos = rel + (1500*n + m/2)/m 插值出
 *                每点角度; 顺便用角度增量(应在 14~16° 之间)判丢圈
 *   append       落点进当前扫描帧, 并记录"前方 30°~150° 第一次见到有效点"的时刻 front_us
 *   boundary / begin_scan / coverage
 *                整圈切帧的状态机: 包内越过相对车后切点的 0°（原生180°）时收上一帧、开新帧; 只有覆盖≈360°、
 *                周期 60~120ms、转速 3000~6000dps 才标记为"可发布"
 *   角度坐标变换单独一个 M10P_AlgorithmAngle(9000 - native, 把"0=正前、顺时针"翻成
 *   LXY 的"0=右/90=前/180=左"); 按 0.5° 分桶的 M10P_Index 是给前方盲区和左右侧计数用的。
 *
 * ======================== 几条铁律 ========================
 * 1) 解析全在主循环上下文跑, 中断只搬字节 —— 这里所有 static 变量都不跟 ISR 抢。
 * 2) 双缓冲 + 状态机(0空闲/1写入/2就绪/3读取), 主循环取走一帧期间解析器还能继续写另一帧。
 * 3) 任何一处发现"跳变"(角度不对、时间断档、包坏了)都调 abandon(): 丢掉正在攒的半帧,
 *    宁可这一圈不要, 也不让半圈的拼接数据流到控制里去。
 * 4) 时间戳一律是"字节到达"的时刻, 不是解析时刻; 起点还被故意算早(取 DMA 块起点), 只当保守下界。
 */

#include "m10p.h"
#include <string.h>

/* 编译期断言: 这个结构必须是 8 字节 —— 2048 点的两帧缓冲一共占 32KB RAM,
 * 字段一改这里就编不过, 提醒你重算内存账。 */
typedef char point_must_be_8_bytes[(sizeof(M10P_Point) == 8) ? 1 : -1];
M10P_Stats M10P_stats;
static M10P_Scan scans[2]; /* 双缓冲: 一帧给主循环读, 另一帧给解析器写 */
/* 解析器和扫描帧状态都归主循环上下文所有, ISR 一律不碰这些对象。 */
static uint8_t state[2]; /* 每个缓冲的状态: 0=空闲 1=正在写 2=已就绪待取 3=主循环正在读 */
static int writer;       /* 当前正在写的缓冲下标, -1 = 没有 */
static uint8_t packet[M10P_PACKET_BYTES]; /* 组包暂存区 */
static uint16_t pending, previous_angle;  /* pending=已缓冲字节数; previous_angle=上一包角度(0.01°) */
static uint32_t packet_start_us, previous_end_us, scan_seq; /* 包起点时刻 / 上一包结束时刻 / 帧号计数 */
static uint8_t have_previous; /* 有没有上一包可比对(上电第一包没法判跳变) */

/* 大端读 16 位: M10P 协议里所有多字节字段都是高字节在前 */
static uint16_t be16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] << 8 | p[1]); }

/**
 * @brief  雷达原生角 -> 算法角
 * @param  native_cdeg 原生角, 单位 0.01°, 0=正前方, 顺时针为正
 * @return 算法角, 单位 0.01°, 0=右侧 / 9000=正前方 / 18000=左侧
 *
 * 就是"镜像 + 旋转": 9000 - 原生角。因为雷达顺时针为正, 而 LXY 老流水线要的是
 * 0 在右边的那套。M10P_BETA_CDEG 是安装角补偿, 雷达装歪了改那个宏;
 * 取模是为了把负数掰回 0~35999。
 */
uint16_t M10P_AlgorithmAngle(uint16_t native_cdeg)
{
    int32_t a = 9000 - (int32_t)native_cdeg - M10P_BETA_CDEG;
    a %= 36000;
    if (a < 0) a += 36000;
    return (uint16_t)a;
}

/**
 * @brief  丢掉正在攒的半帧, 回到"没有在写"的状态
 *
 * 状态 3(主循环正在读)的缓冲不动 —— 那帧已经交出去了, 由 M10P_Release 回收。
 * 顺手清 have_previous: 既然已经跳变, 上一包的角度就没有参考价值了。
 */
static void abandon(void)
{
    int i;
    for (i = 0; i < 2; ++i)
        if (state[i] != 3) state[i] = 0;
    writer = -1;
    have_previous = 0;
}

/**
 * @brief  上电初始化: 清统计、清双缓冲状态机
 */
void M10P_Init(void)
{
    memset(&M10P_stats, 0, sizeof M10P_stats);
    memset(state, 0, sizeof state);
    pending = 0; scan_seq = 0; abandon();
}

/**
 * @brief  接收链路出问题时调用(断流、DMA 故障、换代)
 * @param  epoch 出问题时接收链路的代号, 记进统计便于对账
 *
 * 只做两件事: 扔掉半包(下次从帧头重新同步), 记一次"不连续"。
 */
void M10P_Lost(uint32_t epoch)
{
    pending = 0; abandon();
    M10P_stats.epoch = epoch;
    M10P_stats.discontinuities++;
}

/**
 * @brief  开一圈新扫描: 找一个空闲缓冲, 把状态置成"正在写"
 * @param  t 本圈起点时刻(用触发切帧那个包的起点)
 *
 * 两个缓冲都占着时的取舍: 丢"最旧的已就绪帧"(seq 最小的那个), 记一次 ready_drop ——
 * 说明主循环吃得比雷达出得慢。seq 相减后转 int32 再比较, 是为了帧号回绕时比较仍然正确。
 */
static void begin_scan(uint32_t t)
{
    int i;
    writer = -1;
    for (i = 0; i < 2; ++i) if (!state[i]) { writer = i; break; }
    if (writer < 0) {
        for (i = 0; i < 2; ++i) if (state[i] == 2 &&
            (writer < 0 || (int32_t)(scans[i].seq - scans[writer].seq) < 0)) writer = i;
        if (writer >= 0) M10P_stats.ready_drop++;
    }
    if (writer >= 0) {
        M10P_Scan *s = &scans[writer];
        /* 不整帧清点: count 界定了有效范围, 没写到的槽位不会被读到,
         * 省掉 16KB 缓冲每圈清一次的 CPU 开销。 */
        s->start_us = t; s->count = s->coverage_cdeg = s->invalid_slots = 0;
        s->front_seen = s->overflow = s->unstable = 0; s->front_us = t; s->epoch = M10P_stats.epoch;
        state[writer] = 1;
    }
}

/**
 * @brief  结掉当前这圈: 合格就发布(状态置 2), 然后立刻开下一圈
 * @param  start 触发切帧那个包的起点时刻(会成为新一圈的 start_us)
 * @param  end   本圈结束时刻(所在 DMA 块的结束时刻)
 *
 * 发布门槛, 四条全过才算一圈完整数据:
 *   覆盖 35900~36100 (0.01°)  —— 整圈, 允许 ±1° 误差
 *   周期 60~120ms             —— 对应 8.3~16.7Hz
 *   转速 3000~6000 dps        —— 对应 8.3~16.7Hz, 与周期互相印证
 *   没溢出, 且本圈内转速全程稳定
 * 不合格就直接扔(统计里记 rejected), 绝不把半圈或可疑数据发给控制。
 */
static void boundary(uint32_t start, uint32_t end)
{
    if (writer >= 0) {
        M10P_Scan *s = &scans[writer];
        s->end_us = end;
        s->period_us = start - s->start_us; /* 本圈起点到这次切帧点的间隔 = 转一圈的时间 */
        if (!s->overflow && !s->unstable && s->coverage_cdeg >= 35900 && s->coverage_cdeg <= 36100 &&
            s->period_us >= M10P_MIN_PERIOD_US && s->period_us <= M10P_MAX_PERIOD_US &&
            s->dps >= 3000u && s->dps <= 6000u) {
            s->seq = ++scan_seq; state[writer] = 2; M10P_stats.scans++;
        } else {
            state[writer] = 0; M10P_stats.rejected++;
        }
    }
    begin_scan(start);
}

/**
 * @brief  给当前这圈累加角度覆盖与空槽, 并刷新转速
 * @param  span    本包落在本圈里的角度跨度(0.01°), 正常一包 1500 (=15°)
 * @param  invalid 本包里协议判为空槽(0xFFFF)的个数
 * @param  dps     本包算出来的转速(度/秒)
 *
 * 一个包可能被切帧点劈成两半: 切帧前那半算给上一圈, 切帧后那半算给新一圈,
 * 所以本函数对一个包最多被调用两次(见 decode 末尾)。
 * 转速一旦越界就把 unstable 钉住 —— 不能因为最后一包正常, 就把中间飘过的转速洗白。
 */
static void coverage(uint16_t span, uint16_t invalid, uint16_t dps)
{
    if (writer >= 0) {
        M10P_Scan *s = &scans[writer];
        if ((uint32_t)s->coverage_cdeg + span > 65535u) s->overflow = 1;
        else s->coverage_cdeg = (uint16_t)(s->coverage_cdeg + span);
        s->invalid_slots = (uint16_t)(s->invalid_slots + invalid);
        s->dps = dps;
        /* 最后一包正常也不能洗白中间某一包飘掉的转速。 */
        if (dps < 3000u || dps > 6000u) s->unstable = 1;
    }
}

/**
 * @brief  往当前这圈里塞一个测距点
 * @param  a   原生角度(0.01°)
 * @param  raw 槽里的原始 16 位: bit15 = 高反光标志, bit0~14 = 距离(mm)
 * @param  t   本点所在包的起点时刻(字节到达时刻)
 *
 * 两种溢出保护: 点数到顶 M10P_SCAN_CAPACITY; 本圈拖过 250ms(说明切帧丢了,
 * 再攒下去只会拖垮控制周期)。
 * 另外, 本圈正前方(算法角 30°~150°)的第一个有效点会被记下来, 它的时刻就是 front_us ——
 * 上位机判"感知新不新鲜"看的是它, 不是整圈的结束时刻。
 */
static void append(uint16_t a, uint16_t raw, uint32_t t)
{
    M10P_Scan *s;
    M10P_Point *p;
    uint16_t theta;
    if (writer < 0) return;
    s = &scans[writer];
    if (s->count >= M10P_SCAN_CAPACITY || (uint32_t)(t - s->start_us) > 250000u) {
        if (!s->overflow) M10P_stats.scan_overflow++;
        s->overflow = 1; return;
    }
    p = &s->points[s->count++];
    p->angle_cdeg = a; p->range_mm = raw & 0x7fffu;
    p->flags = (uint8_t)(raw >> 15); p->quality = 0;
    p->offset_100us = (uint16_t)((t - s->start_us) / 100u);
    theta = M10P_AlgorithmAngle(a);
    if (!s->front_seen && theta >= 3000 && theta <= 15000 &&
        p->range_mm >= M10P_MIN_MM && p->range_mm <= M10P_MAX_MM) {
        s->front_seen = 1; s->front_us = t;
    }
}

/**
 * @brief  解码一个已经收全并校验通过的包
 * @param  end_us 本包结束时刻(用所在 DMA 块的结束时刻)
 *
 * 包布局: [0]A5 [1]5A [2~3]长度(=160) [4~5]角度 [6~7]转速
 *         [8~147] 70 槽 × 2 字节 [148~157]保留 [158~159]FA FB 尾
 *
 * 关键三步:
 * 1) 跳变检测 —— 相邻两包角度必须差 14°~16°(正常 15°), 且间隔不超过 20ms;
 *    不满足就当"丢圈/乱码", 记一次不连续并丢掉正在攒的帧。
 * 2) 角度摊开 —— 把 m 个有效点均匀摊在 [rel, rel+1500) 这 15° 上(空槽跳过、不留洞)。
 *    注意这是近似: 丢的点越多, 剩下的点被摊得越开, 与真实角度偏得越多。
 * 3) 切帧 —— 摊点过程中一旦越过回绕点(车身正后方, 原生 180°)就结掉本圈、开新圈。
 *
 * 那段 +18000 / -18000 的平移, 是为了让"以包角度为起点的 15° 窗口"统一落在坐标中间,
 * 不必在窗口跨过 0° 时把它拆成两段算。
 */
static void decode(uint32_t end_us)
{
    uint16_t a = be16(packet + 4), speed = be16(packet + 6), m = 0, i, n = 0;
    uint16_t rel, before, dps, raw, delta;
    uint8_t crossed = 0;
    if (a == 36000) a = 0;
    if (have_previous) {
        delta = (uint16_t)((a + 36000u - previous_angle) % 36000u);
        /* 只看角度判不出"整整少转一圈", 所以要再加时间的约束。 */
        if (delta < 1400 || delta > 1600 || (uint32_t)(end_us - previous_end_us) > 20000u) {
            M10P_stats.discontinuities++; abandon();
        }
    }
    previous_angle = a; previous_end_us = end_us; have_previous = 1;
    for (i = 0; i < 70; ++i) if (be16(packet + 8 + i * 2) != 0xffffu) ++m;
    M10P_stats.invalid_slots += 70 - m;
    dps = (uint16_t)(15000000u / speed); /* 协议给的转速字段 -> 度/秒 */
    rel = (uint16_t)((a + 18000u) % 36000u);
    /* before = 本包这 15° 里落在回绕点之前的那部分, 算在上一圈账上 */
    before = rel + 1500u >= 36000u ? (uint16_t)(36000u - rel) : 1500u;
    coverage(before, (uint16_t)(70 - m), dps);
    for (i = 0; i < 70; ++i) {
        uint32_t pos;
        raw = be16(packet + 8 + i * 2);
        if (raw == 0xffffu) continue;
        /* 空槽计数按"协议上有效"算, 在测距筛选之前 —— 不能拿距离过滤后的数当丢点率。 */
        pos = rel + (1500u * n + m / 2u) / m; ++n;
        if (pos >= 36000u && !crossed) {
            boundary(packet_start_us, end_us); crossed = 1;
        }
        if (raw & 0x8000u) M10P_stats.high_reflect++;
        append((uint16_t)((pos + 18000u) % 36000u), raw, packet_start_us);
    }
    if (rel + 1500u >= 36000u) {
        /* 这一包跨了回绕点: 若摊点时没触发切帧(整包全空槽), 这里补切一次;
         * 后半段的角度覆盖记到新开的这圈上(空槽数前面已经记过, 这里传 0)。 */
        if (!crossed) boundary(packet_start_us, end_us);
        coverage((uint16_t)(1500u - before), 0, dps);
    }
}

/**
 * @brief  喂一块 DMA 数据进来(M10P_Poll 每取到一块就调一次)
 * @param  data     块首地址
 * @param  count    块长度
 * @param  start_us 这块第一个字节的到达时刻(含 DMA 中断延迟, 只会偏早)
 * @param  end_us   这块最后一个字节的到达时刻
 *
 * 逐字节组包, 四种情况判坏包: 第 2 字节不是 5A、长度字段不是 160、尾不是 FA FB、
 * 角度 > 36000、转速字段太小(会让 dps 溢出 uint16)。
 * 坏包不整块丢: 在已缓冲的字节里找下一个 A5 当头, 把后半段 memmove 到前面接着用
 * (帧头可能就藏在坏包中间); 找不到 A5 就清空, 等下一块。
 * 残留后缀沿用它原来的包起点时刻 —— 那是更早的时刻, 对"时间戳只许偏早"的约定是安全的。
 */
void M10P_Feed(const uint8_t *data, size_t count, uint32_t start_us, uint32_t end_us)
{
    size_t i;
    M10P_stats.bytes += (uint32_t)count;
    for (i = 0; i < count; ++i) {
        uint16_t skip;
        uint8_t bad = 0;
        if (!pending) {
            if (data[i] != 0xa5) continue; /* 没在组包: 一路丢字节, 直到看见帧头 */
            packet_start_us = start_us;
        }
        packet[pending++] = data[i];
        if (pending >= 2 && packet[1] != 0x5a) bad = 1;
        if (pending >= 4 && be16(packet + 2) != M10P_PACKET_BYTES) {
            M10P_stats.bad_length++; bad = 1;
        }
        if (!bad && pending < M10P_PACKET_BYTES) continue; /* 还没凑够 160 字节, 继续攒 */
        if (!bad) {
            uint16_t speed = be16(packet + 6);
            if (packet[158] != 0xfa || packet[159] != 0xfb) { M10P_stats.bad_tail++; bad = 1; }
            if (be16(packet + 4) > 36000) { M10P_stats.bad_angle++; bad = 1; }
            /* 转速字段太小会让 dps 溢出 uint16 存不下, 这种包直接判坏;
             * 上电时转速还没稳(超出周期窗口)的包不在这里拦, 交给整圈门槛去筛。 */
            if (speed < 229u) { M10P_stats.bad_speed++; bad = 1; }
        }
        if (!bad) {
            M10P_stats.packets++; decode(end_us); pending = 0;
        } else {
            if (have_previous) { M10P_stats.discontinuities++; abandon(); }
            for (skip = 1; skip < pending && packet[skip] != 0xa5; ++skip) {}
            pending = (uint16_t)(pending - skip);
            if (pending) memmove(packet, packet + skip, pending);
            /* 留下的后缀继续用它原来的(更早的)起点时刻, 时间戳只会偏早。 */
        }
    }
}

/**
 * @brief  取一帧已就绪的扫描; 没有就返回 0
 * @return 扫描帧指针(状态已置成"正在读"), 用完必须 M10P_Release()
 *
 * 取 seq 最大的那帧(最新); 若另一帧也是就绪态就顺手丢掉(记 ready_drop):
 * 主循环一次只吃一帧, 留着只会把缓冲占死。
 */
const M10P_Scan *M10P_Acquire(void)
{
    int i, best = -1;
    for (i = 0; i < 2; ++i) if (state[i] == 2 &&
        (best < 0 || (int32_t)(scans[i].seq - scans[best].seq) > 0)) best = i;
    if (best < 0) return 0;
    for (i = 0; i < 2; ++i) if (i != best && state[i] == 2) {
        state[i] = 0; M10P_stats.ready_drop++;
    }
    state[best] = 3; return &scans[best];
}

/**
 * @brief  归还扫描帧, 缓冲回到空闲, 给下一圈复用
 * @param  scan M10P_Acquire 返回的那个指针
 */
void M10P_Release(const M10P_Scan *scan)
{
    int i;
    for (i = 0; i < 2; ++i) if (scan == &scans[i] && state[i] == 3) state[i] = 0;
}

/**
 * @brief  把一帧扫描按 0.5° 分桶, 供"前方盲区 / 左右侧点数"这类统计使用
 * @param  scan 已取的扫描帧
 * @param  bins 输出: bins[i] = 该桶里最近那个点在 scan->points[] 里的下标, 空桶 = 0xFFFF
 * @return 非空桶的个数
 *
 * 桶下标 i 对应的算法角 = i × 0.5°(0=右侧; 140~220 = 正前方 70°~110°)。
 * 一个桶里落了多个点时保留"最近"的那个 —— 循线关心的是最近的墙, 不是最远的。
 * 超出 M10P_MIN_MM ~ M10P_MAX_MM 的点在这里就被剔掉, 不占桶。
 */
uint16_t M10P_Index(const M10P_Scan *scan, uint16_t bins[M10P_BINS])
{
    uint16_t i, count = 0;
    for (i = 0; i < M10P_BINS; ++i) bins[i] = 0xffffu;
    for (i = 0; i < scan->count; ++i) {
        uint16_t b, old;
        const M10P_Point *p = &scan->points[i];
        if (p->range_mm < M10P_MIN_MM || p->range_mm > M10P_MAX_MM) continue;
        b = M10P_AlgorithmAngle(p->angle_cdeg) / 50u; old = bins[b];
        if (old == 0xffffu) { bins[b] = i; count++; }
        else if (p->range_mm < scan->points[old].range_mm) bins[b] = i;
    }
    return count;
}
