#include "m10p.h"
#include <string.h>

typedef char point_must_be_8_bytes[(sizeof(M10P_Point) == 8) ? 1 : -1];
M10P_Stats M10P_stats;
static M10P_Scan scans[2];
/* Main context owns parser AND scan state; ISR never writes these objects. */
static uint8_t state[2]; /* 0 free, 1 writing, 2 ready, 3 reading */
static int writer;
static uint8_t packet[M10P_PACKET_BYTES];
static uint16_t pending, previous_angle;
static uint32_t packet_start_us, previous_end_us, scan_seq;
static uint8_t have_previous;

static uint16_t be16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] << 8 | p[1]); }
uint16_t M10P_AlgorithmAngle(uint16_t native_cdeg)
{
    int32_t a = 9000 - (int32_t)native_cdeg - M10P_BETA_CDEG;
    a %= 36000;
    if (a < 0) a += 36000;
    return (uint16_t)a;
}
static void abandon(void)
{
    int i;
    for (i = 0; i < 2; ++i)
        if (state[i] != 3) state[i] = 0;
    writer = -1;
    have_previous = 0;
}
void M10P_Init(void)
{
    memset(&M10P_stats, 0, sizeof M10P_stats);
    memset(state, 0, sizeof state);
    pending = 0; scan_seq = 0; abandon();
}
void M10P_Lost(uint32_t epoch)
{
    pending = 0; abandon();
    M10P_stats.epoch = epoch;
    M10P_stats.discontinuities++;
}
static void begin_scan(uint32_t t)
{
    int i;
    writer = -1;
    for (i = 0; i < 2; ++i) if (!state[i]) { writer = i; break; }
    if (writer < 0) {
        for (i = 0; i < 2; ++i) if (state[i] == 2) {
            writer = i; M10P_stats.ready_drop++; break;
        }
    }
    if (writer >= 0) {
        M10P_Scan *s = &scans[writer];
        /* No full-point memset: count defines the initialized range. */
        s->start_us = t; s->count = s->coverage_cdeg = s->invalid_slots = 0;
        s->front_seen = s->overflow = 0; s->epoch = M10P_stats.epoch;
        state[writer] = 1;
    }
}
static void boundary(uint32_t start, uint32_t end)
{
    if (writer >= 0) {
        M10P_Scan *s = &scans[writer];
        s->end_us = end;
        s->period_us = start - s->start_us;
        if (!s->overflow && s->coverage_cdeg >= 35900 && s->coverage_cdeg <= 36100 &&
            s->period_us >= M10P_MIN_PERIOD_US && s->period_us <= M10P_MAX_PERIOD_US) {
            s->seq = ++scan_seq; state[writer] = 2; M10P_stats.scans++;
        } else {
            state[writer] = 0; M10P_stats.rejected++;
        }
    }
    begin_scan(start);
}
static void coverage(uint16_t span, uint16_t invalid, uint16_t dps)
{
    if (writer >= 0) {
        M10P_Scan *s = &scans[writer];
        if ((uint32_t)s->coverage_cdeg + span > 65535u) s->overflow = 1;
        else s->coverage_cdeg = (uint16_t)(s->coverage_cdeg + span);
        s->invalid_slots = (uint16_t)(s->invalid_slots + invalid);
        s->dps = dps;
    }
}
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
static void decode(uint32_t end_us)
{
    uint16_t a = be16(packet + 4), speed = be16(packet + 6), m = 0, i, n = 0;
    uint16_t rel, before, dps, raw, delta;
    uint8_t crossed = 0;
    if (a == 36000) a = 0;
    if (have_previous) {
        delta = (uint16_t)((a + 36000u - previous_angle) % 36000u);
        /* Angle alone cannot detect a lost full revolution. Also bound time. */
        if (delta < 1400 || delta > 1600 || (uint32_t)(end_us - previous_end_us) > 20000u) {
            M10P_stats.discontinuities++; abandon();
        }
    }
    previous_angle = a; previous_end_us = end_us; have_previous = 1;
    for (i = 0; i < 70; ++i) if (be16(packet + 8 + i * 2) != 0xffffu) ++m;
    M10P_stats.invalid_slots += 70 - m;
    dps = (uint16_t)(15000000u / speed);
    rel = (uint16_t)((a + 18000u) % 36000u);
    before = rel + 1500u >= 36000u ? (uint16_t)(36000u - rel) : 1500u;
    coverage(before, (uint16_t)(70 - m), dps);
    for (i = 0; i < 70; ++i) {
        uint32_t pos;
        raw = be16(packet + 8 + i * 2);
        if (raw == 0xffffu) continue;
        /* Count protocol-valid slots BEFORE range filtering. */
        pos = rel + (1500u * n + m / 2u) / m; ++n;
        if (pos >= 36000u && !crossed) {
            boundary(packet_start_us, end_us); crossed = 1;
        }
        if (raw & 0x8000u) M10P_stats.high_reflect++;
        append((uint16_t)((pos + 18000u) % 36000u), raw, packet_start_us);
    }
    if (rel + 1500u >= 36000u) {
        if (!crossed) boundary(packet_start_us, end_us);
        coverage((uint16_t)(1500u - before), 0, dps);
    }
}
void M10P_Feed(const uint8_t *data, size_t count, uint32_t start_us, uint32_t end_us)
{
    size_t i;
    M10P_stats.bytes += (uint32_t)count;
    for (i = 0; i < count; ++i) {
        uint16_t skip;
        uint8_t bad = 0;
        if (!pending) {
            if (data[i] != 0xa5) continue;
            packet_start_us = start_us;
        }
        packet[pending++] = data[i];
        if (pending >= 2 && packet[1] != 0x5a) bad = 1;
        if (pending >= 4 && be16(packet + 2) != M10P_PACKET_BYTES) {
            M10P_stats.bad_length++; bad = 1;
        }
        if (!bad && pending < M10P_PACKET_BYTES) continue;
        if (!bad) {
            uint16_t speed = be16(packet + 6);
            if (packet[158] != 0xfa || packet[159] != 0xfb) { M10P_stats.bad_tail++; bad = 1; }
            if (be16(packet + 4) > 36000) { M10P_stats.bad_angle++; bad = 1; }
            /* Reject values that would overflow dps; startup outside scan period
             * remains diagnostic-only through the scan-period gate. */
            if (speed < 229u) { M10P_stats.bad_speed++; bad = 1; }
        }
        if (!bad) {
            M10P_stats.packets++; decode(end_us); pending = 0;
        } else {
            if (have_previous) { M10P_stats.discontinuities++; abandon(); }
            for (skip = 1; skip < pending && packet[skip] != 0xa5; ++skip) {}
            pending = (uint16_t)(pending - skip);
            if (pending) memmove(packet, packet + skip, pending);
            /* Retained suffix uses old conservative start bound. */
        }
    }
}
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
void M10P_Release(const M10P_Scan *scan)
{
    int i;
    for (i = 0; i < 2; ++i) if (scan == &scans[i] && state[i] == 3) state[i] = 0;
}
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
