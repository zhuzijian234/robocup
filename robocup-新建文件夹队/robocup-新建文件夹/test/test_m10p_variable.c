/* 变长组包回归：直接运行生产解析器，硬件时间用可控输入替代。 */
#include <stdio.h>
#include <string.h>
#include "m10p.c"

static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { printf("FAIL %d: %s\n", __LINE__, #x); return 1; } } while (0)
static uint8_t wire[M10P_PACKET_MAX_BYTES];

static void put16(uint8_t *p, unsigned value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

static void make(unsigned length, unsigned angle, unsigned raw)
{
    unsigned i;
    memset(wire, 0, sizeof wire);
    wire[0] = 0xa5; wire[1] = 0x5a;
    put16(wire + 2, length); put16(wire + 4, angle); put16(wire + 6, 3472);
    for (i = 8; i < length - 12; i += 2) put16(wire + i, raw);
    wire[length - 2] = 0xfa; wire[length - 1] = 0xfb;
}

int main(void)
{
    const unsigned lengths[] = {22, 156, 158, 160, 162, 164, 166, 512};
    unsigned l, split, i, j, length, slots;
    const M10P_Scan *scan;

    /* 每种长度遍历所有拆包位置，包含头部、距离区和尾部被DMA切开的情况。 */
    for (l = 0; l < sizeof lengths / sizeof lengths[0]; ++l) {
        length = lengths[l]; slots = (length - 20) / 2;
        for (split = 0; split <= length; ++split) {
            M10P_Init(); begin_scan(0); make(length, 18000, 0x83e8);
            M10P_Feed(wire, split, 0, 3472);
            M10P_Feed(wire + split, length - split, 0, 3472);
            CHECK(M10P_stats.packets == 1 && pending == 0);
            CHECK(scans[writer].count == slots && M10P_stats.high_reflect == slots);
            CHECK(scans[writer].points[slots - 1].range_mm == 1000);
            CHECK(scans[writer].points[slots - 1].angle_cdeg ==
                  18000 + (1500 * (slots - 1) + slots / 2) / slots);
        }
    }

    /* 变长槽数中FFFF剔除，零距离保留，高反标志不能混入毫米值。 */
    M10P_Init(); begin_scan(0); make(166, 18000, 0x83e8);
    put16(wire + 8, 0xffff); put16(wire + 10, 0);
    M10P_Feed(wire, 166, 0, 3472);
    CHECK(scans[writer].count == 72 && M10P_stats.invalid_slots == 1);
    CHECK(scans[writer].points[0].range_mm == 0);
    CHECK(scans[writer].points[1].angle_cdeg == 18021);
    CHECK(scans[writer].points[1].flags == 1);
    M10P_Init(); begin_scan(0); make(166, 18000, 0xffff);
    M10P_Feed(wire, 166, 0, 3472);
    CHECK(scans[writer].count == 0 && M10P_stats.invalid_slots == 73);

    /* 一条坏长包里含有两个完整短包：应当在同次Feed中恢复两个，而不越界/等待新字节。 */
    {
        uint8_t damaged[512] = {0xa5, 0x5a, 2, 0};
        make(22, 18000, 1000); memcpy(damaged + 100, wire, 22);
        make(22, 19500, 1000); memcpy(damaged + 122, wire, 22);
        M10P_Init(); M10P_Feed(damaged, sizeof damaged, 0, 10000);
        CHECK(M10P_stats.bad_tail > 0 && M10P_stats.packets == 2 && pending == 0);
    }
    /* 非法上下限和奇数长度必须拒绝，并恢复后续完整包。 */
    {
        const unsigned invalid[] = {0, 20, 21, 157, 513, 514, 65535};
        for (i = 0; i < sizeof invalid / sizeof invalid[0]; ++i) {
            M10P_Init(); make(158, 18000, 1000); put16(wire + 2, invalid[i]);
            M10P_Feed(wire, 158, 0, 3472);
            make(166, 19500, 1000); M10P_Feed(wire, 166, 3472, 6944);
            CHECK(M10P_stats.bad_length > 0 && M10P_stats.packets == 1);
        }
    }
    /* 所有插入/删除位置均应重同步；不把半包拼进下一圈。 */
    for (split = 1; split < 166; ++split) {
        uint8_t extra = 0x11;
        for (j = 0; j < 2; ++j) {
            M10P_Init(); make(166, 18000, 1000);
            M10P_Feed(wire, split, 0, 3472);
            if (j) M10P_Feed(&extra, 1, 0, 3472);
            M10P_Feed(wire + split + (j ? 0 : 1), 166 - split - (j ? 0 : 1), 0, 3472);
            make(158, 19500, 1000); M10P_Feed(wire, 158, 3472, 6944);
            CHECK(M10P_stats.packets == 1 && pending == 0);
        }
    }
    /* 软件允许的大包可能装满整圈数组：必须拒绝整圈，不能发布截断数据。 */
    M10P_Init();
    for (i = 0; i < 72; ++i) {
        make(512, (17300 + i * 1500) % 36000, 1000);
        M10P_Feed(wire, 512, i * 3472, (i + 1) * 3472);
        CHECK(!M10P_Acquire());
    }
    CHECK(M10P_stats.scan_overflow > 0 && M10P_stats.rejected > 0);
    for (i = 72; i < 144; ++i) {
        make(166, (17300 + i * 1500) % 36000, 1000);
        M10P_Feed(wire, 166, i * 3472, (i + 1) * 3472);
    }
    scan = M10P_Acquire();
    CHECK(scan && scan->count == 1752 && !scan->overflow);
    M10P_Release(scan);
    printf("PASS %u variable-length checks\n", checks);
    return 0;
}
