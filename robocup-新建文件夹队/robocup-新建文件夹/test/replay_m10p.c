/* 实机bin回放驱动：保持生产解析器不变，只模拟字节到达时间。 */
#include <stdio.h>
#include <stdlib.h>
#include "m10p.c"

int main(int argc, char **argv)
{
    FILE *input, *output;
    uint8_t block[512];
    size_t size, chunk;
    uint32_t total = 0, previous_us = 0, scan_count = 0, point_count = 0;
    double byte_rate;
    if (argc != 5) return 2;
    chunk = (size_t)atoi(argv[3]); byte_rate = atof(argv[4]);
    if (!chunk || chunk > sizeof block || byte_rate <= 0) return 2;
    input = fopen(argv[1], "rb"); output = fopen(argv[2], "wb");
    if (!input || !output) return 3;
    M10P_Init();
    while ((size = fread(block, 1, chunk, input)) != 0) {
        const M10P_Scan *scan;
        uint32_t end_us;
        unsigned i;
        total += (uint32_t)size;
        end_us = (uint32_t)(total * 1000000.0 / byte_rate);
        M10P_Feed(block, size, previous_us, end_us);
        previous_us = end_us;
        scan = M10P_Acquire();
        if (!scan) continue;
        if (scan->overflow || scan->unstable || scan->coverage_cdeg != 36000 ||
            scan->period_us < M10P_MIN_PERIOD_US || scan->period_us > M10P_MAX_PERIOD_US)
            return 4;
        for (i = 0; i < scan->count; ++i) {
            /* 明确输出小端三字段，避免结构体填充/时间量化影响点云核对。 */
            const M10P_Point *p = &scan->points[i];
            uint8_t point[5] = {(uint8_t)p->angle_cdeg, (uint8_t)(p->angle_cdeg >> 8),
                                (uint8_t)p->range_mm, (uint8_t)(p->range_mm >> 8), p->flags};
            if (fwrite(point, 1, sizeof point, output) != sizeof point) return 5;
        }
        scan_count++; point_count += scan->count;
        M10P_Release(scan);
    }
    if (ferror(input)) return 6;
    fclose(input);
    if (fclose(output)) return 6;
    printf("{\"packets\":%lu,\"scans\":%lu,\"points\":%lu,\"bad\":%lu,"
           "\"discontinuities\":%lu,\"overflow\":%lu,\"rejected\":%lu}\n",
           (unsigned long)M10P_stats.packets, (unsigned long)scan_count, (unsigned long)point_count,
           (unsigned long)(M10P_stats.bad_length + M10P_stats.bad_tail + M10P_stats.bad_angle + M10P_stats.bad_speed),
           (unsigned long)M10P_stats.discontinuities, (unsigned long)M10P_stats.scan_overflow,
           (unsigned long)M10P_stats.rejected);
    return 0;
}
