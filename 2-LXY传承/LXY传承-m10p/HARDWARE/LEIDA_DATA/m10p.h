#ifndef M10P_H
#define M10P_H
#include <stdint.h>
#include <stddef.h>

/* M10P 20K serial V2.0 only. Native zero=front, positive=clockwise. */
#define M10P_BAUD 512000u
#define M10P_PACKET_BYTES 160u
#define M10P_SCAN_CAPACITY 2048u
#define M10P_BINS 720u
#define M10P_MAX_AGE_US 150000u
#define M10P_FRONT_MAX_MISSING_BINS 10u /* 5 degrees at half-degree resolution */
#define M10P_MIN_PERIOD_US 60000u
#define M10P_MAX_PERIOD_US 120000u
#define M10P_BETA_CDEG 0
#define M10P_MIN_MM 100u
#define M10P_MAX_MM 10000u
/* Conservative commissioning envelope, measured from the lidar origin.
 * Measure actual vehicle footprint before running on the floor. */
#define M10P_CORRIDOR_HALF_MM 180.0f
#define M10P_STOP_Y_MM 350.0f
#define M10P_SLOW_Y_MM 1000.0f

typedef struct {
    uint16_t angle_cdeg, range_mm, offset_100us;
    uint8_t quality, flags; /* quality=0: unavailable; flags bit0 high reflection */
} M10P_Point;
typedef struct {
    M10P_Point points[M10P_SCAN_CAPACITY];
    uint32_t seq, start_us, end_us, period_us, front_us, epoch;
    uint16_t count, coverage_cdeg, dps, invalid_slots;
    uint8_t front_seen, overflow, unstable;
} M10P_Scan;
typedef struct {
    uint32_t bytes, packets, bad_length, bad_tail, bad_angle, bad_speed;
    uint32_t invalid_slots, high_reflect, discontinuities, scans, rejected;
    uint32_t scan_overflow, ready_drop, epoch;
} M10P_Stats;
extern M10P_Stats M10P_stats;
void M10P_Init(void);
void M10P_Lost(uint32_t epoch);
/* Bounds on byte reception, NOT parsing time. start includes DMA IRQ uncertainty. */
void M10P_Feed(const uint8_t *data, size_t count, uint32_t start_us, uint32_t end_us);
const M10P_Scan *M10P_Acquire(void);
void M10P_Release(const M10P_Scan *scan);
uint16_t M10P_AlgorithmAngle(uint16_t native_cdeg);
/* Bin indices refer to the acquired scan. Empty=UINT16_MAX. */
uint16_t M10P_Index(const M10P_Scan *scan, uint16_t bins[M10P_BINS]);
#endif
