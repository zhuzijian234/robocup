#include "m10p_vehicle.h"
#include "DMA.h"
#include "ble_diag.h"
#include "timer.h"
#include <float.h>
static uint16_t bins[M10P_BINS];
static uint8_t rx[LIDAR_RX_BLOCK];
static uint32_t seen_epoch, seen_discontinuities;
uint32_t M10P_control_seq, M10P_control_front_us, M10P_control_epoch;
uint16_t M10P_front_bins, M10P_left_bins, M10P_right_bins;
float M10P_clearance_mm, M10P_speed_scale;
uint8_t M10P_perception_ok;

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
    if (seen_discontinuities != M10P_stats.discontinuities) {
        seen_discontinuities = M10P_stats.discontinuities;
        Radar_Invalidate();
    }
}
uint16_t M10P_Build(const M10P_Scan *scan, _LEIDA_DATA *out, uint16_t capacity)
{
    uint16_t i, n = 0;
    M10P_front_bins = M10P_left_bins = M10P_right_bins = 0;
    M10P_clearance_mm = (float)M10P_MAX_MM;
    M10P_perception_ok = 0; M10P_speed_scale = 0;
    M10P_control_seq = scan->seq;
    M10P_control_front_us = scan->front_us;
    M10P_control_epoch = scan->epoch;
    M10P_Index(scan, bins);
    for (i = 0; i < M10P_BINS; ++i) if (bins[i] != 0xffffu) {
        const M10P_Point *p = &scan->points[bins[i]];
        float a = M10P_AlgorithmAngle(p->angle_cdeg) / 100.0f;
        if (n >= capacity) return 0;
        out[n].angle = a; out[n++].distance = p->range_mm;
        if (i >= 140 && i <= 220) M10P_front_bins++;
        if (i >= 220 && i <= 340) M10P_left_bins++;
        if (i >= 20 && i <= 140) M10P_right_bins++;
    }
    /* Independent near-obstacle path: use ALL raw returns, not wall filtering.
     * Include close (<100 mm) nonzero returns conservatively as obstacles. */
    for (i = 0; i < scan->count; ++i) {
        const M10P_Point *p = &scan->points[i];
        float angle, x, y;
        if (!p->range_mm || p->range_mm > M10P_MAX_MM) continue;
        angle = M10P_AlgorithmAngle(p->angle_cdeg) * (PI / 18000.0f);
        x = p->range_mm * arm_cos_f32(angle);
        y = p->range_mm * arm_sin_f32(angle);
        if (y > 0 && fabsf(x) < M10P_CORRIDOR_HALF_MM && y < M10P_clearance_mm)
            M10P_clearance_mm = y;
    }
    M10P_perception_ok = scan->front_seen && M10P_front_bins >= 40 &&
        M10P_left_bins >= 16 && M10P_right_bins >= 16 &&
        scan->epoch == LidarRx_epoch &&
        (uint32_t)(Diag_TimeUs() - scan->front_us) <= M10P_MAX_AGE_US &&
        M10P_clearance_mm > M10P_STOP_Y_MM;
    if (M10P_perception_ok) {
        M10P_speed_scale = (M10P_clearance_mm - M10P_STOP_Y_MM) / (M10P_SLOW_Y_MM - M10P_STOP_Y_MM);
        if (M10P_speed_scale > 1) M10P_speed_scale = 1;
        if (M10P_speed_scale < 0.25f) M10P_speed_scale = 0.25f;
    }
    return n;
}
