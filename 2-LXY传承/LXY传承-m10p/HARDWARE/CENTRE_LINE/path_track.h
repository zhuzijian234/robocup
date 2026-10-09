#ifndef PATH_TRACK_H
#define PATH_TRACK_H

#include <stdint.h>

/* 纯几何与控制模块，不访问外设；毫米坐标：x 向右、y 向前。 */
typedef struct
{
    float x, y;
} PathPoint;
enum
{
    PATH_DUAL,
    PATH_LEFT,
    PATH_RIGHT,
    PATH_NONE
};
enum
{
    PATH_APPLY,
    PATH_NO_GEOMETRY,
    PATH_WAIT_EXIT,
    PATH_WAIT_REVERSE,
    PATH_EXIT,
    PATH_REVERSE,
    PATH_BAD_SCAN
};
typedef struct
{
    float a, b; /* x = a*y + b，前向直线不会除以零 */
    float min_y, max_y, rms, gap;
    uint16_t count; /* 独立的 100 mm 前向分区数量，不是密集点数量 */
    uint8_t valid;
} PathFit;
typedef struct
{
    PathFit near_fit[2], far_fit[2];
    float near_x, far_x, near_a, far_a, ref_y, target_x;
    float width, width_candidate;
    uint8_t valid, far_valid, source, width_measured, straight;
} PathObservation;
typedef struct
{
    float width; /* 仅可靠双侧观测可更新；单侧使用此历史宽度 */
    float configured_width;
    uint8_t measured;
} PathGeometry;
typedef struct
{
    float kp, kd, turn_kp, turn_kd, side_kp, side_kd;
    float center_x, straight_limit;
    uint16_t pwm_min, pwm_mid, pwm_max;
} PathGains;
typedef struct
{
    uint32_t previous_us, evidence_us, observed_us;
    float previous_error, previous_ref_y;
    int8_t bend, pending_direction;
    uint8_t history, previous_source, previous_far_valid, evidence_frames;
} PathController;
typedef struct
{
    float error, p, d, kp, kd, unclamped;
    uint16_t candidate_pwm, pwm;
    uint8_t computed, applied, reason, source, previous_source, d_reset;
    int8_t bend;
} PathCommand;

void Path_Build(PathGeometry *state, const PathPoint *points, uint16_t count, float configured_width,
                float preview_y, float center_x, PathObservation *out);
void Path_Control(PathController *state, const PathObservation *path, const PathGains *gains,
                  uint8_t scan_valid, uint32_t now_us, uint16_t actual_pwm, PathCommand *out);
#endif
