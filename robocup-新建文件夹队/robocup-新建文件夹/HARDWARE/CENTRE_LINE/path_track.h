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
/* 位图可以同时报告多个拒绝原因，0 表示该拟合通过。 */
enum
{
    FIT_POINTS = 1, FIT_SPAN = 2, FIT_GAP = 4,
    FIT_DEGENERATE = 8, FIT_RMS = 16, FIT_SLOPE = 32
};
enum
{
    GEOM_OK, GEOM_ARGUMENT, GEOM_NEAR_MISSING, GEOM_CONFLICT, GEOM_SUPPORT
};
enum
{
    AVOID_CLEAR, AVOID_CANDIDATE, AVOID_ACTIVE, AVOID_HOLD, AVOID_RELEASE
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
    uint8_t valid, rejected;
} PathFit;
typedef struct
{
    PathFit near_fit[2], far_fit[2];
    float near_x, far_x, near_a, far_a, ref_y, target_x;
    float width, width_candidate;
    float avoid_offset, avoid_y; /* 避让附加横移(mm)、当前锥桶前向位置；不改变车速 */
    float near_ref_y; /* 近段实际参考位置；必须位于本帧边界支持范围内 */
    float cone_x, cone_y, cone_lateral, avoid_required, avoid_allowed;
    uint16_t cone_candidates;
    uint8_t geometry_reason, avoid_state, avoid_confirm, width_frozen;
    uint8_t valid, far_valid, source, width_measured, straight;
} PathObservation;
typedef struct
{
    float width; /* 仅可靠双侧观测可更新；单侧使用此历史宽度 */
    float configured_width;
    float avoid_offset; /* 相对本帧道路中线的避让偏移，过桶后逐步归零 */
    uint8_t avoid_hold;
    uint8_t measured;
    float candidate_x, candidate_y;
    uint8_t candidate_frames;
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
    float previous_avoid_offset;
    int8_t bend, pending_direction;
    uint8_t history, previous_source, previous_far_valid, evidence_frames;
} PathController;
typedef struct
{
    float error, p, d, kp, kd, unclamped;
    float road_error; /* 未叠加避让、未限幅的道路误差，诊断不再反推 */
    uint8_t gate_reason, avoid_override; /* 保留被避让覆盖前的道路裁决 */
    uint16_t candidate_pwm, pwm;
    uint8_t computed, applied, reason, source, previous_source, d_reset;
    int8_t bend;
} PathCommand;

/* points沿雷达角度递增排列（LEIDA_FrontPoints的输出）；锥桶聚类依赖相邻射线顺序。 */
void Path_Build(PathGeometry *state, const PathPoint *points, uint16_t count, float configured_width,
                float preview_y, float center_x, PathObservation *out);
void Path_Control(PathController *state, const PathObservation *path, const PathGains *gains,
                  uint8_t scan_valid, uint32_t now_us, uint16_t actual_pwm, PathCommand *out);
#endif
