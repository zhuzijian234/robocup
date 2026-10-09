#include "path_track.h"
#include <math.h>
#include <string.h>

#define STRIPS 12
#define STRIP_MM 100.0f
#define MIN_SPAN_MM 250.0f
#define MAX_GAP_MM 310.0f
#define MAX_RMS_MM 60.0f
#define EXIT_TIME_US 60000u

/* 实车：扫描平面高150 mm，锥桶底圆高23 mm、半径75 mm，顶圆高300 mm、半径25 mm。
 * 截面半径约52 mm；200 mm方底座任意朝向的外接圆半径142 mm。
 * 170 mm包络包含底座及中心估计余量，150 mm车体包络包含130 mm半宽及20 mm侧隙。 */
#define CONE_SCAN_RADIUS_MM 52.0f
#define CONE_BASE_ENVELOPE_MM 170.0f
#define VEHICLE_HALF_CLEAR_MM 150.0f
#define CONE_JOIN_MM 75.0f

static float bounded(float value, float low, float high)
{
    return value < low ? low : value > high ? high : value;
}

/* 每个前向分区只贡献一个观测，避免几十个相邻雷达桶把一个锥桶误当整条边界。
 * 从近到远关联边界；预测位置可跨过 x=0，不能用 x 的正负永久划分左右墙。
 * 左右复用 12 个点的工作区；扫描输入有固定上限，无排序、堆分配或整圈复制。 */
static uint16_t boundary(const PathPoint *points, uint16_t count, int side, float width, float center,
                         PathPoint *out)
{
    uint16_t i, n = 0;
    int strip;
    float slope = 0.0f;
    for (strip = 1; strip <= STRIPS; ++strip)
    {
        float lo = strip * STRIP_MM, hi = lo + STRIP_MM;
        float expected = center + side * width * 0.5f;
        float best = 1e9f, chosen_x = 0, chosen_y = 0;
        float sx = 0, sy = 0, samples = 0;
        if (n)
            expected = out[n - 1].x + slope * (lo + 50.0f - out[n - 1].y);
        for (i = 0; i < count; ++i)
        {
            float x = points[i].x, y = points[i].y, difference;
            if (!(y >= lo && y < hi && fabsf(x) < 2200.0f))
                continue;
            /* 首个分区用车体两侧建立身份；后续根据空间连续性关联。 */
            if (!n && (side * (x - center) < 40.0f || lo > 400.0f))
                continue;
            difference = fabsf(x - expected);
            if (difference < best)
            {
                best = difference;
                chosen_x = x;
                chosen_y = y;
            }
        }
        if (best > (n ? 220.0f : 300.0f))
            continue;
        if (n && chosen_y - out[n - 1].y > MAX_GAP_MM)
            break;
        /* 只聚合所选局部簇，不能把同一高度的对墙/横墙一起求均值。 */
        for (i = 0; i < count; ++i)
        {
            if (points[i].y >= lo && points[i].y < hi && fabsf(points[i].x - chosen_x) < 65.0f)
            {
                sx += points[i].x;
                sy += points[i].y;
                samples += 1.0f;
            }
        }
        if (!samples)
            continue;
        out[n].x = sx / samples;
        out[n].y = sy / samples;
        if (n)
            slope = bounded((out[n].x - out[n - 1].x) / (out[n].y - out[n - 1].y), -1.5f, 1.5f);
        ++n;
    }
    return n;
}

/* 固定空间窗口的稳健拟合：第二遍剔除离群分区，再检查跨度、间隙和残差。
 * 不使用数组末尾百分比，也不把 1.9 m 处回波冒充 700 mm 参考。 */
static PathFit fit(const PathPoint *points, uint16_t count, float low, float high)
{
    PathFit out;
    uint16_t i, n;
    int pass;
    float sx, sy, yy, xy, residual, last;
    memset(&out, 0, sizeof out);
    for (pass = 0; pass < 2; ++pass)
    {
        sx = sy = yy = xy = 0;
        n = 0;
        last = 0;
        out.min_y = high;
        out.max_y = low;
        out.gap = 0;
        for (i = 0; i < count; ++i)
        {
            float x = points[i].x, y = points[i].y;
            if (y < low || y > high)
                continue;
            if (pass && fabsf(x - (out.a * y + out.b)) > 100.0f)
                continue;
            sx += x;
            sy += y;
            yy += y * y;
            xy += x * y;
            if (n && y - last > out.gap)
                out.gap = y - last;
            last = y;
            ++n;
            if (y < out.min_y)
                out.min_y = y;
            if (y > out.max_y)
                out.max_y = y;
        }
        out.count = n;
        if (!n)
            out.min_y = out.max_y = 0;
        if (n < 4 || out.max_y - out.min_y < MIN_SPAN_MM || out.gap > MAX_GAP_MM)
            return out;
        yy -= sy * sy / n;
        xy -= sx * sy / n;
        if (yy <= 1.0f)
            return out;
        out.a = xy / yy;
        out.b = (sx - out.a * sy) / n;
    }
    residual = 0;
    n = 0;
    for (i = 0; i < count; ++i)
    {
        float e = points[i].x - (out.a * points[i].y + out.b);
        if (points[i].y < low || points[i].y > high || fabsf(e) > 100.0f)
            continue;
        residual += e * e;
        ++n;
    }
    out.rms = n ? sqrtf(residual / n) : 1e9f;
    out.valid = n >= 4 && out.rms <= MAX_RMS_MM && fabsf(out.a) <= 1.8f;
    return out;
}

static float at(const PathFit *line, float y)
{
    return line->a * y + line->b;
}

/* 双侧在相同 y 比较；单侧按法线半宽偏移，不把斜墙简单横移半宽。 */
static float center_at(const PathFit *left, const PathFit *right, uint8_t source, float width, float y)
{
    if (source == PATH_DUAL)
        return 0.5f * (at(left, y) + at(right, y));
    if (source == PATH_LEFT)
        return at(left, y) + 0.5f * width * sqrtf(1 + left->a * left->a);
    return at(right, y) - 0.5f * width * sqrtf(1 + right->a * right->a);
}

static float direction(const PathFit *left, const PathFit *right, uint8_t source)
{
    return source == PATH_DUAL ? 0.5f * (left->a + right->a) : source == PATH_LEFT ? left->a : right->a;
}

/* 输入已按雷达角度排列。仅接纳独立、具有圆弧形状的小簇；平墙短片、单点噪声和
 * 连续围挡不作为锥桶。用已知截面圆拟合中心，不能把可见圆弧表面当作底座边缘。 */
static uint8_t cone_center(const PathPoint *points, uint16_t begin, uint16_t end, PathPoint *center)
{
    uint16_t i;
    float dx, dy, chord2, mx, my, h, nx, ny, bulge = 0, residual = 0;
    if (end - begin < 4)
        return 0;
    dx = points[end - 1].x - points[begin].x;
    dy = points[end - 1].y - points[begin].y;
    chord2 = dx * dx + dy * dy;
    if (!(chord2 >= 35 * 35 && chord2 <= 115 * 115))
        return 0;
    mx = 0.5f * (points[begin].x + points[end - 1].x);
    my = 0.5f * (points[begin].y + points[end - 1].y);
    h = sqrtf(fmaxf(0, CONE_SCAN_RADIUS_MM * CONE_SCAN_RADIUS_MM - 0.25f * chord2));
    nx = -dy / sqrtf(chord2);
    ny = dx / sqrtf(chord2);
    /* 雷达看到圆的近侧；两个候选圆心中选择远离雷达的一侧。 */
    if (mx * nx + my * ny < 0)
        h = -h;
    center->x = mx + nx * h;
    center->y = my + ny * h;
    for (i = begin; i < end; ++i)
    {
        float x = points[i].x - center->x, y = points[i].y - center->y;
        float e = sqrtf(x * x + y * y) - CONE_SCAN_RADIUS_MM;
        float cross = fabsf(dx * (points[i].y - points[begin].y) -
                            dy * (points[i].x - points[begin].x));
        if (cross > bulge)
            bulge = cross;
        residual += e * e;
    }
    return bulge * bulge >= 4 * 4 * chord2 && residual <= (end - begin) * 12 * 12 &&
           center->y >= 100 && center->y <= 1100;
}

/* 避让处必须有本帧边界支持；分区均值的边缘仅允许50mm容差。 */
static uint8_t supports_y(const PathFit *segment, uint8_t source, float y)
{
    uint8_t side = source == PATH_RIGHT ? 1 : 0;
    float lo = segment[side].min_y, hi = segment[side].max_y;
    if (source == PATH_DUAL)
    {
        lo = fmaxf(lo, segment[1].min_y);
        hi = fminf(hi, segment[1].max_y);
    }
    return y >= lo - 50 && y <= hi + 50;
}

/* 小范围局部避让：保持既有道路路径，只向可通行一侧平移参考。
 * 按道路法向计算底座/车体占用；同时受对侧墙余量限制，不因躲锥桶而直接指向墙外。
 * 多个锥桶优先处理最近的冲突，远方另一侧锥桶不能抵消当前避让。无堆分配/整圈复制。 */
static void avoid_cones(PathGeometry *state, const PathPoint *points, uint16_t count, float center_x,
                        PathObservation *out)
{
    uint16_t begin = 0, end;
    float requested = 0, nearest = 1e9f;
    float normal = sqrtf(1 + out->near_a * out->near_a);
    /* 安装标定可能让车体相对道路中线偏置；两侧预算都计入它，避免吃掉侧隙。 */
    float body_clearance = VEHICLE_HALF_CLEAR_MM + fabsf(center_x);
    float allowance = fmaxf(0, out->width * 0.5f - body_clearance);
    /* S弯近远段角度不同：横移量不能超过任一段的对墙余量。 */
    if (out->far_valid)
        normal = fminf(normal, sqrtf(1 + out->far_a * out->far_a));
    for (end = 1; end <= count; ++end)
    {
        PathPoint cone;
        float dx, dy, lateral, required, shift, path_x, a, scale;
        if (end < count)
        {
            dx = points[end].x - points[end - 1].x;
            dy = points[end].y - points[end - 1].y;
            if (dx * dx + dy * dy <= CONE_JOIN_MM * CONE_JOIN_MM)
                continue;
        }
        if (cone_center(points, begin, end, &cone))
        {
            uint8_t use_far = out->far_valid && cone.y >= 550 &&
                              supports_y(out->far_fit, out->source, cone.y);
            /* 近处用近段，远处有可信远段才使用远段；禁止跨很远外推避让。 */
            a = use_far ? out->far_a : out->near_a;
            path_x = use_far ? out->far_x + a * (cone.y - out->ref_y) : out->near_x + a * (cone.y - 400);
            scale = sqrtf(1 + a * a);
            lateral = (cone.x - path_x) / scale;
            required = CONE_BASE_ENVELOPE_MM + body_clearance - fabsf(lateral);
            if (required > 0 && cone.y < nearest &&
                (use_far || supports_y(out->near_fit, out->source, cone.y)) &&
                fabsf(lateral) <= out->width * 0.5f + 60)
            {
                /* 中线附近沿既有避让方向通过，避免圆心小幅抖动导致左右反复切换。 */
                shift = fabsf(lateral) < 30 && state->avoid_offset != 0 ?
                            (state->avoid_offset > 0 ? required : -required) :
                            (lateral <= 0 ? required : -required);
                /* 1.1m开始平滑介入，800mm内给出完整避让偏移。 */
                shift *= bounded((1100 - cone.y) / 300, 0, 1);
                requested = bounded(shift * scale, -allowance * normal, allowance * normal);
                nearest = cone.y;
            }
        }
        begin = end;
    }
    if (nearest < 1e9f)
    {
        state->avoid_offset = requested;
        state->avoid_hold = 3;
        out->avoid_y = nearest;
    }
    else if (state->avoid_hold)
        --state->avoid_hold; /* 过桶/短暂遮挡不立刻回中，连续三帧后再平滑释放。 */
    else if (state->avoid_offset > 0)
        state->avoid_offset = fmaxf(0, state->avoid_offset - 20);
    else
        state->avoid_offset = fminf(0, state->avoid_offset + 20);
    state->avoid_offset = bounded(state->avoid_offset, -allowance * normal, allowance * normal);
    out->avoid_offset = state->avoid_offset;
    out->target_x += out->avoid_offset;
}

void Path_Build(PathGeometry *state, const PathPoint *points, uint16_t count, float configured_width,
                float preview_y, float center_x, PathObservation *out)
{
    PathPoint samples[STRIPS];
    PathFit *near, *far;
    uint16_t n;
    uint8_t side, source;
    float width, near_y = 400.0f, far_y, slope, lo, hi;
    memset(out, 0, sizeof *out);
    out->source = PATH_NONE;
    if (!(configured_width >= 400 && configured_width <= 900) || !(preview_y >= 600 && preview_y <= 1100) ||
        !(fabsf(center_x) <= 300))
        return;
    if (state->configured_width != configured_width)
    {
        state->configured_width = configured_width;
        state->width = configured_width;
        state->measured = 0;
        state->avoid_offset = 0;
        state->avoid_hold = 0;
    }
    out->width = state->width;
    if (!points || count > 400)
        return;
    for (side = 0; side < 2; ++side)
    {
        n = boundary(points, count, side ? 1 : -1, state->width, center_x, samples);
        out->near_fit[side] = fit(samples, n, 100, 750);
        out->far_fit[side] = fit(samples, n, 550, 1300);
    }
    near = out->near_fit;
    far = out->far_fit;
    if (!near[0].valid && !near[1].valid)
        return;
    source = near[0].valid ? PATH_LEFT : PATH_RIGHT;
    if (near[0].valid && near[1].valid)
    {
        slope = 0.5f * (near[0].a + near[1].a);
        width = (at(&near[1], near_y) - at(&near[0], near_y)) / sqrtf(1 + slope * slope);
        out->width_candidate = width;
        /* 墙面交叉/分叉或宽度明显不一致时，不把两个不相干的面合为中线。 */
        if (width >= 400 && width <= 900 && fabsf(near[0].a - near[1].a) <= 0.35f)
        {
            source = PATH_DUAL;
        }
        else
        {
            /* 两面互相矛盾且质量相近时不猜方向；显著较好的单侧才接管。 */
            if (near[0].rms + 20 < near[1].rms)
                source = PATH_LEFT;
            else if (near[1].rms + 20 < near[0].rms)
                source = PATH_RIGHT;
            else
                return;
        }
    }
    out->width = state->width;
    side = source == PATH_RIGHT ? 1 : 0;
    lo = near[side].min_y;
    hi = near[side].max_y;
    if (source == PATH_DUAL)
    {
        if (near[1].min_y > lo)
            lo = near[1].min_y;
        if (near[1].max_y < hi)
            hi = near[1].max_y;
    }
    if (near_y < lo || near_y > hi)
        return;
    if (source == PATH_DUAL)
    {
        /* 参考高度确实有共同支持以后，才允许更新历史宽度。 */
        width = out->width_candidate;
        state->width = state->measured ? 0.75f * state->width + 0.25f * width : width;
        state->measured = 1;
        out->width_measured = 1;
        out->width = state->width;
    }
    /* 双侧近段可信、但一侧远段被遮挡时，用另一侧连续边界保留预瞄。
     * 必须在两段实际接缝处位置连续，不能把横墙/下一个不相干物体接成远段。 */
    if (source == PATH_DUAL && far[0].valid != far[1].valid)
    {
        uint8_t available = far[1].valid ? 1 : 0;
        float join_y = far[available].min_y;
        if (near[available].max_y >= join_y - 50 &&
            far[available].min_y <= preview_y + 100 && far[available].max_y >= preview_y - 100 &&
            fabsf(at(&near[available], join_y) - at(&far[available], join_y)) <= 80 &&
            fabsf(near[available].a - far[available].a) <= 0.8f)
        {
            source = available ? PATH_RIGHT : PATH_LEFT;
            side = available;
        }
    }
    out->source = source;
    out->valid = 1;
    out->near_x = center_at(&near[0], &near[1], source, state->width, near_y);
    out->near_a = direction(&near[0], &near[1], source);
    out->far_valid = source == PATH_DUAL ? far[0].valid && far[1].valid : far[side].valid;
    if (out->far_valid)
    {
        lo = far[side].min_y;
        hi = far[side].max_y;
        if (source == PATH_DUAL)
        {
            if (far[1].min_y > lo)
                lo = far[1].min_y;
            if (far[1].max_y < hi)
                hi = far[1].max_y;
            width = at(&far[1], preview_y) - at(&far[0], preview_y);
            slope = direction(&far[0], &far[1], source);
            width /= sqrtf(1 + slope * slope);
            if (width < 400 || width > 900 || fabsf(far[0].a - far[1].a) > 0.5f)
                out->far_valid = 0;
        }
        far_y = bounded(preview_y, lo, hi);
        if (fabsf(far_y - preview_y) > 100.0f || hi < lo)
            out->far_valid = 0;
        if (out->far_valid)
        {
            out->far_x = center_at(&far[0], &far[1], source, state->width, far_y);
            out->far_a = direction(&far[0], &far[1], source);
            out->ref_y = far_y;
            out->target_x = out->far_x;
        }
    }
    if (!out->far_valid)
    {
        /* 仅近段有效仍可跟线；明确缩短预瞄，禁止无支持地远距离外推。 */
        out->ref_y = bounded(preview_y, near[side].min_y, near[side].max_y);
        if (source == PATH_DUAL && out->ref_y > near[1].max_y)
            out->ref_y = near[1].max_y;
        out->target_x = center_at(&near[0], &near[1], source, state->width, out->ref_y);
    }
    out->straight = out->far_valid && fabsf(out->near_a) < 0.10f && fabsf(out->far_a) < 0.10f &&
                    fabsf(out->near_a - out->far_a) < 0.10f && fabsf(out->far_x - out->near_x) < 60;
    avoid_cones(state, points, count, center_x, out);
}

static int8_t sign(float value)
{
    return value > 0 ? 1 : value < 0 ? -1 : 0;
}

void Path_Control(PathController *state, const PathObservation *path, const PathGains *g, uint8_t scan_valid,
                  uint32_t now, uint16_t actual_pwm, PathCommand *out)
{
    float limit, dt, original_d;
    int8_t wanted;
    uint8_t curved, wait = 0;
    memset(out, 0, sizeof *out);
    out->pwm = out->candidate_pwm = actual_pwm;
    out->source = path->source;
    out->previous_source = state->previous_source;
    out->bend = state->bend;
    /* 长时间没有扫描时，旧的一帧证据不能与恢复后的第一帧凑成连续两帧。 */
    if ((uint32_t)(now - state->observed_us) > 250000u)
        state->evidence_frames = 0;
    state->observed_us = now;
    if (!scan_valid || !path->valid)
    {
        out->reason = scan_valid ? PATH_NO_GEOMETRY : PATH_BAD_SCAN;
        state->history = 0;
        state->evidence_frames = 0;
        return;
    }
    curved = fabsf(path->near_a) > 0.14f ||
             (path->far_valid && (fabsf(path->far_a) > 0.14f || fabsf(path->far_x - path->near_x) > 70));
    /* 弯向状态依据道路本身；避让偏移不能把锥桶误解释成新的S弯。 */
    out->error = g->center_x - (path->target_x - path->avoid_offset);
    wanted = sign(out->error);
    /* 来源只是左/右墙身份，绝不规定输出方向。可靠反向路径当帧接管。 */
    if (state->bend && path->straight)
    {
        if (!state->evidence_frames || state->pending_direction != 0)
        {
            state->evidence_us = now;
            state->evidence_frames = 0;
        }
        state->pending_direction = 0;
        if (state->evidence_frames < 255)
            ++state->evidence_frames;
        if (state->evidence_frames >= 2 && (uint32_t)(now - state->evidence_us) >= EXIT_TIME_US)
        {
            state->bend = 0;
            out->reason = PATH_EXIT;
        }
        else
        {
            wait = 1;
            out->reason = PATH_WAIT_EXIT;
        }
    }
    else if (state->bend && wanted != state->bend && fabsf(out->error) > 30)
    {
        if (path->far_valid && fabsf(path->far_a) > 0.14f && sign(-path->far_a) == wanted &&
            (sign(-path->near_a) == wanted || fabsf(path->near_a) < 0.10f))
        {
            state->bend = wanted;
            state->evidence_frames = 0;
            out->reason = PATH_REVERSE;
        }
        else
        {
            /* 小S弯近段方向超过0.10也可积累证据；仍需两帧，单个抖点不能反打。 */
            if (!state->evidence_frames || state->pending_direction != wanted)
            {
                state->evidence_frames = 0;
                state->evidence_us = now;
            }
            state->pending_direction = wanted;
            if (fabsf(path->near_a) > 0.10f && sign(-path->near_a) == wanted)
            {
                if (state->evidence_frames < 255)
                    ++state->evidence_frames;
            }
            else
                state->evidence_frames = 0;
            if (state->evidence_frames >= 2 && (uint32_t)(now - state->evidence_us) >= EXIT_TIME_US)
            {
                state->bend = wanted;
                state->evidence_frames = 0;
                out->reason = PATH_REVERSE;
            }
            else
            {
                wait = 1;
                out->reason = PATH_WAIT_REVERSE;
            }
        }
    }
    else
    {
        state->evidence_frames = 0;
        if (curved && fabsf(out->error) > 30)
            state->bend = wanted;
        /* 小误差但远段仍弯曲不能作为回正证据；有同向路径则允许持续调整舵角。 */
        if (state->bend && fabsf(out->error) <= 30)
        {
            wait = 1;
            out->reason = PATH_WAIT_EXIT;
        }
    }
    /* 有空间验证的避让不受旧弯向锁定；只覆盖本帧参考，不改道路弯向和速度。 */
    if (path->avoid_offset != 0 && wait)
    {
        wait = 0;
        out->reason = PATH_APPLY;
    }
    out->error -= path->avoid_offset;
    /* 状态确定后限幅：确认出弯的这一帧就恢复直道限幅。 */
    limit = curved || state->bend ? 500.0f : g->straight_limit;
    out->error = bounded(out->error, -limit, limit);
    out->kp = path->source != PATH_DUAL ? g->side_kp : curved || state->bend ? g->turn_kp : g->kp;
    out->kd = path->source != PATH_DUAL ? g->side_kd : curved || state->bend ? g->turn_kd : g->kd;
    /* 只增强大弯：250mm内保持原比例，250~500mm连续增加至1.35倍。
     * 默认单边500mm稳态输出约1178/1711，利用余量而不放大直道小误差和D噪声。 */
    if (curved || state->bend)
        out->kp *= 1 + 0.35f * bounded((fabsf(out->error) - 250) / 250, 0, 1);
    dt = (float)(uint32_t)(now - state->previous_us);
    out->d_reset = !state->history || path->source != state->previous_source ||
                   path->far_valid != state->previous_far_valid ||
                   fabsf(path->ref_y - state->previous_ref_y) > 100 || dt < 20000 || dt > 250000 ||
                   fabsf(path->avoid_offset - state->previous_avoid_offset) > 30 ||
                   out->reason == PATH_REVERSE;
    out->p = 10 * out->kp * out->error;
    original_d = out->d_reset ? 0 : 10 * out->kd * (out->error - state->previous_error) * 115000.0f / dt;
    out->d = bounded(original_d, -150, 150);
    if (out->p * (out->p + out->d) < 0)
        out->d = -out->p;
    out->unclamped = g->pwm_mid + out->p + out->d;
    out->candidate_pwm = (uint16_t)bounded(out->unclamped, g->pwm_min, g->pwm_max);
    out->computed = 1;
    out->bend = state->bend;
    if (wait)
    {
        state->history = 0;
        return;
    }
    out->applied = 1;
    out->pwm = out->candidate_pwm;
    state->history = 1;
    state->previous_error = out->error;
    state->previous_source = path->source;
    state->previous_us = now;
    state->previous_ref_y = path->ref_y;
    state->previous_far_valid = path->far_valid;
    state->previous_avoid_offset = path->avoid_offset;
}
