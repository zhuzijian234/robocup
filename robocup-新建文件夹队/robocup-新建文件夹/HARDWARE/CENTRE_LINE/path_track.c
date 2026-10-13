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
 * 170 mm包络包含底座及中心估计余量。车身宽240 mm，但此前实测轮外沿半宽约130 mm，
 * 仍按较大的轮外沿加20 mm侧隙计算，不能因更新车身宽度而缩小实际轮胎包络。
 * 实测轴距305 mm、雷达到后轴205 mm，因此前轴在雷达前100 mm；轮胎前后外廓385 mm。
 * 这里仍计算局部横向余量，未标定实际舵角前不冒充完整转弯扫掠模型。 */
#define CONE_SCAN_RADIUS_MM 52.0f
#define CONE_BASE_ENVELOPE_MM 170.0f
#define VEHICLE_HALF_CLEAR_MM 150.0f
#define CONE_JOIN_MM 75.0f
#define ACUTE_OUTER_MM 70.0f /* 入口局部位置目标上限，不是远端目标减70mm。
                              * 2026-10-13 由 40 提到 70：goal 仍受下面的 available
                              * （=宽度/2 - 车身半宽投影 - 40mm 贴墙余量）压制，
                              * 所以 40mm 的最小余量由算式保证，不靠这个常量保证。
                              * 宽500/yaw≈0 时 available=80，取70 留10mm 余量，
                              * 此时车心离墙 250-70-130=50mm。窄道会按 available 自动降。 */
#define ACUTE_TAKEOVER_MM 700.0f /* 可见偏离位置到此范围便交还主弯，未标定真实转弯半径。 */
#define ACUTE_FULL_PREP_MM 950.0f

static void acute_clear(PathGeometry *state)
{
    state->acute_frames = state->acute_exit_frames = 0;
    state->acute_direction = 0;
    state->acute_phase = ACUTE_OFF;
}

void Path_ClearHistory(PathGeometry *state)
{
    state->history = state->history_misses = 0;
    state->history_model = 0;
    acute_clear(state);
    state->baseline_offset = 0;
    state->baseline_frames = 0;
    state->baseline_direction = 0;
}

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

/* 两次独立搜索可能接上同一面弯墙。保留较早建立的近场身份，截断另一条
 * 链从重合处开始的尾部；不删除原点云，也不把x的正负当作整条墙的身份。 */
static uint8_t separate_boundaries(PathPoint samples[2][STRIPS], uint16_t n[2])
{
    uint16_t i, j, first[2] = {STRIPS, STRIPS};
    uint8_t shared = 0, keep;
    for (i = 0; i < n[0]; ++i)
        for (j = 0; j < n[1]; ++j)
            if (fabsf(samples[0][i].x - samples[1][j].x) < 12 &&
                fabsf(samples[0][i].y - samples[1][j].y) < 12)
            {
                if (i < first[0]) first[0] = i;
                if (j < first[1]) first[1] = j;
                ++shared;
            }
    if (!shared)
        return 0;
    if (fabsf(samples[0][0].y - samples[1][0].y) > 20)
        keep = samples[0][0].y < samples[1][0].y ? 0 : 1;
    else if (first[0] != first[1])
        keep = first[0] > first[1] ? 0 : 1;
    else if (n[0] != n[1])
        keep = n[0] > n[1] ? 0 : 1;
    else
    {
        /* 同等证据时不猜墙面；保留各自未重合的近场部分。 */
        n[0] = first[0]; n[1] = first[1];
        return shared;
    }
    n[1 - keep] = first[1 - keep];
    return shared;
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
        out.rejected = (n < 4 ? FIT_POINTS : 0) | (out.max_y - out.min_y < MIN_SPAN_MM ? FIT_SPAN : 0) |
                       (out.gap > MAX_GAP_MM ? FIT_GAP : 0);
        if (out.rejected)
            return out;
        yy -= sy * sy / n;
        xy -= sx * sy / n;
        if (yy <= 1.0f)
        {
            out.rejected = FIT_DEGENERATE;
            return out;
        }
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
    out.rejected = (n < 4 ? FIT_POINTS : 0) | (out.rms > MAX_RMS_MM ? FIT_RMS : 0) |
                   (fabsf(out.a) > 1.8f ? FIT_SLOPE : 0);
    out.valid = !out.rejected;
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

/* 急弯中外墙可能已跨车体中线：根据上一帧道路的法向位置恢复墙身份。
 * 只有同向大角度、共同可见区以及明显“一整条车道”的错位才修正；
 * 小角度、S弯和真实向另一侧移动不靠历史硬猜。 */
static void restore_wall_identity(const PathGeometry *state, const PathPoint *points, uint16_t count,
                                  PathObservation *out)
{
    unsigned side, i, sample;
    if (!state->history || state->history_misses || fabsf(state->history_a) < .6f)
        return;
    if (out->near_fit[0].valid && out->near_fit[1].valid)
    {
        float slope = .5f * (out->near_fit[0].a + out->near_fit[1].a);
        float width = (at(&out->near_fit[1], 400) - at(&out->near_fit[0], 400)) /
                      sqrtf(1 + slope * slope);
        if (width >= 400 && width <= 900 &&
            fabsf(out->near_fit[0].a - out->near_fit[1].a) <= .35f)
            return; /* 本帧独立双侧走廊优先，历史不推翻可靠的当前观测。 */
    }
    for (side = 0; side < 2; ++side)
    {
        PathFit *n = &out->near_fit[side];
        float y, predicted, wall, normal, original, alternate;
        if (!n->valid || n->a * state->history_a <= 0 || fabsf(n->a) < .6f ||
            fabsf(atanf(n->a) - atanf(state->history_a)) > .3f)
            continue;
        y = bounded(state->history_y, n->min_y, n->max_y);
        if (fabsf(y - state->history_y) > 80)
            continue;
        predicted = state->history_x + state->history_a * (y - state->history_y);
        wall = at(n, y);
        normal = sqrtf(1 + n->a * n->a);
        original = wall + (side ? -1 : 1) * out->width * .5f * normal;
        alternate = wall - (side ? -1 : 1) * out->width * .5f * normal;
        if (fabsf(original - predicted) / normal > out->width * .70f &&
            fabsf(alternate - predicted) / normal < out->width * .25f)
        {
            uint8_t clear = 1;
            for (sample = 0; sample < 3 && clear; ++sample)
            {
                float cy = bounded(250.0f + sample * 100, n->min_y, n->max_y);
                float cx = at(n, cy) - (side ? -1 : 1) * out->width * .5f * normal;
                for (i = 0; i < count; ++i)
                {
                    float dx = points[i].x - cx, dy = points[i].y - cy;
                    if (dx * dx + dy * dy < VEHICLE_HALF_CLEAR_MM * VEHICLE_HALF_CLEAR_MM)
                    {
                        clear = 0;
                        break;
                    }
                }
            }
            if (!clear)
                continue; /* 不能为了接上历史而把中心移到当前占用上。 */
            /* 当前另一侧不能证明是独立对墙时，只恢复这一面连续墙。 */
            out->near_fit[1 - side] = *n;
            out->far_fit[1 - side] = out->far_fit[side];
            memset(n, 0, sizeof *n);
            memset(&out->far_fit[side], 0, sizeof out->far_fit[side]);
            n->rejected = out->far_fit[side].rejected = FIT_DEGENERATE;
            out->identity_corrected = 1;
            return;
        }
    }
}

/* 冲突时只允许有独立连续性证据的单侧接管，不因“某侧RMS略小”盲猜。
 * 近远段必须在实际接缝相连，远段覆盖预瞄；候选中线在共同可见区内
 * 还须给另一面保留轮外沿及侧隙，避免单侧半宽偏移指到对墙上。 */
static uint8_t recovery_side(const PathFit *near, const PathFit *far, uint8_t side, float width,
                             float preview)
{
    const PathFit *n = &near[side], *f = &far[side], *other = &near[1 - side];
    float lo, hi, join, y, center, clearance;
    unsigned i;
    if (!n->valid || !f->valid || n->rms > 35)
        return 0;
    join = f->min_y;
    if (n->max_y < join - 50 || f->min_y > preview + 100 || f->max_y < preview - 100 ||
        fabsf(at(n, join) - at(f, join)) > 80 || fabsf(n->a - f->a) > .8f)
        return 0;
    lo = fmaxf(n->min_y, other->min_y);
    hi = fminf(n->max_y, other->max_y);
    if (hi - lo < 200)
        return 0;
    for (i = 0; i < 3; ++i)
    {
        y = lo + (hi - lo) * i * .5f;
        center = at(n, y) + (side ? -1 : 1) * width * .5f * sqrtf(1 + n->a * n->a);
        clearance = (side ? center - at(other, y) : at(other, y) - center) / sqrtf(1 + other->a * other->a);
        if (clearance < VEHICLE_HALF_CLEAR_MM)
            return 0;
    }
    return 1;
}

/* 输入已按雷达角度排列。仅接纳独立、具有圆弧形状的小簇；平墙短片、单点噪声和
 * 连续围挡不作为锥桶。用已知截面圆拟合中心，不能把可见圆弧表面当作底座边缘。 */
static uint8_t cone_center(const PathPoint *points, uint16_t begin, uint16_t end, PathPoint *center)
{
    uint16_t i;
    float dx, dy, chord2, mx, my, h, nx, ny, bulge = 0, residual = 0;
    if (end - begin < 4)
        return 0;
    if (points[begin].y < 125 || points[end - 1].y < 125)
        return 0; /* y=100mm裁剪可能把连续内墙切成小圆弧，不授予“桶”身份。 */
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
        float cross = fabsf(dx * (points[i].y - points[begin].y) - dy * (points[i].x - points[begin].x));
        if (cross > bulge)
            bulge = cross;
        residual += e * e;
    }
    return bulge * bulge >= 4 * 4 * chord2 && residual <= (end - begin) * 12 * 12 && center->y >= 100 &&
           center->y <= 1100;
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
            uint8_t use_far =
                out->far_valid && cone.y >= 550 && supports_y(out->far_fit, out->source, cone.y);
            /* 近处用近段，远处有可信远段才使用远段；禁止跨很远外推避让。 */
            a = use_far ? out->far_a : out->near_a;
            ++out->cone_candidates;
            path_x = use_far ? out->far_x + a * (cone.y - out->ref_y)
                             : out->near_x + a * (cone.y - out->near_ref_y);
            scale = sqrtf(1 + a * a);
            lateral = (cone.x - path_x) / scale;
            required = CONE_BASE_ENVELOPE_MM + body_clearance - fabsf(lateral);
            if (required > 0 && cone.y < nearest &&
                (use_far || supports_y(out->near_fit, out->source, cone.y)) &&
                fabsf(lateral) <= out->width * 0.5f + 60)
            {
                /* 中线附近沿既有避让方向通过，避免圆心小幅抖动导致左右反复切换。 */
                shift = fabsf(lateral) < 30 && state->avoid_offset != 0
                            ? (state->avoid_offset > 0 ? required : -required)
                            : (lateral <= 0 ? required : -required);
                /* 1.1m开始平滑介入，800mm内给出完整避让偏移。 */
                shift *= bounded((1100 - cone.y) / 300, 0, 1);
                requested = bounded(shift * scale, -allowance * normal, allowance * normal);
                nearest = cone.y;
                out->cone_x = cone.x;
                out->cone_y = cone.y;
                out->cone_lateral = lateral;
                out->avoid_required = required;
                out->avoid_allowed = allowance;
            }
        }
        begin = end;
    }
    if (nearest < 1e9f)
    {
        out->avoid_y = nearest;
        /* 两帧圆弧中心相容才介入；反侧新目标重新确认，不能继承旧目标的资格。
         * 纵向允许150 mm接近量，横向100 mm；单帧波纹/反光不能直接改变舵机。 */
        if (!state->candidate_frames || fabsf(out->cone_x - state->candidate_x) > 100 ||
            fabsf(out->cone_y - state->candidate_y) > 150)
            state->candidate_frames = 1;
        else if (state->candidate_frames < 255)
            ++state->candidate_frames;
        state->candidate_x = out->cone_x;
        state->candidate_y = out->cone_y;
        out->avoid_confirm = state->candidate_frames;
        out->avoid_state = AVOID_CANDIDATE;
        if (state->candidate_frames >= 2)
        {
            state->avoid_offset = requested;
            state->avoid_hold = 3;
            out->avoid_state = AVOID_ACTIVE;
        }
    }
    else
    {
        state->candidate_frames = 0;
        if (state->avoid_hold)
        {
            --state->avoid_hold;
            out->avoid_state = AVOID_HOLD;
        }
        else
        {
            state->avoid_offset += bounded(-state->avoid_offset, -20, 20);
            out->avoid_state = state->avoid_offset ? AVOID_RELEASE : AVOID_CLEAR;
        }
    }
    state->avoid_offset = bounded(state->avoid_offset, -allowance * normal, allowance * normal);
    out->avoid_offset = state->avoid_offset;
    out->target_x += out->avoid_offset;
}

static void build_lines(PathGeometry *state, const PathPoint *points, uint16_t count, float configured_width,
                        float preview_y, float center_x, PathObservation *out)
{
    PathPoint samples[2][STRIPS];
    PathFit *near, *far;
    uint16_t n[2], original_n[2];
    uint8_t side, source;
    float width, near_y = 400.0f, far_y, slope, lo, hi;
    memset(out, 0, sizeof *out);
    out->source = PATH_NONE;
    out->geometry_reason = GEOM_ARGUMENT;
    out->width_frozen = 1;
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
        state->candidate_frames = 0;
        Path_ClearHistory(state);
    }
    out->width = state->width;
    if (!points || count > 400)
        return;
    for (side = 0; side < 2; ++side)
        n[side] = boundary(points, count, side ? 1 : -1, state->width, center_x, samples[side]);
    original_n[0] = n[0]; original_n[1] = n[1];
    out->duplicate_samples = separate_boundaries(samples, n);
    /* 只在实际重合时额外复核，普通道路不增加两次拟合。 */
    if (out->duplicate_samples)
    {
        PathFit left = fit(samples[0], original_n[0], 100, 750);
        PathFit right = fit(samples[1], original_n[1], 100, 750);
        out->duplicate_conflict = left.valid && right.valid;
    }
    for (side = 0; side < 2; ++side)
    {
        out->near_fit[side] = fit(samples[side], n[side], 100, 750);
        out->far_fit[side] = fit(samples[side], n[side], 550, 1300);
    }
    restore_wall_identity(state, points, count, out);
    near = out->near_fit;
    far = out->far_fit;
    /* 弯道尺度限制须先于来源选择，否则900mm支持不足会错拒原有700mm路径。
     * 远段仍读550~1300mm；只限制实际跟踪目标，不删掉前方弯向证据。 */
    for (side = 0; side < 2; ++side)
        if (preview_y > 700 && ((near[side].valid && fabsf(near[side].a) > .14f) ||
                               (far[side].valid && fabsf(far[side].a) > .14f)))
            preview_y = 700;
    out->geometry_reason = GEOM_NEAR_MISSING;
    if (!near[0].valid && !near[1].valid)
    {
        state->candidate_frames = 0;
        state->avoid_hold = 0;
        state->avoid_offset = 0; /* 断开的几何不能延续旧锥桶确认。 */
        return;
    }
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
            uint8_t left = recovery_side(near, far, 0, state->width, preview_y);
            uint8_t right = recovery_side(near, far, 1, state->width, preview_y);
            /* 真实回放中内侧结束、外侧连续，却因斜率差被整帧弃用。
             * 优先唯一连续候选；两个候选同样可信且冲突时仍拒绝猜方向。 */
            if (left != right)
                source = left ? PATH_LEFT : PATH_RIGHT;
            else if (near[0].rms + 20 < near[1].rms)
                source = PATH_LEFT;
            else if (near[1].rms + 20 < near[0].rms)
                source = PATH_RIGHT;
            else
            {
                out->geometry_reason = GEOM_CONFLICT;
                state->candidate_frames = 0;
                return;
            }
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
    /* 默认仍取400 mm；短暂遮挡时在250~450 mm内选择有真实支持的位置。
     * 不因固定参考恰好落在可见段之外而整帧弃用，也不向无回波区域外推。 */
    near_y = bounded(near_y, fmaxf(lo, 250), fminf(hi, 450));
    if (fmaxf(lo, 250) > fminf(hi, 450))
    {
        out->geometry_reason = GEOM_SUPPORT;
        state->candidate_frames = 0;
        return;
    }
    out->near_ref_y = near_y;
    /* 本帧可靠双侧的间距可供当前避让/单侧远段使用；是否写入历史另行裁决。
     * 否则宽赛段的锥桶遮住一侧远段时，会误用旧500mm宽度把道路中心移偏。 */
    if (source == PATH_DUAL)
    {
        out->width = out->width_candidate;
        out->width_measured = 1; /* 本帧实测与写入历史是两个不同状态。 */
    }
    if (source == PATH_DUAL && fabsf(near[0].a) < .10f && fabsf(near[1].a) < .10f && far[0].valid &&
        far[1].valid && fabsf(far[0].a) < .10f && fabsf(far[1].a) < .10f && near[0].rms < 30 &&
        near[1].rms < 30 && !state->avoid_offset && !state->candidate_frames)
    {
        /* 只在近远段一致的直道学习宽度，弯道透视变化不能污染后续单侧偏移。 */
        width = out->width_candidate;
        state->width = state->measured ? 0.75f * state->width + 0.25f * width : width;
        state->measured = 1;
        out->width_frozen = 0;
        out->width = state->width;
    }
    /* 双侧近段可信、但一侧远段被遮挡时，用另一侧连续边界保留预瞄。
     * 必须在两段实际接缝处位置连续，不能把横墙/下一个不相干物体接成远段。 */
    if (source == PATH_DUAL && far[0].valid != far[1].valid)
    {
        uint8_t available = far[1].valid ? 1 : 0;
        float join_y = far[available].min_y;
        if (near[available].max_y >= join_y - 50 && far[available].min_y <= preview_y + 100 &&
            far[available].max_y >= preview_y - 100 &&
            fabsf(at(&near[available], join_y) - at(&far[available], join_y)) <= 80 &&
            fabsf(near[available].a - far[available].a) <= 0.8f)
        {
            source = available ? PATH_RIGHT : PATH_LEFT;
            side = available;
        }
    }
    out->source = source;
    out->valid = 1;
    out->geometry_reason = GEOM_OK;
    out->near_x = center_at(&near[0], &near[1], source, out->width, near_y);
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
            out->far_x = center_at(&far[0], &far[1], source, out->width, far_y);
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
        out->target_x = center_at(&near[0], &near[1], source, out->width, out->ref_y);
    }
    out->straight = out->far_valid && fabsf(out->near_a) < 0.10f && fabsf(out->far_a) < 0.10f &&
                    fabsf(out->near_a - out->far_a) < 0.10f && fabsf(out->far_x - out->near_x) < 60;
    avoid_cones(state, points, count, center_x, out);
}

/* 急弯补充模型只处理同帧、相邻射线的连续轮廓；固定空间抽样而非密集点投票。
 * 不扩大原直线RMS容差，不把没有回波的区域当作出口。最多32个等距工作点。 */
#define CURVE_N 32

typedef struct
{
    PathPoint wall[CURVE_N], center[CURVE_N];
    float yaw[CURVE_N];
    uint16_t n;
} CurveWork;

static float point_distance(PathPoint a, PathPoint b)
{
    float x = a.x - b.x, y = a.y - b.y;
    return sqrtf(x * x + y * y);
}

/* 每条中心线小段均检查所有已见占用；150mm包含130mm轮外沿及20mm余量。
 * 这是可见范围的局部通路检查，不冒充后轴运动学或不可见侧后方扫掠。 */
static uint8_t corridor_clear(const PathPoint *points, uint16_t count, PathPoint a, PathPoint b)
{
    uint16_t i;
    float dx = b.x - a.x, dy = b.y - a.y, length2 = dx * dx + dy * dy;
    if (length2 < 1)
        return 0;
    for (i = 0; i < count; ++i)
    {
        float u = bounded(((points[i].x - a.x) * dx + (points[i].y - a.y) * dy) / length2, 0, 1);
        float x = points[i].x - a.x - u * dx, y = points[i].y - a.y - u * dy;
        if (x * x + y * y < VEHICLE_HALF_CLEAR_MM * VEHICLE_HALF_CLEAR_MM)
            return 0;
    }
    return 1;
}

static uint8_t curve_reject(PathObservation *candidate, uint32_t reason)
{
    candidate->curve_checks |= reason;
    return 0;
}

/* 首尾差给出方向，局部正交残差验证形状；方向不依赖y跨度或无穷大斜率。 */
static uint8_t curve_candidate(const PathPoint *points, uint16_t count, uint16_t begin, uint16_t end,
                               int side, float width, float preview, float cx, PathObservation *candidate)
{
    CurveWork w;
    PathPoint seed;
    uint16_t i, j, lo, hi, first, stop;
    int index, step;
    float distance = 0, tx, ty, len, yaw0 = 0, bend = 0, total = 0, best = 1e9f;
    float near_yaw = 0, far_yaw = 0;
    PathPoint near_point = {0, 0}, target = {0, 0};
    preview = fminf(preview, 700); /* 弧形跟踪不随直道提速扩大跨弯弦长。 */
    if (end - begin < 10)
        return curve_reject(candidate, CURVE_SHORT);
    step = points[begin].y <= points[end - 1].y ? 1 : -1;
    index = step > 0 ? begin : end - 1;
    seed = points[index];
    if (seed.y > 400 || side * (seed.x - cx) < 60 || fabsf(seed.x - cx) > width * .5f + 160)
        return curve_reject(candidate, CURVE_SEED);
    w.n = 0;
    for (i = begin; i < end; ++i, index += step)
    {
        PathPoint p = points[index];
        distance = w.n ? point_distance(w.wall[w.n - 1], p) : 0;
        if (w.n && distance < 55)
            continue;
        if (w.n == CURVE_N)
            break;
        w.wall[w.n++] = p;
        distance = 0;
    }
    if (w.n < 7)
        return curve_reject(candidate, CURVE_SHORT);
    for (i = 0; i < w.n; ++i)
    {
        lo = i > 1 ? i - 2 : 0;
        hi = i + 2 < w.n ? i + 2 : w.n - 1;
        tx = w.wall[hi].x - w.wall[lo].x;
        ty = w.wall[hi].y - w.wall[lo].y;
        len = sqrtf(tx * tx + ty * ty);
        if (len < 80)
            return curve_reject(candidate, CURVE_TANGENT);
        tx /= len;
        ty /= len;
        for (j = lo; j <= hi; ++j)
        {
            float e = (w.wall[j].x - w.wall[lo].x) * ty - (w.wall[j].y - w.wall[lo].y) * tx;
            if (fabsf(e) > 45)
                return curve_reject(candidate, CURVE_RESIDUAL);
        }
        w.yaw[i] = atan2f(tx, ty);
        if (!i)
            yaw0 = w.yaw[i];
        if (fabsf(w.yaw[i] - yaw0) > bend)
            bend = fabsf(w.yaw[i] - yaw0);
        if (i && fabsf(w.yaw[i] - w.yaw[i - 1]) > .65f)
            return curve_reject(candidate, CURVE_JUMP);
        w.center[i].x = w.wall[i].x - side * width * .5f * ty;
        w.center[i].y = w.wall[i].y + side * width * .5f * tx;
    }
    /* 直墙不需要补充模型；很短的弧不能因密集点数多伪装成长边界。 */
    if (bend < .25f || fabsf(yaw0) > 1.0f)
        return curve_reject(candidate, CURVE_BEND);
    first = 0;
    for (i = 0; i < w.n; ++i)
    {
        float d = (w.center[i].x - cx) * (w.center[i].x - cx) + w.center[i].y * w.center[i].y;
        if (w.center[i].y >= 100 && d < best)
        {
            best = d;
            first = i;
        }
    }
    if (best == 1e9f || first + 3 >= w.n)
        return curve_reject(candidate, CURVE_SUPPORT);
    {
        PathPoint entry = {0, 100}; /* cx是跟踪偏置，不把它误当作车体中心位置。 */
        if (point_distance(entry, w.center[first]) > 5 &&
            !corridor_clear(points, count, entry, w.center[first]))
            return curve_reject(candidate, CURVE_ENTRY);
    }
    total = sqrtf(best);
    stop = first;
    near_point = target = w.center[first];
    near_yaw = far_yaw = w.yaw[first];
    for (i = first + 1; i < w.n; ++i)
    {
        float old = total, f;
        len = point_distance(w.center[i - 1], w.center[i]);
        if (len < 5 || len > 180 || w.center[i].y < 100)
            break;
        if (!corridor_clear(points, count, w.center[i - 1], w.center[i]))
        {
            candidate->curve_checks |= CURVE_BLOCKED;
            break;
        }
        total += len;
        stop = i;
        if (old < 400)
        {
            f = bounded((400 - old) / len, 0, 1);
            near_point.x = w.center[i - 1].x + f * (w.center[i].x - w.center[i - 1].x);
            near_point.y = w.center[i - 1].y + f * (w.center[i].y - w.center[i - 1].y);
            near_yaw = w.yaw[i - 1] + f * (w.yaw[i] - w.yaw[i - 1]);
        }
        f = bounded((preview - old) / len, 0, 1);
        target.x = w.center[i - 1].x + f * (w.center[i].x - w.center[i - 1].x);
        target.y = w.center[i - 1].y + f * (w.center[i].y - w.center[i - 1].y);
        far_yaw = w.yaw[i - 1] + f * (w.yaw[i] - w.yaw[i - 1]);
        if (total >= preview)
            break;
    }
    if (stop < first + 3 || total < 400 || target.y < 150)
        return curve_reject(candidate, CURVE_SUPPORT);
    candidate->valid = 1;
    candidate->geometry_reason = GEOM_OK;
    candidate->source = side > 0 ? PATH_RIGHT : PATH_LEFT;
    candidate->model = 2;
    candidate->straight = 0;
    candidate->width = width;
    candidate->width_frozen = 1;
    candidate->near_x = near_point.x;
    candidate->near_ref_y = near_point.y;
    candidate->target_x = candidate->far_x = target.x;
    candidate->ref_y = target.y;
    candidate->heading_rad = near_yaw;
    candidate->preview_heading_rad = far_yaw;
    candidate->near_a = bounded(sinf(near_yaw) / fmaxf(cosf(near_yaw), .2f), -1.8f, 1.8f);
    candidate->far_a = bounded(sinf(far_yaw) / fmaxf(cosf(far_yaw), .2f), -1.8f, 1.8f);
    candidate->far_valid = total >= preview;
    candidate->support_mm = fminf(total, preview);
    candidate->avoid_offset = 0;
    candidate->avoid_state = AVOID_CLEAR;
    candidate->bend_rad = bend;
    return 1;
}

static uint8_t recover_curve(const PathPoint *points, uint16_t count, float width, float preview, float cx,
                             PathObservation *out)
{
    uint16_t begin = 0, end;
    uint8_t found = 0;
    PathObservation candidate, best;
    while (begin < count)
    {
        end = begin + 1;
        while (end < count && point_distance(points[end - 1], points[end]) < 100)
            ++end;
        candidate = *out;
        if (curve_candidate(points, count, begin, end, 1, width, preview, cx, &candidate) ||
            curve_candidate(points, count, begin, end, -1, width, preview, cx, &candidate))
        {
            if (found && (candidate.target_x - cx) * (best.target_x - cx) < 0)
            {
                out->curve_rejected = 2;
                out->curve_checks |= CURVE_AMBIGUOUS;
                return 0; /* 两个相反出口都有证据时禁止猜方向。 */
            }
            if (!found || candidate.support_mm > best.support_mm)
                best = candidate;
            found = 1;
        }
        out->curve_checks |= candidate.curve_checks;
        begin = end;
    }
    if (found)
    {
        *out = best;
        return 1;
    }
    out->curve_rejected = 1;
    return 0;
}

/* 无远段时只在共同可见区建立短走廊；不放宽原双侧斜率门槛或外推700mm。 */
static uint8_t recover_local(const PathPoint *points, uint16_t count, PathObservation *out)
{
    const PathFit *l = &out->near_fit[0], *r = &out->near_fit[1];
    float lo = fmaxf(l->min_y, r->min_y), hi = fminf(l->max_y, r->max_y);
    PathPoint previous, p;
    unsigned i;
    if ((out->far_fit[0].rejected & (FIT_RMS | FIT_SLOPE)) ||
        (out->far_fit[1].rejected & (FIT_RMS | FIT_SLOPE)))
        return 0;
    if (!l->valid || !r->valid || l->rms > 30 || r->rms > 30 || hi - lo < 250 || lo > 250 || hi < 400 ||
        fabsf(l->a - r->a) > .65f)
        return 0;
    hi = fminf(hi, 550);
    previous.x = .5f * (at(l, lo) + at(r, lo));
    previous.y = lo;
    for (i = 1; i <= 6; ++i)
    {
        float width;
        p.y = lo + (hi - lo) * i / 6;
        p.x = .5f * (at(l, p.y) + at(r, p.y));
        width = at(r, p.y) - at(l, p.y);
        if (width < 340 || width > 900 || !corridor_clear(points, count, previous, p))
            return 0;
        previous = p;
    }
    out->valid = 1;
    out->geometry_reason = GEOM_OK;
    out->model = 1;
    out->source = PATH_DUAL;
    out->near_ref_y = 400;
    out->near_x = .5f * (at(l, 400) + at(r, 400));
    out->near_a = .5f * (l->a + r->a);
    out->far_valid = 0;
    out->straight = 0;
    out->ref_y = hi;
    out->target_x = p.x;
    out->support_mm = hi;
    return 1;
}

/* 急弯识别使用同一连续回波簇，允许外墙跨越x=0。只保存24个50mm分区，
 * 每簇/每分区有固定上限；不复制整圈、不排序、不分配堆内存。
 * 近切线只看100~400mm，避免把弯内远点混成“当前车身横移”。 */
#define ACUTE_BINS 24
typedef struct
{
    float a, b, yaw, distance;
    uint16_t begin, end;
    uint8_t valid;
} AcuteWall;

static AcuteWall acute_wall(const PathPoint *points, uint16_t count, int side)
{
    AcuteWall out = {0};
    PathPoint bins[ACUTE_BINS] = {{0, 0}};
    uint16_t sizes[ACUTE_BINS] = {0};
    uint16_t begin = 0, end, i, seed, best = 0, n = 0;
    float sx = 0, sy = 0, xy = 0, yy = 0, residual = 0, normal, first = 0, last = 0;
    float departure = 0, peak_turn = 0;
    unsigned streak = 0;
    while (begin < count)
    {
        end = begin + 1;
        while (end < count && point_distance(points[end - 1], points[end]) < 100)
            ++end;
        seed = 0;
        for (i = begin; i < end; ++i)
            if (points[i].y >= 100 && points[i].y < 400 && side * points[i].x > 60)
                ++seed;
        if (seed > best)
        {
            best = seed;
            out.begin = begin;
            out.end = end;
        }
        begin = end;
    }
    if (best < 8) return out;
    for (i = out.begin; i < out.end; ++i)
    {
        int bin;
        if (points[i].y < 100 || points[i].y >= 1300) continue;
        bin = (int)((points[i].y - 100) / 50);
        bins[bin].x += points[i].x;
        bins[bin].y += points[i].y;
        ++sizes[bin];
    }
    for (i = 0; i < ACUTE_BINS; ++i)
    {
        if (sizes[i] < 2) continue;
        bins[i].x /= sizes[i]; bins[i].y /= sizes[i];
        if (i >= 6) continue;
        if (!n) first = bins[i].y;
        last = bins[i].y;
        sx += bins[i].x; sy += bins[i].y;
        xy += bins[i].x * bins[i].y; yy += bins[i].y * bins[i].y;
        ++n;
    }
    if (n < 5 || first > 180 || last < 325 || last - first < 200) return out;
    yy -= sy * sy / n;
    if (yy < 1) return out;
    out.a = (xy - sx * sy / n) / yy;
    out.b = (sx - out.a * sy) / n;
    normal = sqrtf(1 + out.a * out.a);
    for (i = 0; i < 6; ++i)
        if (sizes[i] >= 2)
        {
            float e = (bins[i].x - out.a * bins[i].y - out.b) / normal;
            residual += e * e;
        }
    if (sqrtf(residual / n) > 12) return out;
    out.valid = 1;
    out.yaw = atanf(out.a);
    /* 三个连续分区向弯内偏离至少40mm，且后续200mm切线转过至少0.60rad。
     * 这是有限距离内的持续弯折，不能用远斜率大或两直线交点代替。
     * 可见尾部出现反向折返时拒绝独立急弯，交给已有S弯分支。 */
    for (i = 6; i < ACUTE_BINS; ++i)
    {
        float deviation, turn;
        uint16_t j;
        if (sizes[i] < 2) { streak = 0; continue; }
        deviation = -side * (bins[i].x - out.a * bins[i].y - out.b) / normal;
        if (deviation >= 40)
        {
            if (!streak) departure = bins[i].y;
            if (++streak == 3 && !out.distance) out.distance = departure;
        }
        else streak = 0;
        if (i < 10 || sizes[i - 4] < 2) continue;
        for (j = i - 4; j <= i && sizes[j] >= 2; ++j) {}
        if (j <= i) continue; /* 不跨缺失分区凑出200mm转角证据。 */
        turn = -side * (atan2f(bins[i].x - bins[i - 4].x,
                              bins[i].y - bins[i - 4].y) - out.yaw);
        if (turn < -.20f || (peak_turn >= .60f && turn < peak_turn - .35f))
        { out.distance = 0; return out; }
        peak_turn = fmaxf(peak_turn, turn);
        if (out.distance && turn >= .60f && bins[i].y - out.distance <= 400)
            out.valid = 2;
    }
    if (out.valid != 2) out.distance = 0;
    return out;
}

/* 入口准备只修改有实测近墙支持的局部目标，弯向仍来自原道路目标。
 * 主弯接管优先于达到40mm；进入TURN后不允许重新退到PREPARE追偏移。
 * 没有实际舵角/毫米速度标定，700/950只是保守试验窗口，不声称最优距离。 */
static void acute_prepare(PathGeometry *state, const PathPoint *points, uint16_t count,
                        float cx, PathObservation *out)
{
    AcuteWall wall[2];
    float yaw, a, b, normal, width, available, goal, weight, local_y;
    int direction = 0, side;
    PathPoint entry = {0, 100}, target;
    out->road_target_x = out->target_x;
    out->outer_offset = 0;
    out->acute_phase = ACUTE_OFF;
    out->acute_reason = ACUTE_NO_EVIDENCE;
    if (out->s_preview || out->avoid_state != AVOID_CLEAR || out->avoid_offset)
    {
        acute_clear(state);
        out->acute_reason = ACUTE_PRIORITY;
        return;
    }
    /* 近远同向的整条新路径也可能已反向，不一定带s_preview标志。
     * 必须先撤销旧入口，再判断模型/航向接管，不能多保留一帧旧弯方向。 */
    if (state->acute_phase != ACUTE_OFF &&
        (out->near_a * state->acute_direction < -.10f ||
         (out->far_valid && out->far_a * state->acute_direction < -.14f)))
    {
        acute_clear(state);
        out->acute_reason = ACUTE_PRIORITY;
        return;
    }
    /* 已进入主弯时只观察出口；原有循迹负责减舵，不增加定时锁舵。
     * 近远反向、真实新弯或感知失效立即取消附加阶段。 */
    if (state->acute_phase == ACUTE_TURN || state->acute_phase == ACUTE_EXIT)
    {
        if (out->straight)
        {
            if (++state->acute_exit_frames >= 2)
                acute_clear(state);
            else state->acute_phase = ACUTE_EXIT;
            out->acute_reason = ACUTE_EXIT_SEEN;
        }
        else
        {
            state->acute_exit_frames = 0;
            state->acute_phase = ACUTE_TURN;
            out->acute_reason = ACUTE_NEAR_TURN;
        }
        out->acute_phase = state->acute_phase;
        out->acute_direction = state->acute_direction;
        return;
    }
    /* 只有经历本轮入口确认的弯才拥有新增主弯保护。仅靠近远同号、
     * 远角度大就冷启动TURN会误伤S弯（连续快照已复现），因此禁止。 */
    if (state->acute_phase == ACUTE_PREPARE &&
        (out->model || fabsf(out->heading_rad) > .50f))
    {
        if (out->near_a * state->acute_direction > .10f)
        {
            state->acute_phase = ACUTE_TURN;
            out->acute_phase = ACUTE_TURN;
            out->acute_direction = state->acute_direction;
            out->acute_reason = ACUTE_NEAR_TURN;
        }
        else
        {
            acute_clear(state);
            out->acute_reason = ACUTE_LOST;
        }
        return;
    }
    /* 预滤波的唯一职责是"直道上省掉分区扫描"，不是决定有没有急弯——那是
     * acute_wall() 的严格门限该做的事（种子≥8点、≥5个50mm分区、跨度≥200mm、
     * RMS≤12mm、连续3个分区向内偏≥40mm、400mm内切线转过≥0.60rad）。
     * 2026-10-13：原条件额外要求"某侧远段拟合有效且|a|>0.30"。但实车数据
     * （10/12 T01~T07，fw=9b8c0a52；10/13 六组）里急弯分支 181 行只进过 1 行，
     * 而进弯前那一刻 |近a| 中位数只有 0.09~0.11（近段本来就该是直的）、
     * 远段窗口在弯口既不共线又关联不上，被 FIT_POINTS|FIT_SPAN 拒掉——
     * 于是"要求远段"等于把证据链断在第一步。
     * 改为只跳过"已确认的直道"：S弯(s_preview)与锥桶(avoid_state)在更前面已
     * 优先返回，缓弯达不到 acute_wall() 的 40mm/0.60rad 门限，因此不会误激活。 */
    if (out->model || (!state->acute_phase && out->straight))
    {
        acute_clear(state);
        return;
    }
    wall[0] = acute_wall(points, count, -1);
    wall[1] = acute_wall(points, count, 1);
    if (!wall[0].valid || !wall[1].valid || wall[0].begin == wall[1].begin ||
        fabsf(wall[0].yaw - wall[1].yaw) > .12f)
    {
        acute_clear(state);
        out->acute_reason = ACUTE_LOST;
        return;
    }
    if (wall[0].distance && !wall[1].distance) direction = 1;
    if (wall[1].distance && !wall[0].distance) direction = -1;
    if (!direction)
    {
        acute_clear(state);
        return;
    }
    side = direction > 0 ? 0 : 1;
    a = .5f * (wall[0].a + wall[1].a);
    b = .5f * (wall[0].b + wall[1].b);
    yaw = atanf(a); normal = sqrtf(1 + a * a);
    width = ((wall[1].a - wall[0].a) * 100 + wall[1].b - wall[0].b) / normal;
    out->acute_position = direction * (a * 100 + b) / normal;
    out->acute_heading = yaw;
    out->acute_distance = wall[side].distance;
    out->acute_direction = (int8_t)direction;
    /* 宽度越界是入口证据失效，不是主弯到来；不能赋予短时保舵权限。 */
    if (width < 390 || width > 650 || yaw * direction < -.22f)
    {
        acute_clear(state);
        out->acute_reason = width < 390 ? ACUTE_NO_ROOM : ACUTE_LOST;
        return;
    }
    if (yaw * direction > .22f || out->acute_distance <= ACUTE_TAKEOVER_MM)
    {
        if (state->acute_phase == ACUTE_PREPARE && state->acute_direction == direction)
        {
            state->acute_phase = ACUTE_TURN;
            out->acute_phase = ACUTE_TURN;
        }
        else acute_clear(state);
        out->acute_reason = ACUTE_NEAR_TURN;
        return;
    }
    /* position的参考在前轴：前轴到后轮外廓305+40=345mm，而非车长的一半。
     * 使用385x260mm轮廓在当前姿态的保守横向投影，再留40mm余量。
     * 这里只约束入口局部平行走廊，不代表未知后方和转向轮扫掠已经验证。 */
    available = width * .5f - 130 * cosf(yaw) - 345 * fabsf(sinf(yaw)) - 40;
    goal = fminf(ACUTE_OUTER_MM, available);
    if (goal < 10)
    {
        acute_clear(state);
        out->acute_reason = ACUTE_NO_ROOM;
        return;
    }
    if (state->acute_direction != direction)
        acute_clear(state);
    state->acute_direction = (int8_t)direction;
    if (state->acute_frames < 2) ++state->acute_frames;
    state->acute_phase = state->acute_frames < 2 ? ACUTE_CONFIRM : ACUTE_PREPARE;
    out->acute_phase = state->acute_phase;
    out->acute_goal = goal;
    out->acute_reason = state->acute_frames < 2 ? ACUTE_CONFIRMING : ACUTE_POSITIONING;
    if (state->acute_frames < 2) return;
    /* 目标是绝对局部位置，已在外侧不会再累计40mm。沿近切线追踪也会消除
     * 向外的航向误差，不能只有横移而一直朝外墙。cx仍用于普通循迹；这里
     * 显式抵消其数值，保证goal=40表示相对于实际局部中线的40mm。 */
    weight = bounded((out->acute_distance - ACUTE_TAKEOVER_MM) /
                     (ACUTE_FULL_PREP_MM - ACUTE_TAKEOVER_MM), 0, 1);
    local_y = 450;
    target.x = (1 - weight) * out->target_x +
               weight * (a * local_y + b - direction * goal * normal + cx);
    target.y = (1 - weight) * out->ref_y + weight * local_y;
    if (!corridor_clear(points, count, entry, target))
    {
        acute_clear(state);
        out->acute_phase = ACUTE_OFF;
        out->acute_reason = ACUTE_NO_ROOM;
        return;
    }
    out->outer_offset = -direction * goal * weight;
    out->outer_clearance = width * .5f - goal;
    out->target_x = target.x;
    out->ref_y = target.y;
}


/* 已有实车基线的远目标25mm微调，仅在新入口分支未接管时使用。
 * 它不是车身位置控制；保留它是为了让未确认独立急弯的S弯/普通弯逐帧不变。 */
static void baseline_curve_offset(PathGeometry *state, const PathPoint *points, uint16_t count,
                        PathObservation *out)
{
    float near_yaw = out->heading_rad, far_yaw = out->preview_heading_rad;
    float requested = 0, normal, available, yaw;
    uint8_t side;
    PathPoint before, after;
    const PathFit *wall;
    out->outer_offset = 0;
    if (out->s_preview || out->avoid_state != AVOID_CLEAR || out->avoid_offset)
    {
        state->baseline_offset = 0;
        state->baseline_frames = 0;
        state->baseline_direction = 0;
        return;
    }
    if (out->far_valid && near_yaw * far_yaw >= 0 && fabsf(near_yaw) < .6f &&
        fabsf(far_yaw) > .6f && fabsf(far_yaw) - fabsf(near_yaw) > .35f)
    {
        int8_t direction = far_yaw > 0 ? 1 : -1;
        if (state->baseline_direction != direction)
        {
            state->baseline_frames = 0;
            state->baseline_offset = 0;
            state->baseline_direction = direction;
        }
        if (state->baseline_frames < 2) ++state->baseline_frames;
        if (state->baseline_frames >= 2)
            requested = far_yaw > 0 ? -25.0f : 25.0f;
    }
    else
        state->baseline_frames = 0;
    /* 出口/入弯准备结束后每帧释放10mm；反向证据在上面的分支立即清零。 */
    requested = state->baseline_offset + bounded(requested - state->baseline_offset, -10, 10);
    if (!requested)
        return;
    side = requested < 0 ? 0 : 1;
    wall = &out->far_fit[side];
    if (!wall->valid || out->ref_y < wall->min_y || out->ref_y > wall->max_y)
    {
        state->baseline_offset = 0;
        return;
    }
    normal = sqrtf(1 + wall->a * wall->a);
    available = (side ? at(wall, out->ref_y) - out->target_x :
                       out->target_x - at(wall, out->ref_y)) / normal;
    out->outer_clearance = available;
    available = fmaxf(available - VEHICLE_HALF_CLEAR_MM, 0);
    requested = bounded(requested, -fminf(available, 25.0f),
                         fminf(available, 25.0f));
    yaw = out->preview_heading_rad;
    before.x = out->target_x; before.y = out->ref_y;
    after.x = before.x + requested * cosf(yaw);
    after.y = before.y - requested * sinf(yaw);
    /* 包含目标横移经过的区域；不把单侧拟合误当成已验证的全车扫掠。 */
    if (!requested || !corridor_clear(points, count, before, after))
    {
        state->baseline_offset = 0;
        return;
    }
    state->baseline_offset = out->outer_offset = requested;
    out->target_x = after.x;
    out->ref_y = after.y;
}

static void acute_entry(PathGeometry *state, const PathPoint *points, uint16_t count,
                        float cx, PathObservation *out)
{
    acute_prepare(state, points, count, cx, out);
    if (out->acute_phase == ACUTE_OFF || out->acute_phase == ACUTE_CONFIRM)
        baseline_curve_offset(state, points, count, out);
    else
    {
        state->baseline_offset = 0;
        state->baseline_frames = 0;
        state->baseline_direction = 0;
    }
}

void Path_Build(PathGeometry *state, const PathPoint *points, uint16_t count, float configured_width,
                float preview_y, float center_x, PathObservation *out)
{
    build_lines(state, points, count, configured_width, preview_y, center_x, out);
    if (out->geometry_reason == GEOM_ARGUMENT || !points || count > 400)
    {
        Path_ClearHistory(state);
        return;
    }
    if (out->valid && out->duplicate_conflict)
    {
        PathPoint entry = {0, 100};
        PathPoint near_point = {out->near_x, out->near_ref_y};
        PathPoint target = {out->target_x, out->ref_y};
        /* 去重只证明墙面身份，不证明直线穿过的区域可走。
         * 贴墙或跨弯弦线受阻时交给轮廓恢复，不能把旧拒绝现场强行放行。 */
        if (!corridor_clear(points, count, entry, near_point) ||
            !corridor_clear(points, count, near_point, target))
        {
            out->valid = 0;
            out->geometry_reason = GEOM_CONFLICT;
        }
    }
    if (out->valid && !out->avoid_offset && out->avoid_state == AVOID_CLEAR)
    {
        const PathFit *n = &out->near_fit[out->source == PATH_RIGHT ? 1 : 0];
        uint8_t coherent_turn = out->far_valid && out->near_a * out->far_a > 0 &&
                               fabsf(out->far_a) > .85f &&
                               fabsf(atanf(out->far_a) - atanf(out->near_a)) > .25f;
        /* 除原来的高残差接管，再覆盖“直线仍过检，但轮廓明确转弯”的入口。
         * 近远反向不走这个新分支，保留既有S弯的局部切换能力。 */
        if (out->source != PATH_DUAL &&
            ((!out->far_valid && n->rms > 45 && fabsf(n->a) > .6f) || coherent_turn ||
             (out->duplicate_samples && fabsf(n->a) > .8f) ||
             (state->history_model == 2 && out->near_a * state->history_a > 0 &&
              (!out->far_valid || out->near_a * out->far_a > 0) && fabsf(n->a) > .6f)))
        {
            PathObservation candidate = *out;
            if (recover_curve(points, count, state->width, preview_y, center_x, &candidate) &&
                (candidate.target_x - center_x) * (out->target_x - center_x) > 0 &&
                (!coherent_turn || fabsf(candidate.target_x - center_x) *
                 bounded(700 / fmaxf(candidate.ref_y, 250), 1, 1.5f) >=
                 .85f * fabsf(out->target_x - center_x)))
                *out = candidate; /* 弧形证据已充分时提前接管，普通/低残差S弯保持原分支。 */
            else
                out->curve_checks |= candidate.curve_checks;
        }
    }
    if (!out->valid)
    {
        if (!recover_curve(points, count, state->width, preview_y, center_x, out))
            recover_local(points, count, out);
    }
    if (!out->valid)
    {
        if (++state->history_misses >= 2) Path_ClearHistory(state);
        acute_clear(state);
        state->baseline_offset = 0;
        state->baseline_frames = 0;
        out->acute_reason = ACUTE_LOST;
        return;
    }
    if (out->model)
    {
        state->avoid_offset = 0;
        state->avoid_hold = 0;
        state->candidate_frames = 0;
    }
    if (!out->model)
    {
        out->heading_rad = atanf(out->near_a);
        out->preview_heading_rad = out->far_valid ? atanf(out->far_a) : out->heading_rad;
        out->support_mm = out->ref_y;
    }
    else if (out->model == 1)
    {
        out->heading_rad = out->preview_heading_rad = atanf(out->near_a);
    }
    /* S预告只描述近远段相反，不能直接改写当前弯向或最终PWM。 */
    out->s_preview = out->far_valid && out->near_a * out->far_a < 0 && fabsf(out->near_a) > .10f &&
                     fabsf(out->far_a) > .14f;
    out->turn_distance_mm = out->s_preview ? out->near_ref_y : out->support_mm;
    if (!out->model)
        out->bend_rad = out->far_valid ? fabsf(out->preview_heading_rad - out->heading_rad) : 0;
    {
        float target = (out->target_x - center_x) * bounded(700 / fmaxf(out->ref_y, 250), 1, 1.5f);
        if (state->history && !out->s_preview && !out->avoid_offset)
        {
            out->target_delta = target - state->history_target;
            out->target_transition = fabsf(out->target_delta) > 120 &&
                                     out->near_a * state->history_a > 0 &&
                                     fabsf(out->near_a) > .35f &&
                                     fabsf(atanf(out->near_a) - atanf(state->history_a)) < .25f;
        }
        state->history = !out->s_preview && out->avoid_state == AVOID_CLEAR;
        state->history_misses = 0;
        state->history_x = out->near_x;
        state->history_y = out->near_ref_y;
        state->history_a = out->near_a;
        state->history_model = out->model;
        state->history_target = target;
    }
    acute_entry(state, points, count, center_x, out);
}

float Path_Preview(float base_mm, float speed)
{
    /* 未标定m/s时采用明确的控制单位增量；只扩大识别，不允许超出实际支持。 */
    return bounded(base_mm + 40 * bounded(fabsf(speed) - 12, 0, 5), 600, 1100);
}

float Path_SpeedLimit(PathSpeed *s, const PathObservation *p, uint8_t valid, float ceiling, float bend_speed,
                      uint32_t now)
{
    float floor = fminf(ceiling, bend_speed), curve;
    uint8_t straight =
        valid && p->valid && !p->model && p->straight && !p->avoid_offset && p->ref_y >= 700 && !p->s_preview;
    s->reason = 1;
    if (!straight)
    {
        s->straight_seen = 0;
        s->straight_us = now;
    }
    else if (!s->straight_seen)
    {
        s->straight_seen = 1;
        s->straight_us = now;
    }
    s->last_us = now;
    if (!valid || !p->valid)
    {
        s->reason = 3;
        return s->target = floor;
    }
    if (p->model || p->avoid_state || p->s_preview)
        return s->target = floor;
    curve = fmaxf(fabsf(p->near_a), p->far_valid ? fabsf(p->far_a) : fabsf(p->near_a));
    if (straight && (uint32_t)(now - s->straight_us) >= 1000000u)
    {
        s->reason = 0;
        return s->target = ceiling;
    }
    if (curve < .25f && p->far_valid && !p->s_preview)
    {
        s->reason = 2;
        return s->target = fminf(ceiling, floor + 2);
    }
    return s->target = floor;
}

float Path_SpeedStep(float current, float ceiling, float limit, float bend_speed, uint8_t fresh, float accel,
                     float decel, float dt)
{
    float target;
    if (!(ceiling > 0))
        return 0; /* 仅手动spd 0，不是雷达自动停车。 */
    target = fminf(ceiling, fresh ? limit : bend_speed);
    target = fmaxf(fminf(ceiling, bend_speed), target);
    if (!(current > 0))
        return fminf(target, bend_speed);
    return current + bounded(target - current, -decel * dt, accel * dt);
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
    uint8_t curved, wait = 0, preparing, acute_far_loss, acute_handoff;
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
        state->transition_frames = 0;
        state->transition_used = 0;
        state->previous_acute_phase = ACUTE_OFF;
        return;
    }
    curved = fabsf(path->near_a) > 0.14f ||
             (path->far_valid && (fabsf(path->far_a) > 0.14f || fabsf(path->far_x - path->near_x) > 70));
    preparing = path->acute_phase == ACUTE_PREPARE;
    /* 弯向状态依据道路本身；布位和避让都不能制造假的S弯换向。 */
    out->road_error = g->center_x -
                      ((preparing ? path->road_target_x : path->target_x) - path->avoid_offset);
    out->error = out->road_error;
    wanted = sign(out->error);
    /* 连续的准备->同向主弯已有入口证据。此时远段短暂丢失也不能让上一
     * 弯的bend再拦一次接管；只给本帧权限，不放宽普通S弯的反向门槛。 */
    acute_handoff = state->history && state->previous_acute_phase == ACUTE_PREPARE &&
                     path->acute_phase == ACUTE_TURN && path->acute_direction != 0 &&
                     !path->s_preview && !path->straight && path->avoid_state == AVOID_CLEAR &&
                     path->avoid_offset == 0 && (uint32_t)(now - state->previous_us) <= 250000u &&
                     path->near_a * path->acute_direction > .10f &&
                     wanted == -path->acute_direction && fabsf(out->error) > 30;
    /* 来源只是左/右墙身份，绝不规定输出方向。可靠反向路径当帧接管。 */
    if (preparing)
    {
        /* 布位方向可能与主弯相反，但不改写道路弯向，也不等待S弯反向确认。 */
        state->evidence_frames = 0;
    }
    else if (acute_handoff)
    {
        if (state->bend && state->bend != wanted) out->reason = PATH_REVERSE;
        state->bend = wanted;
        state->pending_direction = 0;
        state->evidence_frames = 0;
    }
    else if (state->bend && path->straight)
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
    out->gate_reason = out->reason;
    if (preparing)
        out->error = g->center_x - path->target_x;
    /* 只有本帧再次确认的碰撞目标可覆盖保舵。旧偏移保持/释放不具备这个权限，
     * 否则会先把舵机拉回中位，再由弯向状态锁在中位（第三次遥测已出现）。 */
    if (path->avoid_state == AVOID_ACTIVE && path->avoid_offset != 0 && wait)
    {
        wait = 0;
        out->avoid_override = 1;
        out->reason = PATH_APPLY;
    }
    /* 高速S预告只允许小幅提前收旧弯舵量，不能改变目标符号或绕过换向证据。
     * <=12沿用原控制；>12最多减少10%且不超过35mm，避免重复叠加航向P。 */
    if (g->speed > 12 && path->s_preview && !path->model && !path->avoid_offset &&
        fabsf(path->near_a) < .65f && fabsf(out->error) > 30 && sign(-path->near_a) == sign(out->error))
    {
        float fraction = bounded((g->speed - 12) * .02f, 0, .10f);
        out->error -= sign(out->error) * fminf(fabsf(out->error) * fraction, 35);
    }
    /* 将实际可见目标的横向误差换到700mm基准的有界角度尺度。
     * 仅改变控制误差尺度，不制造700mm处的虚假点；短参考最多放大1.5倍。
     * road_error仍保存原始毫米误差供诊断，避让横移不重复放大。 */
    if (path->ref_y >= 250 && path->ref_y < 700)
        out->error *= bounded(700.0f / path->ref_y, 1, 1.5f);
    out->error -= path->avoid_offset;
    /* 状态确定后限幅：确认出弯的这一帧就恢复直道限幅。 */
    limit = curved || state->bend ? 500.0f : g->straight_limit;
    out->error = bounded(out->error, -limit, limit);
    /* 远段丢失不等于出弯。补充已确认急弯、近斜率0.35~0.6的漏检区间，
     * 仅该事件将两帧减舵步长收至40mm；旧有大斜率分支仍为80mm。
     * 新主弯阶段同一弯向只触发一次，跳变不能逐帧续期；未接管的场景
     * 保留原80mm/两帧规则，避免改变已经实测良好的普通弯和S弯。 */
    acute_far_loss = path->acute_phase == ACUTE_TURN && state->previous_far_valid &&
                     !path->far_valid && fabsf(atanf(state->previous_far_a)) > .70f &&
                     fabsf(path->near_a) > .35f;
    if (!state->history || path->straight || path->s_preview ||
        sign(out->error) != sign(state->previous_error) ||
        path->avoid_state != AVOID_CLEAR || preparing || path->acute_phase != ACUTE_TURN)
        state->transition_used = 0;
    if (!state->transition_used && !preparing &&
        state->previous_acute_phase != ACUTE_PREPARE &&
        (path->target_transition || acute_far_loss))
    {
        state->transition_frames = 2;
        state->transition_step = acute_far_loss ? 40 : 80;
    }
    if (state->transition_frames && state->history && !path->s_preview && !path->straight &&
        !preparing && state->previous_acute_phase != ACUTE_PREPARE &&
        path->avoid_state == AVOID_CLEAR && path->avoid_offset == 0 &&
        (uint32_t)(now - state->previous_us) <= 250000u &&
        fabsf(path->near_a) > (state->transition_step == 40 ? .35f : .6f) &&
        path->near_a * state->previous_near_a > 0 &&
        fabsf(atanf(path->near_a) - atanf(state->previous_near_a)) < .2f &&
        sign(out->error) == sign(state->previous_error) && fabsf(state->previous_error) > 250)
    {
        float minimum = fmaxf(fabsf(state->previous_error) - state->transition_step, 0);
        if (fabsf(out->error) < minimum)
        {
            out->error = sign(out->error) * minimum;
            out->transition_limited = 1;
            state->transition_used = 1;
        }
        --state->transition_frames;
    }
    else
        state->transition_frames = 0;
    out->kp = path->source != PATH_DUAL ? g->side_kp : curved || state->bend ? g->turn_kp : g->kp;
    out->kd = path->source != PATH_DUAL ? g->side_kd : curved || state->bend ? g->turn_kd : g->kd;
    /* 只增强大弯：250mm内保持原比例，250~500mm连续增加至1.35倍。
     * 默认单边500mm稳态输出约1178/1711，利用余量而不放大直道小误差和D噪声。 */
    if (curved || state->bend)
        out->kp *= 1 + 0.35f * bounded((fabsf(out->error) - 250) / 250, 0, 1);
    dt = (float)(uint32_t)(now - state->previous_us);
    out->d_reset = !state->history || path->source != state->previous_source ||
                   path->far_valid != state->previous_far_valid || path->model != state->previous_model ||
                   fabsf(path->ref_y - state->previous_ref_y) > 100 || dt < 20000 || dt > 250000 ||
                   fabsf(path->avoid_offset - state->previous_avoid_offset) > 30 ||
                   out->transition_limited || path->identity_corrected ||
                   preparing || state->previous_acute_phase == ACUTE_PREPARE ||
                   out->reason == PATH_REVERSE;
    out->p = 10 * out->kp * out->error;
    original_d = out->d_reset ? 0 : 10 * out->kd * (out->error - state->previous_error) * 115000.0f / dt;
    out->d = bounded(original_d, -150, 150);
    if (out->p * (out->p + out->d) < 0)
        out->d = -out->p;
    out->unclamped = g->pwm_mid + out->p + out->d;
    out->candidate_pwm = (uint16_t)bounded(out->unclamped, g->pwm_min, g->pwm_max);
    if (preparing)
        out->candidate_pwm = (uint16_t)bounded(out->candidate_pwm,
            fmaxf(g->pwm_min, (float)actual_pwm - 45), fminf(g->pwm_max, (float)actual_pwm + 45));
    out->computed = 1;
    out->bend = state->bend;
    if (wait)
    {
        const PathFit *near = &path->near_fit[path->source == PATH_RIGHT ? 1 : 0];
        float old_delta = (float)actual_pwm - g->pwm_mid;
        float new_delta = (float)out->candidate_pwm - g->pwm_mid;
        /* 等待确认只限制反打，不冻结有证据的减舵。近段方向已支持目标或
         * 近远段均已直时，允许旧弯舵量减小到中位；确认后才跨中位反打。
         * 残余避让不获得减舵权限，避免恢复旧的“释放偏移强制回中”问题。 */
        if (((near->valid && near->count >= 4 && near->rms <= 35) || path->model == 2) &&
            path->avoid_offset == 0 &&
            (path->straight || (fabsf(path->near_a) > .10f && sign(-path->near_a) == wanted)))
        {
            if (old_delta * new_delta < 0)
                new_delta = 0;
            if (fabsf(new_delta) < fabsf(old_delta))
            {
                out->pwm = (uint16_t)(g->pwm_mid + new_delta);
                out->applied = 1;
                out->reason = PATH_APPLY; /* gate_reason保留原等待原因。 */
            }
        }
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
    state->previous_model = path->model;
    state->previous_far_valid = path->far_valid;
    state->previous_avoid_offset = path->avoid_offset;
    state->previous_near_a = path->near_a;
    state->previous_far_a = path->far_a;
    state->previous_acute_phase = path->acute_phase;
}
