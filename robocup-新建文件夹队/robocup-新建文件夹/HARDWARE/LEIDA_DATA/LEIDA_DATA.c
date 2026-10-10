/** M10P 工作点云：转换前半圈一次，供路径跟踪使用。0° 右、90° 前。 */
#include "LEIDA_DATA.h"
#include <math.h>
_LEIDA_DATA LEIDA_DATA2[LEIDA_DATA_COUNTER];
PathPoint LEIDA_front_points[LEIDA_DATA_COUNTER / 2];
uint16_t valid_couter, LEIDA_speed_dps;
volatile uint16_t LEIDA_raw_count;
float BLUE_ANGLE_LEFT_RIGHT = 90.0f;
float duandian_MIN_Y = 200.0f;

uint16_t LEIDA_FrontPoints(const _LEIDA_DATA *points, uint16_t count)
{
    uint16_t i, n = 0;
    if (count > LEIDA_DATA_COUNTER)
        return 0;
    for (i = 0; i < count; ++i)
    {
        float angle = points[i].angle, distance = points[i].distance, x, y;
        if (!(angle >= 0 && angle <= 180 && distance >= 100 && distance <= 2600))
            continue;
        x = distance * arm_cos_f32(angle * PI / 180);
        y = distance * arm_sin_f32(angle * PI / 180);
        if (!(y >= 100 && y < 1300))
            continue;
        if (n == LEIDA_DATA_COUNTER / 2)
            return 0; /* 容量不完整则整份几何作废 */
        LEIDA_front_points[n].x = x;
        LEIDA_front_points[n].y = y;
        ++n;
    }
    return n;
}

/* 辅助开口检测：独立保留的空间诊断，不参加方向仲裁。 */
static uint16_t boundary_extract(_LEIDA_DATA *out, _LEIDA_DATA *in, uint16_t count, uint8_t left)
{
    static int16_t nearest[181];
    uint16_t i, n = 0, span;
    if (count > LEIDA_DATA_COUNTER)
        count = LEIDA_DATA_COUNTER;
    span = BLUE_ANGLE_LEFT_RIGHT < 0 ? 0 : BLUE_ANGLE_LEFT_RIGHT > 90 ? 90 : (uint16_t)BLUE_ANGLE_LEFT_RIGHT;
    for (i = 0; i <= 180; ++i)
        nearest[i] = -1;
    for (i = 0; i < count; ++i)
    {
        int degree = (int)(in[i].angle + 0.5f);
        float diff;
        if (degree == 360)
            degree = 0;
        if (degree < 0 || degree > 180 || in[i].distance < 100 || in[i].distance > 4000)
            continue;
        diff = fabsf(in[i].angle - degree);
        if (diff > 180)
            diff = 360 - diff;
        if (diff <= 0.5f && (nearest[degree] < 0 || in[i].distance < in[nearest[degree]].distance))
            nearest[degree] = (int16_t)i;
    }
    for (i = 0; i <= span && n < LEIDA_DATA_COUNTER / 2; ++i)
    {
        int16_t ix = nearest[left ? 180 - i : i];
        if (ix >= 0)
            out[n++] = in[ix];
    }
    return n;
}

uint16_t LEIDA_DATA_HANDLE6(_LEIDA_DATA out[], _LEIDA_DATA in[], u16 size)
{
    return boundary_extract(out, in, size, 1);
}

uint16_t LEIDA_DATA_HANDLE7(_LEIDA_DATA out[], _LEIDA_DATA in[], u16 size)
{
    return boundary_extract(out, in, size, 0);
}

uint16_t LEIDA_DATA_HANDLE8(_LEIDA_DATA arr[], u16 size)
{
    uint16_t i = 0;
    float min = 0;

    if (size < 8 || size > LEIDA_DATA_COUNTER / 2)
        return 0;
    for (i = 3; i + 4 < size; i++)
    {
        uint16_t k;
        uint8_t continuous = 1;
        for (k = i - 2; k < i + 3; ++k)
        {
            float gap = fabsf(arr[k + 1].angle - arr[k].angle);
            if (gap < 0.25f || gap > 2.0f)
                continuous = 0;
        }
        if (!continuous)
            continue;
        if (arr[i].angle >= (180 - 80)) /* 仅左侧 (角度>=100°) */
            if (fabs(arr[i].distance - arr[i + 1].distance) >=
                fmaxf(400.0f, 0.25f * fminf(arr[i].distance, arr[i + 1].distance)))
                if (fabs(arr[i - 1].distance - arr[i + 2].distance) >=
                    fmaxf(400.0f, 0.25f * fminf(arr[i].distance, arr[i + 1].distance)))
                    if (fabs(arr[i - 2].distance - arr[i + 3].distance) >=
                        fmaxf(400.0f, 0.25f * fminf(arr[i].distance, arr[i + 1].distance)))
                    {
                        /* 取突变两侧中距离较近的点 */
                        if (arr[i].distance > arr[i + 2].distance)
                            min = arr[i + 2].distance * arm_sin_f32(arr[i + 2].angle * PI / 180);
                        else
                            min = arr[i - 1].distance * arm_sin_f32(arr[i - 1].angle * PI / 180);

                        /* 20261008: y投影太小的突变是"贴着车身的锥桶/墙头", 不是弯道开口。
                         * 必须在搜索内部跳过它并继续往后找真开口 —— 原来在主循环里事后清零,
                         * 会把整侧断点直接丢掉(复核复现: first=60, 事后清零=0, 后面的 281 被漏)。
                         * duandian_MIN_Y=0 时行为与旧代码完全一致。 */
                        if (min >= duandian_MIN_Y && min < 1000) /* 仅报告1米以内的突变 */
                            return min;
                    }
    }
    return 0;
}

uint16_t LEIDA_DATA_HANDLE9(_LEIDA_DATA arr[], u16 size)
{
    uint16_t i = 0;
    float min = 0;

    if (size < 8 || size > LEIDA_DATA_COUNTER / 2)
        return 0;
    for (i = 3; i + 4 < size; i++)
    {
        uint16_t k;
        uint8_t continuous = 1;
        for (k = i - 2; k < i + 3; ++k)
        {
            float gap = fabsf(arr[k + 1].angle - arr[k].angle);
            if (gap < 0.25f || gap > 2.0f)
                continuous = 0;
        }
        if (!continuous)
            continue;
        if (arr[i].angle <= (80)) /* 仅右侧 (角度<=80°) */
            if (fabs(arr[i].distance - arr[i + 1].distance) >=
                fmaxf(400.0f, 0.25f * fminf(arr[i].distance, arr[i + 1].distance)))
                if (fabs(arr[i - 1].distance - arr[i + 2].distance) >=
                    fmaxf(400.0f, 0.25f * fminf(arr[i].distance, arr[i + 1].distance)))
                    if (fabs(arr[i - 2].distance - arr[i + 3].distance) >=
                        fmaxf(400.0f, 0.25f * fminf(arr[i].distance, arr[i + 1].distance)))
                    {
                        if (arr[i - 1].distance > arr[i + 2].distance)
                            min = arr[i + 2].distance * arm_sin_f32(arr[i + 2].angle * PI / 180);
                        else
                            min = arr[i - 1].distance * arm_sin_f32(arr[i - 1].angle * PI / 180);

                        if (min >= duandian_MIN_Y && min < 1000) /* 同上: 跳过贴车身的突变 */
                            return min;
                    }
    }
    return 0;
}

void LEIDA_Opening(const _LEIDA_DATA *points, uint16_t count, uint16_t *left, uint16_t *right)
{
    static _LEIDA_DATA side[181];
    uint16_t n;
    n = LEIDA_DATA_HANDLE6(side, (_LEIDA_DATA *)points, count);
    *left = LEIDA_DATA_HANDLE8(side, n);
    n = LEIDA_DATA_HANDLE7(side, (_LEIDA_DATA *)points, count);
    *right = LEIDA_DATA_HANDLE9(side, n);
}
