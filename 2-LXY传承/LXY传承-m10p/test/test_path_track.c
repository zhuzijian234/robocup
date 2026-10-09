/* 直接编译生产模块，以真实坐标验证，不能用桩伪造可用中线。 */
#include "../HARDWARE/CENTRE_LINE/path_track.c"
#include <stdio.h>
static unsigned checks;
#define CHECK(x)                                                                                             \
    do                                                                                                       \
    {                                                                                                        \
        ++checks;                                                                                            \
        if (!(x))                                                                                            \
        {                                                                                                    \
            printf("FAIL line %d: %s\n", __LINE__, #x);                                                      \
            return 1;                                                                                        \
        }                                                                                                    \
    } while (0)
static PathPoint points[400];
static PathGeometry geometry;
static PathObservation path;
static PathController control;
static PathCommand command;
static PathGains gains = {.035f, .035f, .040f, .022f, .0395f, .020f, 0, 200, 1170, 1445, 1720};
static unsigned road(float slope, float shift, unsigned sides)
{
    unsigned i, n = 0;
    float half = 250 * sqrtf(1 + slope * slope);
    for (i = 0; i < 60; ++i)
    {
        float y = 110 + i * 19.0f;
        if (sides & 1)
        {
            points[n].x = shift + slope * y - half;
            points[n++].y = y;
        }
        if (sides & 2)
        {
            points[n].x = shift + slope * y + half;
            points[n++].y = y;
        }
    }
    return n;
}
static void build(unsigned n)
{
    Path_Build(&geometry, points, (uint16_t)n, 500, 700, 0, &path);
}
static void step(uint32_t now, uint8_t valid, uint16_t pwm)
{
    Path_Control(&control, &path, &gains, valid, now, pwm, &command);
}
int main(void)
{
    unsigned n, i;
    float width;
    uint16_t turn;
    build(road(0, 0, 3));
    CHECK(path.valid && path.far_valid && path.source == PATH_DUAL && path.straight);
    CHECK(fabsf(path.target_x) < .01f && fabsf(path.width - 500) < .01f);
    CHECK(path.near_fit[0].count >= 4 && path.far_fit[0].count >= 4);
    step(100000, 1, 1445);
    CHECK(command.applied && command.pwm == 1445 && command.d_reset);
    /* 六十个单侧点真实跟线；同一来源可向两边修正。 */
    build(road(0, 60, 1));
    CHECK(path.valid && path.source == PATH_LEFT && path.far_valid);
    CHECK(fabsf(path.target_x - 60) < .01f);
    step(200000, 1, 1445);
    CHECK(command.applied && command.pwm < 1445 && command.d_reset);
    build(road(0, -60, 1));
    step(300000, 1, command.pwm);
    CHECK(command.applied && command.pwm > 1445);
    width = geometry.width;
    build(road(.25f, 0, 2));
    CHECK(path.valid && path.source == PATH_RIGHT && path.far_valid);
    CHECK(fabsf(path.target_x - 175) < .1f && geometry.width == width);
    /* 镜像验证；左墙跨 x=0 后仍属于左墙。 */
    for (i = 0; i <= 12; ++i)
    {
        float slope = -.6f + i * .1f;
        memset(&geometry, 0, sizeof geometry);
        memset(&control, 0, sizeof control);
        build(road(slope, 0, 3));
        CHECK(path.valid && path.far_valid && path.source == PATH_DUAL);
        CHECK(fabsf(path.target_x - 700 * slope) < .2f && fabsf(path.width - 500) < .2f);
        step(100000, 1, 1445);
        CHECK(command.applied);
        CHECK(slope < -.05f ? command.pwm > 1445 : slope > .05f ? command.pwm < 1445 : command.pwm == 1445);
    }
    /* 单簇密集点、只有1.9m外点、容量错误均不能伪造局部参考。 */
    for (i = 0; i < 60; ++i)
    {
        points[i].x = -250;
        points[i].y = 320 + i * .5f;
    }
    build(60);
    CHECK(!path.valid);
    for (i = 0; i < 60; ++i)
        points[i].y = 1900 + i;
    build(60);
    CHECK(!path.valid);
    build(401);
    CHECK(!path.valid);
    build(0);
    CHECK(!path.valid);
    Path_Build(&geometry, 0, 10, 500, 700, 0, &path);
    CHECK(!path.valid);
    points[0].x = NAN;
    points[0].y = 150;
    build(1);
    CHECK(!path.valid);
    n = road(0, 0, 1);
    for (i = 0; i < n; ++i)
        if (points[i].y > 600)
            points[i].y = 3000;
    build(n);
    CHECK(path.valid && !path.far_valid && path.ref_y <= 600 && !path.straight);
    n = road(0, 0, 3);
    points[n].x = 1800;
    points[n++].y = 700;
    build(n);
    CHECK(path.valid && fabsf(path.target_x) < .1f);
    /* 六个稀疏锥桶观测有真实跨度；与同一簇的六十个密集点严格区分。 */
    for (i = 0; i < 6; ++i)
    {
        points[i].x = -250;
        points[i].y = 150 + 200.0f * i;
    }
    build(6);
    CHECK(path.valid && path.far_valid && path.source == PATH_LEFT);
    CHECK(path.near_fit[0].gap <= MAX_GAP_MM && path.near_fit[0].count == 4);
    /* 弯中有路径就持续调整，断点不是保持转弯的必要条件。 */
    memset(&control, 0, sizeof control);
    build(road(.4f, 0, 3));
    step(100000, 1, 1445);
    CHECK(command.applied && command.bend == -1 && command.pwm < 1445);
    turn = command.pwm;
    build(road(.5f, 0, 3));
    step(200000, 1, turn);
    CHECK(command.applied && command.pwm < turn);
    turn = command.pwm;
    /* 小误差但远段弯曲不能回中；350ms 到期不能批准弱反向。 */
    path.target_x = 0;
    path.straight = 0;
    step(700000, 1, turn);
    CHECK(command.computed && !command.applied && command.pwm == turn);
    path.target_x = -200;
    path.far_valid = 0;
    path.near_a = 0;
    step(1200000, 1, turn);
    CHECK(!command.applied && command.reason == PATH_WAIT_REVERSE && command.pwm == turn);
    build(road(-.5f, 0, 3));
    step(1300000, 1, turn);
    CHECK(command.applied && command.reason == PATH_REVERSE && command.pwm > 1445 && command.d == 0);
    turn = command.pwm;
    build(road(0, 0, 3));
    step(1400000, 1, turn);
    CHECK(!command.applied && command.reason == PATH_WAIT_EXIT);
    step(1450000, 1, turn);
    CHECK(!command.applied);
    step(1480000, 1, turn);
    CHECK(command.applied && command.reason == PATH_EXIT && command.pwm == 1445);
    build(road(.4f, 0, 3));
    step(1600000, 1, 1445);
    turn = command.pwm;
    step(1700000, 0, turn);
    CHECK(!command.computed && !command.applied && command.reason == PATH_BAD_SCAN);
    step(1800000, 1, turn);
    CHECK(command.applied && command.d == 0);
    /* 坏帧打断出弯连续证据；恢复不制造假 D。 */
    build(road(0, 0, 3));
    step(1900000, 1, turn);
    step(2000000, 0, turn);
    step(2100000, 1, turn);
    CHECK(!command.applied);
    step(2180000, 1, turn);
    CHECK(command.applied);
    memset(&control, 0, sizeof control);
    step(0xffff0000u, 1, 1445);
    step(0x3880u, 1, 1445);
    CHECK(command.applied && !command.d_reset);
    /* 真实直道但位置偏移时仍能结束旧弯向并居中，不能锁住出弯纠偏。 */
    build(road(.4f, 0, 3));
    step(100000, 1, 1445);
    turn = command.pwm;
    build(road(0, -150, 3));
    CHECK(path.straight);
    step(200000, 1, turn);
    CHECK(!command.applied);
    step(300000, 1, turn);
    CHECK(command.applied && command.pwm > 1445 && command.bend == 0);
    /* 无数据经过很久：下一帧不能复用上一帧的出弯证据。 */
    build(road(.4f, 0, 3));
    step(400000, 1, 1445);
    turn = command.pwm;
    build(road(0, 0, 3));
    step(500000, 1, turn);
    step(1500000, 1, turn);
    CHECK(!command.applied);
    step(1600000, 1, turn);
    CHECK(command.applied);
    printf("PASS %u production path geometry / steering checks\n", checks);
    return 0;
}
