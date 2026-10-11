/* 新功能的行为边界：保持普通跟踪、S预告不反打、变速连续、未知通路不放行。 */
#include "../HARDWARE/CENTRE_LINE/path_track.c"
#include <stdio.h>
static unsigned checks;
#define CHECK(x)                                                                                             \
    do                                                                                                       \
    {                                                                                                        \
        ++checks;                                                                                            \
        if (!(x))                                                                                            \
        {                                                                                                    \
            printf("FAIL %d: %s\n", __LINE__, #x);                                                           \
            return 1;                                                                                        \
        }                                                                                                    \
    } while (0)

int main(void)
{
    PathObservation p;
    PathSpeed speed;
    PathController a, b;
    PathCommand ca, cb;
    PathGains g = {.035f, .035f, .04f, .022f, .0395f, .020f, -27, 200, 1170, 1445, 1720, 12};
    PathPoint wall[3] = {{250, 100}, {250, 400}, {250, 700}}, x = {0, 100}, y = {0, 700};
    float v, prior;
    unsigned i;
    memset(&p, 0, sizeof p);
    memset(&speed, 0, sizeof speed);
    p.valid = p.far_valid = p.straight = 1;
    p.ref_y = 700;
    CHECK(Path_SpeedLimit(&speed, &p, 1, 17, 10, 100000) == 12);
    CHECK(Path_SpeedLimit(&speed, &p, 1, 17, 10, 1099999) == 12);
    CHECK(Path_SpeedLimit(&speed, &p, 1, 17, 10, 1100000) == 17);
    p.straight = 0;
    p.s_preview = 1;
    p.near_a = -.3f;
    p.far_a = .5f;
    CHECK(Path_SpeedLimit(&speed, &p, 1, 17, 10, 1200000) == 10);
    p.s_preview = 0;
    p.near_a = .4f;
    CHECK(Path_SpeedLimit(&speed, &p, 1, 17, 10, 1300000) == 10);
    p.straight = 1;
    p.near_a = p.far_a = 0;
    CHECK(Path_SpeedLimit(&speed, &p, 1, 17, 10, 1400000) == 12);
    CHECK(Path_SpeedLimit(&speed, &p, 0, 17, 10, 1500000) == 10);
    CHECK(Path_SpeedLimit(&speed, &p, 0, 8, 10, 1600000) == 8);
    memset(&speed, 0, sizeof speed);
    Path_SpeedLimit(&speed, &p, 1, 17, 10, 0xffff0000u);
    CHECK(Path_SpeedLimit(&speed, &p, 1, 17, 10, 0xffff0000u + 1000000u) == 17);
    v = 10;
    for (i = 0; i < 400; ++i)
    {
        prior = v;
        v = Path_SpeedStep(v, 17, 17, 10, 1, 2, 12, .01f);
        CHECK(v >= prior && v - prior < .0201f && v <= 17);
    }
    CHECK(v == 17);
    for (i = 0; i < 100; ++i)
    {
        prior = v;
        v = Path_SpeedStep(v, 17, 10, 10, 0, 2, 12, .01f);
        CHECK(v <= prior && prior - v < .1201f && v >= 10);
    }
    CHECK(v == 10);
    CHECK(Path_SpeedStep(10, 0, 17, 10, 1, 2, 12, .01f) == 0);
    CHECK(Path_SpeedStep(0, 12, 12, 10, 1, 2, 12, .01f) == 10);
    CHECK(Path_Preview(700, 10) == 700 && Path_Preview(700, 12) == 700);
    CHECK(Path_Preview(700, 17) == 900 && Path_Preview(1100, 17) == 1100);
    CHECK(corridor_clear(wall, 3, x, y));
    wall[1].x = 120;
    CHECK(!corridor_clear(wall, 3, x, y));

    memset(&p, 0, sizeof p);
    p.valid = p.far_valid = 1;
    p.source = PATH_RIGHT;
    p.ref_y = 700;
    p.near_a = -.3f;
    p.far_a = .5f;
    p.target_x = -227;
    p.s_preview = 1;
    memset(&a, 0, sizeof a);
    memset(&b, 0, sizeof b);
    Path_Control(&a, &p, &g, 1, 100000, 1445, &ca);
    g.speed = 17;
    Path_Control(&b, &p, &g, 1, 100000, 1445, &cb);
    CHECK(ca.applied && cb.applied && cb.pwm >= 1445 && cb.pwm < ca.pwm);
    CHECK(cb.error >= .9f * ca.error && cb.error > 0);
    p.s_preview = 0;
    memset(&a, 0, sizeof a);
    memset(&b, 0, sizeof b);
    g.speed = 12;
    Path_Control(&a, &p, &g, 1, 100000, 1445, &ca);
    g.speed = 17;
    Path_Control(&b, &p, &g, 1, 100000, 1445, &cb);
    CHECK(ca.pwm == cb.pwm); /* 高速增益没有污染普通路径。 */
    p.valid = 0;
    Path_Control(&b, &p, &g, 1, 200000, 1550, &cb);
    CHECK(!cb.applied && cb.pwm == 1550); /* 证据不足不得猜测出口。 */
    printf("PASS %u race speed/preview/S-turn/corridor checks\n", checks);
    return 0;
}
