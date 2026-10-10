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

/* 以真实0.5度射线取最近回波，圆锥会遮挡其后的墙；顺序与生产点云一致。 */
static unsigned scene(float width, float slope, const PathPoint *cones, unsigned cone_count)
{
    unsigned i, j, n = 0;
    for (i = 1; i < 360; ++i)
    {
        float angle = i * 0.5f * 3.14159265358979323846f / 180;
        float x = cosf(angle), y = sinf(angle);
        float denominator = x - slope * y;
        float r = fabsf(denominator) > .0001f ? width * .5f * sqrtf(1 + slope * slope) /
                                                     fabsf(denominator) : 1e6f;
        for (j = 0; j < cone_count; ++j)
        {
            float projection = cones[j].x * x + cones[j].y * y;
            float discriminant = projection * projection - cones[j].x * cones[j].x -
                                  cones[j].y * cones[j].y + 52 * 52;
            if (discriminant >= 0)
            {
                float hit = projection - sqrtf(discriminant);
                if (hit > 0 && hit < r)
                    r = hit;
            }
        }
        if (r >= 100 && r <= 2600 && r * y >= 100 && r * y < 1300)
        {
            points[n].x = r * x;
            points[n++].y = r * y;
        }
    }
    return n;
}
static unsigned cone_road(float width, float cone_x, float cone_y)
{
    PathPoint cone = {cone_x, cone_y};
    return scene(width, 0, &cone, 1);
}
static void build(unsigned n)
{
    Path_Build(&geometry, points, (uint16_t)n, 500, 700, 0, &path);
}
static void step(uint32_t now, uint8_t valid, uint16_t pwm)
{
    Path_Control(&control, &path, &gains, valid, now, pwm, &command);
}
static void confirmed_build(unsigned n)
{
    build(n);
    build(n); /* 两帧空间相容的圆弧，而不是单帧放宽识别门槛。 */
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

    /* 实测500mm误差平台：稳态应能利用更多轮角，直道小误差仍保持原增益。 */
    memset(&control, 0, sizeof control);
    build(road(.6f, 0, 2));
    path.target_x = 500;
    step(100000, 1, 1445);
    step(200000, 1, command.pwm);
    CHECK(command.applied && command.d == 0 && command.pwm >= 1170 && command.pwm < 1200);
    CHECK(command.kp > gains.side_kp);
    memset(&control, 0, sizeof control);
    build(road(0, 60, 3));
    step(100000, 1, 1445);
    CHECK(command.kp == gains.kp && command.pwm == 1424);

    /* 一侧远段遮挡：另一侧位置连续时保留700mm预瞄，而不是缩至近段末尾。 */
    n = road(.15f, 0, 3);
    for (i = 0; i < n; i += 2)
        if (points[i].y > 720)
            points[i].y = 3000;
    build(n);
    CHECK(path.valid && path.far_valid && path.source == PATH_RIGHT && path.ref_y == 700);
    CHECK(fabsf(path.target_x - 105) < .2f);

    /* 远段已经进入反向小弯、近段接近中性：当帧换向；近段仍强烈相反则等待。 */
    memset(&control, 0, sizeof control);
    build(road(.4f, 0, 3));
    step(100000, 1, 1445);
    turn = command.pwm;
    build(road(-.18f, 0, 3));
    path.near_a = .04f;
    step(200000, 1, turn);
    CHECK(command.applied && command.reason == PATH_REVERSE && command.pwm > 1445 && command.d == 0);
    build(road(.18f, 0, 3));
    path.near_a = -.3f;
    step(300000, 1, command.pwm);
    CHECK(!command.applied && command.reason == PATH_WAIT_REVERSE);
    /* 小于原0.18门槛的近段小S弯，连续两帧可换向，不会永远卡在旧弯。 */
    build(road(.11f, 0, 3));
    path.far_valid = 0;
    step(400000, 1, 1520);
    CHECK(!command.applied);
    step(480000, 1, 1520);
    CHECK(command.applied && command.reason == PATH_REVERSE && command.pwm < 1445);

    /* 方底座外接圆与对墙间约500mm：扫描截面只有104mm，左右镜像均应主动让开。 */
    memset(&geometry, 0, sizeof geometry);
    memset(&control, 0, sizeof control);
    build(cone_road(800, -242, 650));
    CHECK(path.avoid_state == AVOID_CANDIDATE && path.avoid_offset == 0);
    build(cone_road(800, -242, 650));
    CHECK(path.valid && path.avoid_y > 600 && path.avoid_y < 700);
    CHECK(path.avoid_offset > 50 && path.avoid_offset < 110 && path.target_x > 0);
    CHECK(path.avoid_offset < path.width * .5f - VEHICLE_HALF_CLEAR_MM);
    step(100000, 1, 1445);
    CHECK(command.applied && command.pwm < 1445);
    memset(&geometry, 0, sizeof geometry);
    confirmed_build(cone_road(800, 242, 650));
    CHECK(path.valid && path.avoid_offset < -50 && path.target_x < 0);
    step(200000, 1, command.pwm);
    CHECK(command.applied && command.pwm > 1445 && command.d_reset);
    /* 旧弯向不能锁住避让；路况状态保持道路含义，坏扫描仍拒绝写舵机。 */
    control.bend = -1;
    step(300000, 1, 1250);
    CHECK(command.applied && command.pwm > 1445);
    step(400000, 0, command.pwm);
    CHECK(!command.applied && command.reason == PATH_BAD_SCAN);
    width = path.avoid_offset;
    build(cone_road(800, 5000, 5000));
    CHECK(path.avoid_offset == width && path.avoid_y == 0);
    build(cone_road(800, 5000, 5000));
    build(cone_road(800, 5000, 5000));
    CHECK(path.avoid_offset == width);
    build(cone_road(800, 5000, 5000));
    CHECK(path.avoid_offset > width && path.avoid_offset < 0);
    for (i = 0; i < 20; ++i)
        build(cone_road(800, 5000, 5000));
    CHECK(path.avoid_offset == 0 && fabsf(path.target_x) < .1f);
    /* 50cm普通直道、孤立噪点、平直短片，均不能伪装圆锥触发避让。 */
    memset(&geometry, 0, sizeof geometry);
    build(cone_road(500, 5000, 5000));
    CHECK(path.valid && path.avoid_offset == 0 && path.avoid_y == 0);
    /* 同一锥桶逐步接近；斜道路上的左右镜像，不把圆弧附近的遮挡当转向反号。 */
    for (i = 0; i < 7; ++i)
    {
        float y = 350 + i * 100.0f;
        PathPoint cone = {.12f * y - 242 * sqrtf(1 + .12f * .12f), y};
        memset(&geometry, 0, sizeof geometry);
        confirmed_build(scene(800, .12f, &cone, 1));
        CHECK(path.valid && path.avoid_offset > 5 && path.avoid_y > 0);
        cone.x = -.12f * y + 242 * sqrtf(1 + .12f * .12f);
        memset(&geometry, 0, sizeof geometry);
        confirmed_build(scene(800, -.12f, &cone, 1));
        CHECK(path.valid && path.avoid_offset < -5 && path.avoid_y > 0);
    }
    {
        PathPoint cones[2] = {{-242, 600}, {242, 1000}};
        memset(&geometry, 0, sizeof geometry);
        confirmed_build(scene(800, 0, cones, 2));
        CHECK(path.valid && path.avoid_offset > 0 && path.avoid_y < 650);
        cones[0].x = -5000;
        confirmed_build(scene(800, 0, cones, 2));
        CHECK(path.valid && path.avoid_offset < 0 && path.avoid_y > 950);
    }
    /* 标定偏移也计入底座/对侧预算；普通斜墙和墙外物体不触发避让。 */
    memset(&geometry, 0, sizeof geometry);
    n = cone_road(800, -242, 650);
    Path_Build(&geometry, points, (uint16_t)n, 500, 700, -27, &path);
    Path_Build(&geometry, points, (uint16_t)n, 500, 700, -27, &path);
    CHECK(path.valid && path.avoid_offset > 95 && path.avoid_offset < 120);
    memset(&geometry, 0, sizeof geometry);
    build(scene(500, .25f, 0, 0));
    CHECK(path.valid && path.avoid_offset == 0);
    build(cone_road(800, 600, 700));
    CHECK(path.valid && path.avoid_offset == 0);
    for (i = 0; i < 10; ++i)
    {
        points[i].x = -200 + i * 8;
        points[i].y = 650;
    }
    {
        PathPoint center;
        CHECK(!cone_center(points, 0, 10, &center));
        CHECK(!cone_center(points, 0, 1, &center));
    }
    /* 复现第三次seq7454的危险组合：道路误差15.9，释放残余15.2。
     * 旧版把候选中位1445写入，随后WAIT_EXIT一直保持中位。 */
    memset(&control, 0, sizeof control);
    memset(&path, 0, sizeof path);
    control.bend = 1;
    path.valid = 1; path.source = PATH_DUAL; path.near_a = -.18f;
    path.target_x = -.7f; path.avoid_offset = 15.2f;
    path.avoid_state = AVOID_RELEASE; path.ref_y = 700;
    step(100000, 1, 1600);
    CHECK(!command.applied && command.pwm == 1600 && command.reason == PATH_WAIT_EXIT);
    CHECK(fabsf(command.road_error-15.9f)<.01f && !command.avoid_override);
    path.avoid_state = AVOID_HOLD;
    step(180000, 1, 1600);
    CHECK(!command.applied && command.pwm == 1600);
    path.avoid_state = AVOID_ACTIVE;
    step(260000, 1, 1600);
    CHECK(command.applied && command.avoid_override && command.gate_reason == PATH_WAIT_EXIT);
    /* 弯道宽度不学习；诊断需区别无点/跨度不足/拟合成功。 */
    memset(&geometry, 0, sizeof geometry);
    build(road(.3f, 0, 3));
    CHECK(path.valid && path.width_frozen && path.width_measured && geometry.width == 500);
    build(0);
    CHECK(path.geometry_reason == GEOM_NEAR_MISSING &&
          (path.near_fit[0].rejected & (FIT_POINTS|FIT_SPAN)) == (FIT_POINTS|FIT_SPAN));
    /* 原固定400mm不在支持区；约425~740mm的可见边界仍可提供近段参考。 */
    n=road(0,0,1);
    for(i=0;i<n;++i)
        if(points[i].y<405 || (points[i].y>448 && points[i].y<500) || points[i].y>749)
            points[i].y=3000;
    build(n);
    CHECK(path.valid && path.near_ref_y > 400 && path.near_ref_y <= 450);
    CHECK(path.near_ref_y >= path.near_fit[0].min_y && path.ref_y <= path.near_fit[0].max_y);
    printf("PASS %u production path geometry / steering checks\n", checks);
    return 0;
}
