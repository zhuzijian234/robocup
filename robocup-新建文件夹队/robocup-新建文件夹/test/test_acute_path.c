/* 急弯修复行为边界：同墙排他、身份修复、入口余量、S弯立即接管。 */
#include "../HARDWARE/CENTRE_LINE/path_track.c"
#include <stdio.h>
static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { printf("FAIL %d: %s\n", __LINE__, #x); return 1; } } while (0)

int main(void)
{
    PathGeometry g = {0};
    PathObservation p = {0};
    PathController c = {0};
    PathCommand o;
    PathGains gains = {.035f,.035f,.04f,.022f,.0395f,.02f,10,200,1170,1445,1720,17};
    PathPoint samples[2][STRIPS] = {{{-250,150},{-100,250},{50,350},{200,450}},
                                  {{50,350},{200,450}}};
    PathPoint obstacle = {400,400};
    uint16_t n[2] = {4,2};
    CHECK(separate_boundaries(samples,n)==2 && n[0]==4 && n[1]==0);
    /* 25854/25855的记录拟合值：相同向墙错误从LEFT变成RIGHT。
     * 本例只回放拟合字段，空占用不冒充当时未保存的原始点云。 */
    g.history=1;g.history_a=1.1992996f;g.history_x=511.06485f;g.history_y=400;
    p.width=481.92789f;
    p.near_fit[1].valid=1;p.near_fit[1].a=1.15934265f;p.near_fit[1].b=-276.218506f;
    p.near_fit[1].min_y=249;p.near_fit[1].max_y=750;
    p.far_fit[1].valid=1;p.far_fit[1].a=1.4466797f;p.far_fit[1].b=-469.73877f;
    restore_wall_identity(&g,0,0,&p);
    CHECK(p.identity_corrected && p.near_fit[0].valid && !p.near_fit[1].valid);
    CHECK(center_at(&p.far_fit[0],&p.far_fit[1],PATH_LEFT,p.width,700)>900);
    /* 当前可见占用位于修复后的走廊上，则不能只为了接历史而修正。 */
    p.near_fit[1]=p.near_fit[0];p.far_fit[1]=p.far_fit[0];
    memset(&p.near_fit[0],0,sizeof(PathFit));p.identity_corrected=0;
    obstacle.x=p.near_fit[1].a*400+p.near_fit[1].b+
               p.width*.5f*sqrtf(1+p.near_fit[1].a*p.near_fit[1].a);
    restore_wall_identity(&g,&obstacle,1,&p);
    CHECK(!p.identity_corrected && p.near_fit[1].valid);
    /* 下一弯方向相反时，旧弯身份不得约束新观测。 */
    g.history_a=-g.history_a;
    restore_wall_identity(&g,0,0,&p);
    CHECK(!p.identity_corrected);

    memset(&p,0,sizeof p);
    p.valid=1;p.source=PATH_LEFT;p.near_a=1.2f;p.ref_y=700;p.target_x=950;
    Path_Control(&c,&p,&gains,1,100000,1445,&o);
    CHECK(o.pwm==1178);
    p.target_x=119;p.target_transition=1;p.source=PATH_RIGHT;
    Path_Control(&c,&p,&gains,1,180000,1178,&o);
    CHECK(o.transition_limited && o.error==-420 && o.pwm<1250 && o.d==0);
    p.target_transition=0;
    Path_Control(&c,&p,&gains,1,260000,o.pwm,&o);
    CHECK(o.transition_limited && o.error==-340);
    Path_Control(&c,&p,&gains,1,340000,o.pwm,&o);
    CHECK(!o.transition_limited && fabsf(o.error+109)<.01f); /* 非无期限保舵。 */
    /* 当前近远段一起反向，有明确S弯证据，当帧跨中位。 */
    p.target_x=-300;p.near_a=-.7f;p.far_a=-.9f;p.far_valid=1;p.target_transition=1;
    Path_Control(&c,&p,&gains,1,420000,o.pwm,&o);
    CHECK(o.applied && o.pwm>1445 && o.reason==PATH_REVERSE && !o.transition_limited);
    Path_ClearHistory(&g);
    CHECK(!g.history && !g.acute_frames && g.baseline_offset==0);
    printf("PASS %u acute boundary/identity/clearance/transition checks\n",checks);
    return 0;
}
