/* S阶段仅约束外移；明确反向接管、独立急弯和断流重置均保持可用。 */
#include "../HARDWARE/CENTRE_LINE/path_track.c"
#include <stdio.h>
#define CHECK(x) do {if(!(x)){printf("FAIL line %d: %s\n",__LINE__,#x);return 1;}} while(0)

static void entry(PathObservation *p)
{
    memset(p,0,sizeof *p);
    p->valid=p->far_valid=1;
    p->heading_rad=.2f;p->preview_heading_rad=.8f;
    p->target_x=200;p->ref_y=700;
    p->far_fit[0].valid=1;p->far_fit[0].a=1;p->far_fit[0].b=-850;
    p->far_fit[0].min_y=550;p->far_fit[0].max_y=1000;
}

int main(void)
{
    PathGeometry g={0};PathObservation p;
    PathController c={0};PathCommand o;
    PathGains gains={.035f,.035f,.04f,.022f,.0395f,.020f,10,200,1170,1445,1720,17};
    unsigned i;
    entry(&p);p.s_preview=1;acute_entry(&g,0,0,&p);
    CHECK(g.s_transition_frames==6 && !g.outer_offset);
    for(i=0;i<5;++i){entry(&p);acute_entry(&g,0,0,&p);CHECK(!p.outer_offset);}
    CHECK(g.s_transition_frames==1);
    for(i=0;i<4;++i){entry(&p);acute_entry(&g,0,0,&p);}
    CHECK(!g.s_transition_frames && p.outer_offset==-25); /* 独立急弯不会永久禁用。 */
    p.s_preview=1;acute_entry(&g,0,0,&p);
    for(i=0;i<2;++i){entry(&p);p.straight=1;p.heading_rad=p.preview_heading_rad=0;acute_entry(&g,0,0,&p);}
    CHECK(!g.s_transition_frames && !g.outer_offset);
    entry(&p);p.s_preview=1;acute_entry(&g,0,0,&p);
    Path_ClearHistory(&g);CHECK(!g.s_transition_frames && !g.s_straight_frames);
    /* S阶段状态不进入Path_Control：可靠反向几何仍当帧跨过中位。 */
    entry(&p);p.s_preview=1;acute_entry(&g,0,0,&p);
    p.s_preview=0;p.source=PATH_LEFT;p.near_a=.7f;p.far_a=.9f;p.target_x=300;
    Path_Control(&c,&p,&gains,1,100000,1445,&o);CHECK(o.pwm<1445);
    p.near_a=-.7f;p.far_a=-.9f;p.target_x=-300;
    Path_Control(&c,&p,&gains,1,180000,o.pwm,&o);
    CHECK(o.applied && o.reason==PATH_REVERSE && o.pwm>1445 && g.s_transition_frames);
    puts("PASS S phase expiry/straight release/reset and immediate reverse");return 0;
}
