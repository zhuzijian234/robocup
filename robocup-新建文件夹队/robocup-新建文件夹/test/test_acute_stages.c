/* 合成走廊检验附加阶段及其控制边界，不冒充车辆闭环动力学仿真。 */
#include "../HARDWARE/CENTRE_LINE/path_track.c"
#include <stdio.h>
#include <stdlib.h>
static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { printf("FAIL %d: %s\n", __LINE__, #x); return 1; } } while (0)

/* 500mm通道、中心半径450mm的独立90度弯；数组沿两条连续墙排列。 */
static unsigned corner(PathPoint *p, float start, int direction, float shift)
{
    PathPoint wall[200];
    unsigned n = 0, m, i;
    int side;
    for (side = 1; side >= -1; side -= 2)
    {
        m = 0;
        for (i = 0; 110 + 10 * i < start; ++i)
        {
            wall[m].x = side * 250.0f;
            wall[m++].y = 110 + 10.0f * i;
        }
        for (i = 0; i <= 78; ++i)
        {
            float angle = .02f * i, radius = 450 - side * 250.0f;
            PathPoint v = {450 - radius * cosf(angle), start + radius * sinf(angle)};
            if (v.y < 1300 && m < 200) wall[m++] = v;
        }
        for (i = 0; i < m; ++i)
        {
            PathPoint v = wall[side == 1 ? i : m - 1 - i];
            v.x = direction * v.x + shift;
            p[n++] = v;
        }
    }
    if (direction < 0)
        for (i = 0; i < n / 2; ++i)
        { PathPoint v = p[i]; p[i] = p[n - 1 - i]; p[n - 1 - i] = v; }
    return n;
}

static unsigned gentle(PathPoint *p, float slope, float curve)
{
    unsigned i, n = 0;
    int side;
    for (side = 1; side >= -1; side -= 2)
        for (i = 0; i < 118; ++i)
        {
            float y = 110 + 10.0f * (side > 0 ? i : 117 - i);
            p[n].x = side * 250 + slope * y + curve * y * y;
            p[n++].y = y;
        }
    return n;
}

/* 专门覆盖“已准备”状态被后续道路推翻的序列；单帧冷启动测试看不到这些问题。 */
static int interruption_checks(void)
{
    PathPoint points[400];
    PathGeometry g;
    PathObservation p;
    PathController c;
    PathCommand o;
    PathGains gains = {.035f,.035f,.04f,.022f,.0395f,.02f,10,200,1170,1445,1720,17};
    unsigned n, i, variant;
    int direction;
    for (direction=-1;direction<=1;direction+=2)
    {
        for (variant=0;variant<3;++variant)
        {
            memset(&g,0,sizeof g);memset(&p,0,sizeof p);
            g.acute_phase=ACUTE_PREPARE;g.acute_direction=(int8_t)direction;g.acute_frames=2;
            p.valid=1;p.target_x=-direction*300.0f;p.ref_y=700;
            p.near_a=-direction*.6f;p.heading_rad=atanf(p.near_a);
            p.far_valid=1;p.far_a=-direction*.8f;p.preview_heading_rad=atanf(p.far_a);
            if(variant==0)p.model=2; /* 整条新路径已反向，不会出现near*far<0的S预告。 */
            if(variant==2){p.near_a=0;p.heading_rad=0;} /* 近段直，远段已反向。 */
            acute_prepare(&g,0,0,10,&p);
            CHECK(g.acute_phase==ACUTE_OFF && p.acute_phase==ACUTE_OFF);
            CHECK(p.acute_reason==ACUTE_PRIORITY && p.outer_offset==0);
        }
        /* 新入口已确认，但控制器仍记着上一弯，且接管帧远段暂时不可见。 */
        memset(&p,0,sizeof p);memset(&c,0,sizeof c);
        c.bend=(int8_t)direction;
        p.valid=1;p.source=PATH_LEFT;p.acute_phase=ACUTE_PREPARE;
        p.acute_direction=(int8_t)direction;p.road_target_x=direction*200.0f;
        p.target_x=10-direction*40.0f;p.ref_y=450;p.near_a=0;
        Path_Control(&c,&p,&gains,1,100000,1445,&o);
        CHECK(o.applied && c.bend==direction);
        p.acute_phase=ACUTE_TURN;p.near_a=direction*.6f;p.target_x=direction*350.0f;p.ref_y=700;
        p.near_fit[0].valid=1;p.near_fit[0].count=5;p.near_fit[0].rms=5;
        Path_Control(&c,&p,&gains,1,180000,o.pwm,&o);
        CHECK(o.applied && c.bend==-direction && (o.pwm-1445.0f)*direction<0);
        CHECK(o.reason==PATH_REVERSE && o.d_reset && !o.transition_limited);
        /* 离开准备后再出现的普通反向仍需要原规则，不能把特权保留下来。 */
        p.acute_phase=ACUTE_OFF;p.near_a=-direction*.6f;p.target_x=-direction*350.0f;
        Path_Control(&c,&p,&gains,1,260000,o.pwm,&o);
        CHECK(o.gate_reason==PATH_WAIT_REVERSE && c.bend==-direction);
        for(variant=0;variant<2;++variant)
        {
            n=corner(points,650,direction,0);
            /* 保持各墙弯曲形状，只改变近处通道宽度至360/680mm。 */
            for(i=0;i<n;++i)
                points[i].x+=(points[i].x*direction>0 ? direction : -direction)*(variant?90.0f:-70.0f);
            memset(&g,0,sizeof g);memset(&p,0,sizeof p);
            g.acute_phase=ACUTE_PREPARE;g.acute_direction=(int8_t)direction;g.acute_frames=2;
            p.valid=1;p.target_x=direction*20.0f;p.ref_y=700;
            acute_prepare(&g,points,(uint16_t)n,10,&p);
            CHECK(g.acute_phase==ACUTE_OFF && p.acute_phase==ACUTE_OFF);
            CHECK(p.acute_reason==ACUTE_NO_ROOM || p.acute_reason==ACUTE_LOST);
        }
    }
    return 0;
}

/* 只在新入口识别中接续真实断墙；普通直线/轮廓聚类仍保持原间隔。 */
static int gap_checks(void)
{
    PathPoint original[400], points[400];
    unsigned n, m, i, pass;
    int direction;
    PathGeometry g;
    PathObservation p;
    AcuteWall wall;
    for (direction=-1; direction<=1; direction+=2)
    {
        n=corner(original,650,direction,0);m=0;
        for(i=0;i<n;++i)
            if(!(original[i].x*direction<0 && original[i].y>800 && original[i].y<935))
                points[m++]=original[i];
        wall=acute_wall(points,(uint16_t)m,-direction);
        CHECK(wall.linked && wall.valid==2 && wall.distance>700);
        memset(&g,0,sizeof g);
        for(pass=0;pass<2;++pass)Path_Build(&g,points,(uint16_t)m,500,900,10,&p);
        CHECK(p.valid && p.acute_phase==ACUTE_PREPARE);
        CHECK(p.acute_checks & ACUTE_CHECK_LINKED);
        /* 将出口整体移远，超过有约束的端点距离，不能把未知空洞接成弯墙。 */
        for(i=0;i<m;++i)if(points[i].x*direction<0 && points[i].y>935)points[i].y+=250;
        wall=acute_wall(points,(uint16_t)m,-direction);
        CHECK(!wall.linked && wall.distance==0);
    }
    /* 受控候选单元测试：两个几何均可接续的远簇，必须拒绝猜测。 */
    {
        uint16_t from=0,to=0;
        memset(&wall,0,sizeof wall);wall.b=-250;wall.end=70;
        for(i=0;i<70;++i){points[i].x=-250;points[i].y=100+10.0f*i;}
        for(i=0;i<40;++i){points[70+i].x=-230+.7f*i*10;points[70+i].y=920+10.0f*i;}
        CHECK(acute_extension(points,110,-1,&wall,&from,&to));
        CHECK(from==70 && to==110);
        for(i=0;i<40;++i){points[110+i].x=-220+.8f*i*10;points[110+i].y=920+10.0f*i;}
        CHECK(!acute_extension(points,150,-1,&wall,&from,&to));
        /* 短小簇不能成为出口；远簇向反方向弯也不能续接。 */
        CHECK(!acute_extension(points,78,-1,&wall,&from,&to));
        for(i=0;i<40;++i)points[70+i].x=-230-.7f*i*10;
        CHECK(!acute_extension(points,110,-1,&wall,&from,&to));
    }
    return 0;
}

int main(void)
{
    PathPoint points[400];
    PathGeometry g = {0};
    PathObservation p;
    PathController c = {0};
    PathCommand o;
    PathGains gains = {.035f,.035f,.04f,.022f,.0395f,.02f,10,200,1170,1445,1720,17};
    unsigned n, i;
    int side;
    for (side = -1; side <= 1; side += 2)
    {
        memset(&g,0,sizeof g); memset(&c,0,sizeof c);
        n = corner(points,650,side,0);
        CHECK(n <= 400);
        for (i = 0; i < 2; ++i)
        {
            Path_Build(&g,points,(uint16_t)n,500,900,10,&p);
            printf("side=%d step=%u valid=%u phase=%u why=%u feature=%.1f goal=%.1f target=%.1f ref=%.1f\n",
                   side,i,p.valid,p.acute_phase,p.acute_reason,p.acute_distance,p.acute_goal,p.target_x,p.ref_y);
        }
        CHECK(p.valid && p.acute_phase == ACUTE_PREPARE && p.acute_direction == side);
        CHECK(p.acute_goal > 0 && p.acute_goal <= 50 && p.outer_offset * side < 0);
        CHECK(fabsf(p.acute_position) < 1 && p.acute_distance > ACUTE_TAKEOVER_MM);
        Path_Control(&c,&p,&gains,1,100000,1445,&o);
        CHECK(o.applied && o.reason == PATH_APPLY && c.bend == 0 && o.d == 0);
        CHECK(abs((int)o.pwm-1445) <= 45);
        CHECK((o.pwm-1445.0f) * side > 0); /* 右弯先向左，左弯先向右。 */
        /* 到达外侧50mm后不再累计偏移，局部位置读数不拿远目标误差充数。 */
        n=corner(points,650,side,side*50.0f);
        Path_Build(&g,points,(uint16_t)n,500,900,10,&p);
        CHECK(p.acute_phase==ACUTE_PREPARE && fabsf(p.acute_position-50)<1);
        CHECK(p.acute_goal<=50 && fabsf(p.target_x-10)<15);
        n=corner(points,650,side,0);
        /* 缺帧与证据中断必须重新确认，不能沿用上一次的单帧。 */
        Path_Build(&g,points,0,500,900,10,&p);
        CHECK(!p.valid && g.acute_phase == ACUTE_OFF);
        Path_Build(&g,points,(uint16_t)n,500,900,10,&p);
        CHECK(p.acute_phase == ACUTE_CONFIRM);
        Path_Build(&g,points,(uint16_t)n,500,900,10,&p);
        CHECK(p.acute_phase == ACUTE_PREPARE);
        /* 已到主弯时即时接管，不能为了凑足偏移仍朝外走；也不得退回准备。 */
        n=corner(points,350,side,0);
        Path_Build(&g,points,(uint16_t)n,500,900,10,&p);
        CHECK(p.acute_phase != ACUTE_PREPARE && p.outer_offset == 0);
        n=corner(points,650,side,0);
        CHECK(g.acute_phase == ACUTE_TURN);
        Path_Build(&g,points,(uint16_t)n,500,900,10,&p);
        CHECK(p.acute_phase == ACUTE_TURN && p.outer_offset == 0);
        n=gentle(points,0,0);
        Path_Build(&g,points,(uint16_t)n,500,900,10,&p);
        CHECK(p.acute_phase==ACUTE_EXIT && p.outer_offset==0);
        Path_Build(&g,points,(uint16_t)n,500,900,10,&p);
        CHECK(p.acute_phase==ACUTE_OFF && g.acute_direction==0);
        /* S弯/锥桶拥有当前通路，不叠加入口偏移。 */
        p.s_preview=1; acute_entry(&g,points,(uint16_t)n,10,&p);
        CHECK(g.acute_phase == ACUTE_OFF && p.outer_offset == 0);
        p.s_preview=0;p.avoid_state=AVOID_ACTIVE;
        acute_entry(&g,points,(uint16_t)n,10,&p);
        CHECK(g.acute_phase == ACUTE_OFF && p.outer_offset == 0);
    }
    /* 直墙斜放、长缓弧、短/窄/断裂的外墙不能靠连续多帧误激活。 */
    for (side=0;side<3;++side)
    {
        memset(&g,0,sizeof g);
        n=gentle(points,side==1?.15f:0,side==2?.00015f:0);
        for (i=0;i<12;++i)
        {
            Path_Build(&g,points,(uint16_t)n,500,900,10,&p);
            CHECK(p.acute_phase!=ACUTE_PREPARE && p.outer_offset==0);
        }
    }
    memset(&g,0,sizeof g);n=corner(points,650,1,0);
    for (i=0;i<n;++i)points[i].x*=.65f;
    for (i=0;i<4;++i)
    {
        Path_Build(&g,points,(uint16_t)n,500,900,10,&p);
        CHECK(p.acute_phase!=ACUTE_PREPARE);
    }
    /* 远处先转再回摆、但整体斜率仍同号的S形，不能只凭far正斜率布位。 */
    memset(&g,0,sizeof g);n=gentle(points,0,0);
    for(i=0;i<n;++i)
    {
        float y=points[i].y,t;
        if(y>1000){t=y-1000;points[i].x+=162+1.08f*t-.0025f*t*t;}
        else if(y>700){t=y-700;points[i].x+=.0018f*t*t;}
    }
    CHECK(acute_wall(points,(uint16_t)n,-1).distance==0);
    for(i=0;i<8;++i)
    {
        Path_Build(&g,points,(uint16_t)n,500,900,10,&p);
        CHECK(p.acute_phase!=ACUTE_PREPARE);
    }
    /* 拟合字段来自160711切换附近；TURN阶段是构造的前置条件，不声称
     * 当时固件已经有此阶段。近段0.503->0.579，验证确认急弯内补充规则。 */
    memset(&p,0,sizeof p); memset(&c,0,sizeof c);
    p.valid=1;p.source=PATH_LEFT;p.near_a=.503f;p.far_a=1.96f;p.far_valid=1;
    p.target_x=550.118f;p.ref_y=720;p.acute_phase=ACUTE_TURN;
    Path_Control(&c,&p,&gains,1,100000,1445,&o);
    CHECK(o.applied && o.pwm < 1200);
    p.near_a=.579f;p.far_valid=0;p.target_x=409.621f;p.ref_y=700;p.target_transition=1;
    Path_Control(&c,&p,&gains,1,180000,o.pwm,&o);
    CHECK(o.transition_limited && o.error == -460 && o.pwm < 1220 && o.d == 0);
    /* 即使每帧都报目标跳变，也仅保两帧；真实新弯不受此约束。 */
    p.target_x=110;
    Path_Control(&c,&p,&gains,1,260000,o.pwm,&o);
    CHECK(o.transition_limited && o.error == -420);
    Path_Control(&c,&p,&gains,1,340000,o.pwm,&o);
    CHECK(!o.transition_limited && o.error == -100);
    p.s_preview=1;p.target_x=-200;p.near_a=-.3f;p.far_a=-.8f;p.far_valid=1;
    Path_Control(&c,&p,&gains,1,420000,o.pwm,&o);
    CHECK(o.applied && o.reason == PATH_REVERSE && o.pwm > 1445 && !o.transition_limited);
    Path_ClearHistory(&g);
    CHECK(!g.acute_phase && !g.acute_frames && !g.baseline_offset);
    CHECK(interruption_checks()==0);
    CHECK(gap_checks()==0);
    printf("PASS %u acute preparation/priority/handoff/bounded-transition checks\n", checks);
    return 0;
}
