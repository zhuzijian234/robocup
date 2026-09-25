#include <stdio.h>
#include <string.h>
#include <math.h>
#include "m10p.c" /* production implementation, static state inspected for bounds */
static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { printf("FAIL %d: %s\n", __LINE__, #x); return 1; } } while (0)
static uint8_t wire[160];
static uint32_t now;
static void make(unsigned angle, unsigned distance)
{
    unsigned j;
    memset(wire,0,sizeof wire); wire[0]=0xa5;wire[1]=0x5a;wire[3]=160;
    wire[4]=(uint8_t)(angle>>8);wire[5]=(uint8_t)angle;
    wire[6]=0x0d;wire[7]=0x90;wire[158]=0xfa;wire[159]=0xfb;
    for(j=0;j<70;j++){wire[8+j*2]=(uint8_t)(distance>>8);wire[9+j*2]=(uint8_t)distance;}
}
static void feed(void){now+=3472;M10P_Feed(wire,160,now-3472,now);}
static const M10P_Scan *circle(unsigned distance)
{
    unsigned j;
    const M10P_Scan *s=0;
    for(j=0;j<60 && !s;j++) {
        make((17300+j*1500)%36000,distance);feed();s=M10P_Acquire();
    }
    return s;
}
int main(void)
{
    unsigned i,j,split;uint16_t index[720];const M10P_Scan *s;
    CHECK(sizeof(M10P_Point)==8);
    CHECK(M10P_AlgorithmAngle(0)==9000);CHECK(M10P_AlgorithmAngle(9000)==0);
    CHECK(M10P_AlgorithmAngle(18000)==27000);CHECK(M10P_AlgorithmAngle(27000)==18000);
    CHECK(M10P_AlgorithmAngle(36000)==9000);
    for(split=0;split<=160;split++) {
        M10P_Init();make(17300,0x9388);
        M10P_Feed(wire,split,0,3472);M10P_Feed(wire+split,160-split,0,3472);
        CHECK(M10P_stats.packets==1);CHECK(M10P_stats.high_reflect==70);CHECK(pending==0);
    }
    M10P_Init();make(17300,1000);
    for(i=0;i<160;i++)M10P_Feed(wire+i,1,0,3472);
    CHECK(M10P_stats.packets==1);
    M10P_Init();now=0;s=circle(0x9388);CHECK(s!=0);CHECK(s->count==1680);
    CHECK(s->coverage_cdeg==36000);CHECK(s->dps==4320);CHECK(s->front_seen);
    for(i=0;i<s->count;i++){CHECK(s->points[i].range_mm==5000);CHECK(s->points[i].flags==1);CHECK(s->points[i].angle_cdeg<36000);}
    CHECK(M10P_Index(s,index)>650);
    /* Reading ownership survives incoming scans; acquire/release are explicit. */
    { uint32_t seq=s->seq;uint16_t a=s->points[0].angle_cdeg;
      for(j=0;j<96;j++){make((17300+j*1500)%36000,2000);feed();}
      CHECK(s->seq==seq);CHECK(s->points[0].angle_cdeg==a);CHECK(s->points[0].range_mm==5000);
    }
    M10P_Release(s);
    M10P_Init();now=0;s=circle(0xffff);CHECK(s!=0);CHECK(s->count==0);CHECK(!s->front_seen);CHECK(s->coverage_cdeg==36000);M10P_Release(s);
    /* Starting exactly at zero angle is legal; first partial revolution drops. */
    M10P_Init();make(36000,1000);feed();CHECK(M10P_stats.packets==1);CHECK(!M10P_Acquire());
    make(36001,1000);feed();CHECK(M10P_stats.bad_angle==1);
    M10P_Init();make(0,1000);wire[6]=wire[7]=0;feed();CHECK(M10P_stats.bad_speed==1);
    for(i=0;i<3;i++) {
        M10P_Init();make(0,1000);wire[2]=(i==2)?255:0;wire[3]=(i==0)?20:255;feed();
        make(1500,1000);feed();CHECK(M10P_stats.packets==1);CHECK(M10P_stats.bad_length>0);
    }
    M10P_Init();make(0,1000);wire[159]=0;feed();make(1500,1000);feed();CHECK(M10P_stats.bad_tail==1);CHECK(M10P_stats.packets==1);
    M10P_Init();make(0,1000);{uint8_t a=0xa5;M10P_Feed(&a,1,0,1);}feed();CHECK(M10P_stats.packets==1);
    /* Missing and inserted bytes resynchronize at a subsequent intact frame. */
    for(split=1;split<159;split++) {
        M10P_Init();make(0,1000);M10P_Feed(wire,split,0,1);M10P_Feed(wire+split+1,159-split,0,1);
        make(1500,1000);feed();CHECK(M10P_stats.packets==1);
    }
    /* Embedded header/tail are data, not frame boundaries. */
    M10P_Init();make(0,0xa55a);wire[148]=0xfa;wire[149]=0xfb;feed();CHECK(M10P_stats.packets==1);
    /* FFFF compaction: a fresh writing scan, m=69, N advances only for non-FFFF. */
    M10P_Init();now=0;begin_scan(0);make(18000,1000);wire[8]=wire[9]=255;feed();
    CHECK(scans[writer].count==69);CHECK(scans[writer].points[0].angle_cdeg==18000);
    CHECK(scans[writer].points[1].angle_cdeg==18022);
    CHECK(scans[writer].points[68].angle_cdeg==19478);
    /* Range filtering must not compress the protocol angle a second time. */
    M10P_Init();now=0;begin_scan(0);make(18000,1000);wire[8]=0;wire[9]=0;feed();
    CHECK(scans[writer].count==70);CHECK(scans[writer].points[1].angle_cdeg==18021);
    CHECK(M10P_Index(&scans[writer],index)>0);
    M10P_Init();now=0;begin_scan(0);scans[writer].count=2048;make(18000,1000);feed();
    CHECK(scans[writer].count==2048);CHECK(scans[writer].overflow);CHECK(M10P_stats.scan_overflow==1);
    M10P_Lost(9);CHECK(writer<0);CHECK(M10P_stats.epoch==9);CHECK(!M10P_Acquire());
    M10P_Init();make(18000,1000);feed();make(21000,1000);feed();CHECK(M10P_stats.discontinuities==1);
    M10P_Init();make(18000,1000);feed();now+=100000;make(19500,1000);feed();CHECK(M10P_stats.discontinuities==1);
    M10P_Init();now=0xffff0000u;s=circle(1000);CHECK(s!=0);CHECK(s->count==1680);M10P_Release(s);
    /* Actual 512-byte DMA boundaries at the nominal 46,080 byte/s data rate. */
    {
        static uint8_t stream[160*120];
        unsigned off, len, received=0;
        uint32_t prev=0, end;
        M10P_Init();
        for(j=0;j<120;j++){make((17300+j*1500)%36000,1000);memcpy(stream+j*160,wire,160);}
        for(off=0;off<sizeof stream;off+=len){
            len=sizeof stream-off;if(len>512)len=512;
            end=(uint32_t)((double)(off+len)*1000000.0/46080.0);
            M10P_Feed(stream+off,len,prev,end);prev=end;
            s=M10P_Acquire();
            if(s){CHECK(s->count==1680);CHECK(s->period_us>=60000 && s->period_us<=120000);CHECK(s->front_seen);received++;M10P_Release(s);}
        }
        CHECK(received==4);CHECK(M10P_stats.packets==120);CHECK(!M10P_stats.discontinuities);
    }
    /* Deterministic noise stress checks parser storage/progress, not CRC guarantees. */
    /* Reject a complete scan with one unstable-speed packet, even if its final
     * packet reports a normal speed. Next clean scan must recover normally. */
    M10P_Init();now=0;
    for(j=0;j<60;j++){
        make((17300+j*1500)%36000,1000);
        if(j==10){wire[6]=0x20;wire[7]=0;}
        feed();
    }
    CHECK(M10P_stats.rejected==1);s=M10P_Acquire();CHECK(s!=0);CHECK(!s->unstable);M10P_Release(s);
    M10P_Init();{uint32_t seed=12345;uint8_t noise[512];for(j=0;j<2000;j++){
        for(i=0;i<512;i++){seed=seed*1664525u+1013904223u;noise[i]=(uint8_t)(seed>>24);}
        M10P_Feed(noise,512,j*10000,(j+1)*10000);CHECK(pending<160);
    }}
    make(0,1000);feed();CHECK(M10P_stats.packets>=1);
    printf("PASS %u M10P production-C checks\n",checks);return 0;
}
