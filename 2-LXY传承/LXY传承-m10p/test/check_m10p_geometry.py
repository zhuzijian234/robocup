"""Exercise production nearest-boundary and gap-aware corner extraction."""
import re,json
from check_m10p import P,OUT,run
t=(P/'HARDWARE/LEIDA_DATA/LEIDA_DATA.c').read_text(encoding='utf-8')
def fn(name):
 m=re.search(r'^(?:static )?\w+\s+'+name+r'\([^;]*?\)\s*\{',t,re.M);assert m
 pos=t.index('{',m.start())+1;d=1
 while d:d+=(t[pos]=='{')-(t[pos]=='}');pos+=1
 return t[m.start():pos]
code=r'''
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
typedef uint16_t u16;
typedef struct {float angle,distance;} _LEIDA_DATA;
#define LEIDA_DATA_COUNTER 800
#define PI 3.14159265358979323846f
#define arm_sin_f32 sinf
#define LEIDA_ANGLE_piancha 0.5f
float BLUE_ANGLE_LEFT_RIGHT=90;
/* 20261008: 断点y投影下限。0 = 旧契约(不过滤); HANDLE8/9 在搜索内部跳过 min<下限 的候选。 */
float duandian_MIN_Y=0.0f;
static unsigned checks;
#define CHECK(x) do{++checks;if(!(x)){printf("FAIL %d: %s\n",__LINE__,#x);return 1;}}while(0)
'''
code+='\n'.join(fn(n) for n in ['boundary_extract','LEIDA_DATA_HANDLE6','LEIDA_DATA_HANDLE7','LEIDA_DATA_HANDLE8','LEIDA_DATA_HANDLE9'])
code+=r'''
static _LEIDA_DATA points[800],out[401];
int main(void){unsigned i,n;uint16_t right,left;
 CHECK(LEIDA_DATA_HANDLE6(out,points,0)==0);CHECK(LEIDA_DATA_HANDLE7(out,points,0)==0);
 points[0].angle=359.9f;points[0].distance=1000;points[1].angle=.1f;points[1].distance=500;
 n=LEIDA_DATA_HANDLE7(out,points,2);CHECK(n==1);CHECK(out[0].distance==500);
 points[0].angle=179.9f;points[1].angle=180.1f;n=LEIDA_DATA_HANDLE6(out,points,2);CHECK(n==1);CHECK(out[0].distance==500);
 for(i=0;i<720;i++){points[i].angle=i*.5f;points[i].distance=1000;}
 out[400].distance=12345;n=LEIDA_DATA_HANDLE6(out,points,720);CHECK(n==91);CHECK(out[400].distance==12345);
 CHECK(out[0].angle>=179.5f && out[n-1].angle>=89.5f);
 n=LEIDA_DATA_HANDLE7(out,points,720);CHECK(n==91);CHECK(out[0].angle<.5f && out[n-1].angle>=89.5f);
 BLUE_ANGLE_LEFT_RIGHT=180;CHECK(LEIDA_DATA_HANDLE7(out,points,720)==91);
 BLUE_ANGLE_LEFT_RIGHT=-1;CHECK(LEIDA_DATA_HANDLE7(out,points,720)<=1);
 for(i=0;i<12;i++){points[i].angle=(float)i;points[i].distance=i<5?1000:2000;}
 right=LEIDA_DATA_HANDLE9(points,12);CHECK(right>0 && right<200);
 for(i=0;i<12;i++)points[i].angle=180.0f-i;
 left=LEIDA_DATA_HANDLE8(points,12);CHECK(left>0 && left<200);CHECK(abs((int)left-(int)right)<40);
 for(i=5;i<12;i++)points[i].angle-=10;CHECK(LEIDA_DATA_HANDLE8(points,12)==0);
 for(i=0;i<12;i++)points[i].angle=i+(i>=5?10.0f:0);CHECK(LEIDA_DATA_HANDLE9(points,12)==0);
 for(i=0;i<8;i++){CHECK(LEIDA_DATA_HANDLE8(points,(uint16_t)i)==0);CHECK(LEIDA_DATA_HANDLE9(points,(uint16_t)i)==0);}
 for(i=0;i<12;i++){points[i].angle=(float)i;points[i].distance=1000+i*20;}
 CHECK(LEIDA_DATA_HANDLE9(points,12)==0);
 /* 20261008 断点y下限回归: HANDLE8/9 必须在**搜索内部**跳过"贴车身的突变"并继续找
 * 后面的真开口。复核复现过: first=60 → 主循环事后清零 → later=281 被整侧丢掉。
 * 构造: 角度0..19, 距离 300(i<=3) / 1000(4..14) / 2000(i>=15) → 两处突变:
 *   i=3  处 y = 300*sin(2°)  ≈ 10   (贴车身, 应被跳过)
 *   i=14 处 y = 1000*sin(13°) ≈ 225 (真开口, 应被报出) */
for(i=0;i<20;i++){points[i].angle=(float)i;points[i].distance=(i<=3)?300.0f:((i<=14)?1000.0f:2000.0f);}
duandian_MIN_Y=0.0f;
{uint16_t a=LEIDA_DATA_HANDLE9(points,20);CHECK(a>0 && a<30);}      /* 旧契约: 报最近那个 */
duandian_MIN_Y=200.0f;
{uint16_t b=LEIDA_DATA_HANDLE9(points,20);CHECK(b>=200 && b<300);}  /* 新契约: 跳过它, 继续找到后面的 */
duandian_MIN_Y=5.0f;
{uint16_t c=LEIDA_DATA_HANDLE9(points,20);CHECK(c>0 && c<30);}      /* 下限放宽后又报最近的 */
duandian_MIN_Y=0.0f;                                                /* 还原 */

 printf("PASS %u nearest-boundary/gap checks\n",checks);return 0;
}
'''
p=OUT/'geometry.c';p.write_text(code,encoding='utf-8')
r=run('geometry',p);(OUT/'geometry_summary.json').write_text(json.dumps(r,indent=2))
