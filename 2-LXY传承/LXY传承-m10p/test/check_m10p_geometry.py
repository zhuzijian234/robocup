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
static int16_t angle_heads[720], angle_next[LEIDA_DATA_COUNTER];
static unsigned visits;
float BLUE_ANGLE_LEFT_RIGHT=90;
static unsigned checks;
#define CHECK(x) do{++checks;if(!(x)){printf("FAIL %d: %s\n",__LINE__,#x);return 1;}}while(0)
'''
code+='\n'.join(fn(n) for n in ['angle_index_build','angle_nearest','LEIDA_Distance','boundary_extract','LEIDA_DATA_HANDLE6','LEIDA_DATA_HANDLE7','LEIDA_DATA_HANDLE8','LEIDA_DATA_HANDLE9'])
code=code.replace('float d = points[i].distance, diff', '++visits; float d = points[i].distance, diff')
code+=r'''
static _LEIDA_DATA points[800],out[401];
static int brute_nearest(float angle,unsigned count){
 unsigned i;int best=-1;float nearest=2001;
 for(i=0;i<count;i++){
  float d=points[i].distance,diff=fabsf(points[i].angle-angle);
  if(diff>180)diff=360-diff;
  if(d>=100 && d<=2000 && diff<=.5f && d<nearest){best=(int)i;nearest=d;}
 }
 return best;
}
static int compare_float(const void *a,const void *b){float x=*(const float*)a,y=*(const float*)b;return (x>y)-(x<y);}
static float brute_width(unsigned count){
 static float sums[800*800];unsigned i,j,n=0;
 for(i=0;i<count;i++)if(points[i].angle>=150 && points[i].angle<=210){
  float opposite=points[i].angle+180;if(opposite>=360)opposite-=360;
  for(j=0;j<count;j++){
   float diff=fabsf(points[j].angle-opposite);if(diff>180)diff=360-diff;
   if(diff<=1)sums[n++]=points[i].distance+points[j].distance;
  }
 }
 qsort(sums,n,sizeof(float),compare_float);
 return n>=6 && sums[5]<5000?sums[5]:5000;
}
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
 /* Exact selection equivalence against independent exhaustive searches.
  * Include shuffled inputs, duplicate angles/ranges, wrap, holes and empty input. */
 {
  uint32_t seed=456;unsigned trial,j,count;
  for(trial=0;trial<100;trial++){
   count=trial==0?0:trial==1?800:720;
   for(j=0;j<count;j++){
    seed=seed*1664525u+1013904223u;points[j].angle=(seed%36001)/100.0f;
    seed=seed*1664525u+1013904223u;points[j].distance=(float)(seed%4901+100);
   }
   if(count){points[0].angle=360;points[1].angle=0;points[0].distance=points[1].distance=500;}
   angle_index_build(points,(uint16_t)count);
   for(j=0;j<=720;j++)CHECK(angle_nearest(points,j*.5f)==brute_nearest(j*.5f,count));
   CHECK(LEIDA_Distance(points,(uint16_t)count)==brute_width(count));
  }
 }
 for(i=0;i<720;i++){points[i].angle=i*.5f;points[i].distance=1000;}
 angle_index_build(points,720);visits=0;
 for(i=0;i<250;i++)(void)angle_nearest(points,(i%125)*.6f);
 CHECK(visits==1250); /* Brute force: 250*720=180000 distance checks. */
 printf("Indexed nearest: %u candidate visits vs 180000 exhaustive visits\n",visits);
 printf("PASS %u nearest-boundary/gap checks\n",checks);return 0;
}
'''
p=OUT/'geometry.c';p.write_text(code,encoding='utf-8')
r=run('geometry',p);(OUT/'geometry_summary.json').write_text(json.dumps(r,indent=2))
