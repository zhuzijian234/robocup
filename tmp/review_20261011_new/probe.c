#include <stdio.h>
#include "D:/OneDrive/Desktop/robocup/robocup-新建文件夹队/robocup-新建文件夹/HARDWARE/CENTRE_LINE/path_track.c"

int main(void) {
  int id, n, i, j, nl, nr, shared; float width; PathPoint pts[400], l[12], r[12];
  while (scanf("%d %d %f", &id, &n, &width)==3) {
    PathGeometry state={0}; PathObservation obs;
    for (i=0;i<n;i++) if(scanf("%f %f", &pts[i].x,&pts[i].y)!=2) return 2;
    nl=boundary(pts,n,-1,width,10,l); nr=boundary(pts,n,1,width,10,r); shared=0;
    for(i=0;i<nl;i++) for(j=0;j<nr;j++)
      if (point_distance(l[i],r[j])<1) ++shared;
    state.width=width; state.configured_width=500; state.measured=1;
    Path_Build(&state,pts,n,500,900,10,&obs);
    printf("%d %d %d %d %d %d %d %.3f %.3f\n",id,nl,nr,shared,
      obs.geometry_reason,obs.source,obs.model,obs.target_x,obs.ref_y);
  } return 0;
}
