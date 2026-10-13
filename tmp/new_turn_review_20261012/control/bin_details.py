from pathlib import Path
import json,math,csv,collections
O=Path(__file__).resolve().parent
frames=json.loads((O/'frames.json').read_text())
reports=[]
for f in frames:
 pts=f['points'];clusters=[];b=0
 for i in range(1,len(pts)+1):
  if i==len(pts) or math.dist(pts[i-1],pts[i])>=100:
   clusters.append((b,i));b=i
 for side in [-1,1]:
  b,e=max(clusters,key=lambda r:sum(100<=y<400 and side*x>60 for x,y in pts[r[0]:r[1]]))
  bins=collections.defaultdict(list)
  for x,y in pts[b:e]:
   if 100<=y<1300:bins[int((y-100)//50)].append((x,y))
  means={k:(sum(x for x,y in p)/len(p),sum(y for x,y in p)/len(p)) for k,p in bins.items() if len(p)>=2}
  near=[v for k,v in means.items() if k<6]
  if len(near)<5:continue
  n=len(near);sx=sum(x for x,y in near);sy=sum(y for x,y in near);yy=sum(y*y for x,y in near)-sy*sy/n
  if yy<1:continue
  a=(sum(x*y for x,y in near)-sx*sy/n)/yy;b0=(sx-a*sy)/n
  for k in range(24):
   points=bins.get(k,[]);mean=means.get(k)
   dev=0;turn=''
   if mean:dev=-side*(mean[0]-a*mean[1]-b0)/math.sqrt(1+a*a)
   if k>=10 and all(i in means for i in range(k-4,k+1)):
    turn=-side*(math.atan2(means[k][0]-means[k-4][0],means[k][1]-means[k-4][1])-math.atan(a))
   reports.append(dict(tag=f['tag'],seq=f['key'][3],side=side,begin=b,end=e,bin=k,n=len(points),x=mean[0] if mean else '',y=mean[1] if mean else '',deviation=dev,turn=turn))
with (O/'bin_details.csv').open('w',newline='',encoding='utf-8') as fp:
 w=csv.DictWriter(fp,fieldnames=reports[0]);w.writeheader();w.writerows(reports)
for row in reports:
 if row['tag']=='224402' and row['seq']=='14163' and row['side']==1:
  print(row)
