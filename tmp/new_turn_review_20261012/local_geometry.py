"""Compare frozen clouds using the SAME offline geometry definition as old successful runs.
No firmware edits. Positions are local front-axle estimates, not total displacement.
"""
from pathlib import Path
import sys, csv, json, math, collections
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tmp/turn_comparison_20261012'))
from contour_reference import features

LOG = ROOT/'robocup-新建文件夹队/robocup-新建文件夹/上位机/log'
OUT = Path(__file__).resolve().parent
TAGS = ('222930','223105','223130','223154','223226','223303','224402')

def rows(path):
    with path.open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))

result = []
for tag in TAGS:
    file, = LOG.glob(f'v2_1012_{tag}_*.cloud.csv')
    groups = collections.defaultdict(list)
    meta = {}
    for r in rows(file):
        key = tuple(r[k] for k in ('session','control_seq','param_revision'))
        groups[key].append((int(r['point_index']),float(r['x_mm']),float(r['y_mm'])))
        meta[key] = r
    direction = -1 if tag in ('222930','223105','224402') else 1
    for key, data in groups.items():
        pts = np.array([[x,y] for _,x,y in sorted(data)])
        r = dict(tag=tag, session=key[0], seq=int(key[1]), revision=int(key[2]),
                 accepted=False, complete=meta[key]['complete'], point_count=len(pts))
        left, right = features(pts,-1), features(pts,1)
        if left['valid'] and right['valid']:
            angles = [math.atan(left['near_a']), math.atan(right['near_a'])]
            if left['cluster'] != right['cluster'] and abs(angles[0]-angles[1])<.2 and max(abs(left['near_a']),abs(right['near_a']))<.6:
                dl,dr = left['front_axle_normal_distance_mm'],right['front_axle_normal_distance_mm']
                r.update(accepted=True, outward_mm=-direction*(dl-dr)/2,
                         equivalent_width_mm=dl+dr, local_yaw_rad=sum(angles)/2,
                         left_mm=dl, right_mm=dr)
        outer = left if direction==1 else right
        r['outer_feature_50mm_y_range'] = outer.get('deviation_y_bins_mm',{}).get('50')
        r['near_wall_failures'] = [w.get('reason') for w in (left,right)]
        result.append(r)
    subset = [r for r in result if r['tag']==tag]
    print(tag, [(r['seq'],round(r['outward_mm'],1) if r['accepted'] else 'invalid',r['outer_feature_50mm_y_range']) for r in subset])
(OUT/'local_geometry.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
