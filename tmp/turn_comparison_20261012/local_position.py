"""两条局部近墙下的前轴位置估计；不是车身总横移量或碰撞余量。"""
import json, math
import numpy as np
from contour_reference import features, OUT

RUNS = [('160711',-1),('161100',-1),('161233',-1),('161426',1),
        ('161746',1),('162029',1),('164512',1),('164616',1),
        ('164757',1),('164942',-1),('165050',-1),('165134',-1)]

def main():
    records=[]
    for tag, outward in RUNS:
        clouds=json.loads((OUT/tag/'clouds.json').read_text(encoding='utf-8'))
        for cloud in clouds[:4]:
            points=np.array(cloud['points'])
            left,right=features(points,-1),features(points,1)
            row=dict(tag=tag,seq=cloud['meta']['control_seq'],accepted=False)
            if left['valid'] and right['valid']:
                # 不把同一簇误当两墙；近处墙面差异过大时不估计局部中间位置。
                distinct=left['cluster'] != right['cluster']
                angles=[math.atan(left['near_a']),math.atan(right['near_a'])]
                accepted=distinct and abs(angles[0]-angles[1])<.2 and max(abs(left['near_a']),abs(right['near_a']))<.6
                if accepted:
                    dl=left['front_axle_normal_distance_mm']
                    dr=right['front_axle_normal_distance_mm']
                    row.update(accepted=True,left_distance_mm=dl,right_distance_mm=dr,
                               equivalent_width_mm=dl+dr,
                               outward_front_axle_offset_estimate_mm=outward*(dl-dr)/2,
                               near_angles=angles)
            records.append(row)
    (OUT/'local_position_estimates.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
    print('Local position estimates:',sum(r['accepted'] for r in records),'/',len(records))

if __name__=='__main__':
    main()
