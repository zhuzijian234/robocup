from pathlib import Path
exec((Path(__file__).parent/'analyze.py').read_text(encoding='utf-8').split('summaries=[]')[0])
for tag in ('180345','181529','181925','182048'):
    file=next((PC/'log').glob(f'v2_1010_{tag}_*.bin'))
    p=core.parse_file(str(file))
    rows=list(csv.DictReader((OUT/f'{tag}_joined.csv').open(encoding='utf-8-sig')))
    rows={int(r['control_seq']):r for r in rows}
    groups=core.v2.assemble_clouds(p.v2.messages)
    fig,axs=plt.subplots(1,4,figsize=(16,6),sharex=True,sharey=True)
    print('\nCLOUD',tag)
    for ax,(_,g) in zip(axs,sorted(groups.items())):
        m=g['meta'];r=rows.get(m['control_seq'],{});pts=np.array(list(g['points'].values()))
        ax.scatter(pts[:,0],pts[:,1],s=3,color='gray')
        ax.add_patch(plt.Rectangle((-130,-205),260,350,fill=False,color='black'))
        for side,color in [('left_near','blue'),('right_near','red'),('left_far','cyan'),('right_far','orange')]:
            if r.get(side+'_a'):
                a,b,lo,hi=[float(r[side+'_'+x]) for x in ('a','b','min_y','max_y')]
                if hi>lo:
                    y=np.linspace(lo,hi,40);ax.plot(a*y+b,y,color=color,label=side)
        if r.get('geometry_reason') == '0' and r.get('target_x') and r.get('ref_y'):
            ax.scatter(float(r['target_x']),float(r['ref_y']),marker='x',s=60,color='green')
        ax.set_title(f"seq {m['control_seq']} geom {r.get('geometry_reason','?')}\n{len(pts)}/{m['total_points']} points")
        ax.set_xlim(-950,950);ax.set_ylim(-250,1350);ax.set_aspect('equal');ax.grid(alpha=.3)
        print(m['control_seq'],'device_t',r.get('device_t'),'geom',r.get('geometry_reason'),'source',r.get('source'),'width',r.get('width_candidate'),'rms',r.get('left_rms'),r.get('right_rms'),'slope',r.get('left_near_a'),r.get('right_near_a'),'fit',r.get('fit_rejected'),'trigger',m['trigger_flags'],'complete',len(pts)==m['total_points'])
    axs[0].legend(fontsize=7);fig.suptitle(tag+' snapshot; x right, y forward (mm)');fig.tight_layout();fig.savefig(OUT/f'{tag}_cloud.png');plt.close(fig)
