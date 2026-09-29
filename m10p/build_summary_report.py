"""Build the consolidated M10P test report from recorded JSON and figures."""
from pathlib import Path
from collections import Counter
import json
from xml.sax.saxutils import escape
from reportlab.pdfgen import canvas
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, PageBreak
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_CENTER
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.utils import ImageReader

ROOT=Path(__file__).resolve().parent
R=ROOT/'reports'
OUT=ROOT/'M10P雷达实测数据与协议核对报告.pdf'
pdfmetrics.registerFont(TTFont('CN','C:/Windows/Fonts/msyh.ttc'))
pdfmetrics.registerFont(TTFont('CNBold','C:/Windows/Fonts/msyhbd.ttc'))
pdfmetrics.registerFont(TTFont('Mono','C:/Windows/Fonts/consola.ttf'))
pdfmetrics.registerFontFamily('CN',normal='CN',bold='CNBold')
W,H=595.276,841.89
styles={
 'title':ParagraphStyle('title',fontName='CNBold',fontSize=23,leading=33,spaceAfter=12),
 'h1':ParagraphStyle('h1',fontName='CNBold',fontSize=17,leading=25,spaceAfter=12),
 'h2':ParagraphStyle('h2',fontName='CNBold',fontSize=12,leading=18,spaceBefore=10,spaceAfter=6),
 'body':ParagraphStyle('body',fontName='CN',fontSize=10,leading=17,spaceAfter=8,wordWrap='CJK'),
 'small':ParagraphStyle('small',fontName='CN',fontSize=8.3,leading=13,spaceAfter=6,wordWrap='CJK',textColor=colors.HexColor('#475569')),
 'cell':ParagraphStyle('cell',fontName='CN',fontSize=8.2,leading=12,wordWrap='CJK',alignment=TA_CENTER),
 'cellhead':ParagraphStyle('cellhead',fontName='CNBold',fontSize=8.2,leading=12,wordWrap='CJK',alignment=TA_CENTER),
 'mono':ParagraphStyle('mono',fontName='Mono',fontSize=8.3,leading=12,spaceAfter=5),
}
story=[]
def p(s,style='body'):
    story.append(Paragraph(s,styles[style]))
def h(s):p(s,'h2')
def page(s):
    if story:story.append(PageBreak())
    p(s,'h1')
def table(rows,widths):
    wrapped=[[Paragraph(escape(str(v)).replace('\n','<br/>'),styles['cellhead' if i==0 else 'cell']) for v in row] for i,row in enumerate(rows)]
    t=Table(wrapped,colWidths=widths,repeatRows=1,hAlign='LEFT')
    t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#dfeaf3')),
        ('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor('#f6f8fa')]),
        ('GRID',(0,0),(-1,-1),0.4,colors.HexColor('#d9d9d9')),
        ('VALIGN',(0,0),(-1,-1),'MIDDLE'),('TOPPADDING',(0,0),(-1,-1),5),
        ('BOTTOMPADDING',(0,0),(-1,-1),5),('LEFTPADDING',(0,0),(-1,-1),5),('RIGHTPADDING',(0,0),(-1,-1),5)]))
    story.append(t);story.append(Spacer(1,8))
def picture(name,width=499):
    path=R/name; iw,ih=ImageReader(str(path)).getSize()
    story.append(Image(str(path),width=width,height=width*ih/iw));story.append(Spacer(1,5))
def load(name):return json.loads((R/name).read_text(encoding='utf-8'))
names=['hardware_check_verified.json']+[p.name for p in sorted(R.glob('20260927_*.json'))]+[p.name for p in sorted(R.glob('20260928_*.json'))]
records=[load(n) for n in names]
assert len(records)==10
totals=Counter()
for r in records:
    assert sum(r['frame_lengths'].values())==r['frames']
    assert sum(int(k)*v for k,v in r['frame_lengths'].items())+r['discarded_bytes']+r['buffered_bytes']==r['bytes']
    totals.update({int(k):v for k,v in r['frame_lengths'].items()})
frames=sum(totals.values());variable=100*(frames-totals[160])/frames
audit=load('最新原始数据_帧边界独立审计.json')
box=load('盒子后移_分析.json')

p('M10P雷达实测数据<br/>与协议核对报告','title')
p('测试日期 2026年9月27日至28日<br/>用途 雷达到货验证  接收程序适配  厂家协议确认','small')
p('本报告整合串口采集、帧边界独立核对、距离异常复查和前方盒子移动测试。核心结论是：<b>雷达基本测距与旋转扫描有实测支持；电脑收到的原始数据确实包含不同长度的包，不能仅因包长不是160字节就判为丢字节。</b>实际固件是否正式支持这些包长，以及字段布局是否与手册完全一致，仍需厂家确认。')
h('主要结论')
p('1　<b>基本功能符合观察。</b>转速约720 rpm，点速率约2万点/秒；10次记录均覆盖24个角度区域，结构异常帧和角度跳变统计均为0。盒子测试中，前方距离从约22.3 cm连续增大至34.6 cm，与用户描述的放置和后移操作吻合。')
p(f'2　<b>变长现象可重复。</b>10次记录共{frames:,}包，出现156、158、160、162、164、166字节。其中158和160字节占绝大多数；非160字节占{variable:.2f}%。164和166字节目前只有报告统计，未保存对应原始包。')
p('3　<b>已排除本次Python分块切帧制造变长的解释。</b>最新原始文件按帧头、帧尾、长度字段三种独立方式划分，均得到相同的8635个完整包；改变输入分块大小后，全部解析结果的哈希仍一致。')
p('4　<b>仍有待核验事项。</b>旧记录发现一个3080 → 394 → 3070 mm的孤立近点；邻圈场景也变化，无法归因为通信错误。没有做精确测距标定、直接TX抓取或STM32板上验证，不能据此宣称全部测距点无误或整机完成验收。')
h('测试条件')
table([['项目','条件'],['雷达与接收链路','M10P → 串口转接模块 → USB → 电脑'],['串口','COM11  CH9102  512000 bps  8N1'],['主机工具','独立conda环境m10p  Python 3.10.20  pyserial 3.5  Tk 8.6'],['采集方式','读取串口原始字节；可选原样保存bin；不发送电机或配置命令']],[105,394])
p('PASS表示通过该自检程序的基本规则，不代表厂家认证、测距精度达标或已排除所有硬件故障。缺少校验和限制了字节完整性的判断。','small')

page('一  多次测试统计')
p('下表统一使用报告中的结束时间。A1为首次修正变长接收后的实机验证，A10为前方盒子测试。早期hardware_check.json和plot_check.json使用过窄的长度上限产生误报，未纳入本表。','small')
rows=[['编号','结束时间','时长 s','包数','平均 rpm','点每秒','结果']]
for i,r in enumerate(records,1):
    ts=r['timestamp'];rows.append([f'A{i}',ts[5:10]+' '+ts[11:19],f"{r['duration_s']:.2f}",r['frames'],f"{r['rpm_mean']:.2f}",f"{r['points_per_s']:.0f}",r['status']])
table(rows,[34,120,51,61,69,74,90])
p('各次均为结构异常帧0、角度跳变0。A9的WARN仅由13.09秒时提前结束触发，报告未记录通信故障。A1和A10有完整原始数据可供独立核对，其余A2至A9仅保留JSON统计。','small')
h('各次包长分布')
rows=[['编号','156 B','158 B','160 B','162 B','164 B','166 B']]
for i,r in enumerate(records,1):rows.append([f'A{i}']+[r['frame_lengths'].get(str(n),0) for n in (156,158,160,162,164,166)])
rows.append(['总计']+[totals[n] for n in (156,158,160,162,164,166)])
table(rows,[43,76,76,76,76,76,76])
p(f'总计{frames:,}包；所有报告中的“各包长度之和 + 起始丢弃字节 + 结束残留字节”均与接收总字节数一致。起始丢弃与结束残留通常包含打开、关闭采集时的半帧，不应直接等同于传输丢包。','small')

page('二  原始字节和帧边界独立核对')
p('核对对象为最新30秒采集的latest_capture.bin，共1,372,252字节。原采集代码在调用解析器之前执行raw_file.write(data)，保存的是串口read返回的字节，没有把解析结果重新拼包保存。')
h('三种划分方法结果一致')
p('第一种方法搜索所有A5 5A字节序列，以相邻帧头的实际文件偏移差计数，不依赖长度字段。第二种方法从首个完整帧头开始，按FA FB帧尾结束位置划分，不依赖长度字段。第三种方法只按包内双字节长度向后递进，不在中途搜索或重新同步。三种方法得到的8635组起止偏移全部相同。')
table([['包长','数量','起点示例','下一帧起点','包内长度字段'],
       *[[x['length'],audit['frame_lengths'][str(x['length'])],x['start'],x['end'],x['header'].split(' ')[2]+' '+x['header'].split(' ')[3]] for x in audit['examples']]],[61,65,116,116,141])
p('全部完整包的实际长度与包内长度字段对应；帧尾后紧接下一帧头；8635次可检查的相邻起始角增量全部为15°，其中最后一次利用了末尾不完整包中已经收到的角度字段。文件首部100字节和末尾130字节不计入完整包。')
h('读取分块与固定长度的对照')
p('将同一原始文件分别按1、7、160、4096、65536字节分块喂入现有解析器，五次均得到相同的包长计数与解码结果哈希，结构错误均为0。这排除了本次数据中的read分块边界导致错误切帧的解释。')
p('反过来，从首个帧头强制每160字节切分，共得到8575个块，仅120个块以A5 5A开头，其中仅60个同时满足160字节长度字段与固定位置帧尾；说明固定160字节切分无法保持真实边界。')
h('可直接交给厂家核查的例子')
p('文件偏移100处头部为 A5 5A 00 9E 08 FC 0D 94。00 9E等于158；下一帧起点为258，实际间距也是158，边界为FA FB | A5 5A。偏移118018处则为00 A2，即162，下一帧起点为118180。')
p('<b>结论范围：</b>已确认电脑收到的包长在变化，并排除上述Python切帧因素；尚未直接在雷达TX引脚抓取，因此不能把变长源头确定为雷达固件，也不能排除中间硬件处理。采样时序、转速波动导致点数变化只是解释假设，不能写成厂家已确认的机制。')

page('三  前方盒子测距与后移验证')
p('用户在雷达0°前方放置盒子，初始估计距离22至25 cm，随后缓慢后移。对A10的原始数据取0°±2°范围，每次扫描用非零距离中位数代表前方距离。359次前方扫描共检查6821个点，零距离点为0。')
picture('M10P_盒子后移_距离时间图.png')
table([['相对时段','测距结果'],['前22秒','中位数22.3 cm；逐次扫描中位数范围22.1至22.7 cm'],['约第23至26秒','每秒中位数约25.7、28.3、30.8、32.7 cm'],['最后约2秒','中位数34.6 cm；逐次扫描中位数范围34.5至35.0 cm'],['首尾变化','约12.3 cm']],[116,383])
p('图中时间按包内电机转速和每包15°跨度估算；原始文件没有逐包主机时间戳，因此不是精确的操作计时。蓝色带是同次扫描窗口内点距离的P10至P90，橙点为每秒中位数。')
p('<b>判断：</b>初始测距落在用户给出的22至25 cm范围，后段距离连续增加并再次稳定，与盒子后移相符。本次证据支持前方目标的基本测距与移动响应；由于没有精确距离基准，不能由此确定绝对误差。')
p('A10同时包含4742个158字节包、3862个160字节包、26个156字节包和5个162字节包。变长包混合出现时仍可得到上述连续趋势，但这并不单独证明每一种长度的全部字段均已正确解释。','small')

page('四  距离异常复查')
p('对A1的4311个包做离线复核：比较包内相邻非零距离，以及前后扫描圈同方向的点。点数不同时按最近角度对齐，避免直接比较同一槽号。发现1个可疑孤立近点，位于158字节包，原始文件偏移459687，起始角248°。')
picture('M10P_158字节_可疑距离跳变.png',470)
p('该包第59、60、61点（从0开始）为<b>3080 → 394 → 3070 mm</b>；原始字节为0C 08 | 01 8A | 0B FE。前一圈同方向约3104 mm，后一圈匹配点变为0，邻近还出现887和906 mm。场景存在变化迹象，不能仅凭此点判断为传输损坏、雷达故障或变长造成的异常。')
table([['包长','包数','零值比例','包内孤立尖峰候选','邻圈较小差值P95'],['156',2,'3.68%',0,'10 mm'],['158',2341,'7.35%',1,'12 mm'],['160',1964,'7.65%',0,'12 mm'],['162',4,'16.55%',0,'10 mm']],[48,58,85,161,147])
p('孤立尖峰阈值为中心点与左右各差超过200 mm，左右相差不超过50 mm，三点均非零。表中P95取当前点对前后圈差值较小者，仅纳入三圈均非零的比较点，是相对稳定性指标，不是精度。阈值为本次筛查规则，不是厂家标准。','small')
p('6个156/162字节包未命中同类尖峰，其包内相邻非零点最大差为17至22 mm。162字节包集中在零值较多的方向；按同方向比较，未见零值明显增加。样本少且现场运动未控制，不能排除罕见异常。','small')

page('五  接收代码适配和后续确认')
h('主机与STM32的适配状态')
p('主机check_m10p.py按长度字段组包、动态定位帧尾，并按当前假定布局计算点数，实时图直接使用串口数据。当前规则仍沿用文档中的8字节头部、10字节保留时间区、2字节帧尾和15°角度跨度，固件布局需厂家确认。')
p('本次复核的LXY传承-m10p工程仍固定160字节缓冲与帧长判断，固定70槽解码和第158、159字节帧尾。它会拒绝其他长度的正常结构包，并反复丢弃正在拼接的扫描圈。<b>Python测试通过不代表STM32工程已经适配。</b>本次工作没有修改该固件。')
p('需在m10p_config.h、m10p.c中区分最大容量与实际包长，动态计算测距槽数和帧尾位置；保留异常重同步、角度连续性、整圈和时效检查。用真实bin回放C解析器，并提供合理的模拟到达时间，再完成构建回归及USART/DMA板上验证。')
h('请厂家确认的问题')
p('1　该M10P型号及固件是否允许156至166字节或其他长度的输出包？总长度字段是否为接收依据，合法上下限是什么？<br/>2　变长时哪些字段变化？测距区、保留时间区的位置如何确定，0xFFFF和高反标志如何解释？<br/>3　每包是否始终覆盖15°，角度是否按有效点数均分？<br/>4　手册的160字节与70点是固定约束还是典型示例，是否有更新协议？<br/>5　对于已提供的孤立近点，是否有近距离、边缘目标或回波无效值方面的说明？')
h('建议的验证顺序')
p('先由厂家核对原始包；再用已知距离和方向的平整静止目标验证距离及角度。若仍怀疑链路，保持供电和雷达不变更换合适的USB转TTL模块，或用逻辑分析仪直接捕获雷达TX作对照。只有直接对照源端与接收端数据，才能进一步定位中间链路是否改变数据。')
h('综合判断')
p('现有证据支持基本测距和旋转扫描正常，未呈现大量结构错误或角度断裂；但无法量化损坏概率，也不能保证每个测距点正确。最需要解决的是<b>实机协议与手册及固件接收代码之间的差异</b>，不能把变长直接称为丢字节或硬件损坏。')

page('附录  证据文件与复现')
p('以下文件位于项目m10p目录，除另注外路径相对reports子目录。时间为北京时间。原始bin保留接收字节；JSON与图表为派生证据。')
table([['编号','报告文件'],*[[f'A{i}',n] for i,n in enumerate(names,1)]],[43,456])
h('两份用于详细核验的原始文件')
p('A1　hardware_check_verified.bin　685216字节','small')
p('SHA256  e57aa16fa97db1f6998819e32fa96e27ed529e6ad28247ceafadb6712189ed5e','mono')
p('A10　latest_capture.bin　1372252字节','small')
p('SHA256  cc8fcc5d075bf7f56652b666048566dd65681deb505af3f9992019f1afe9ef34','mono')
h('分析记录与样本')
p('最新原始数据_帧边界独立审计.json：三种边界检查与五种分块测试。<br/>distance_analysis.json及距离异常复核.md：A1的距离检查细节。<br/>盒子后移_分析.json：A10前方目标的每秒距离统计。<br/>M10P_厂家原始包样本.zip：156、158、160、162字节各两个完整样本及图片；164/166仅统计说明。','small')
h('复现命令')
p('在m10p目录激活conda环境m10p，执行以下命令。再次采集前请为raw指定新文件名，避免覆盖latest_capture.bin。','small')
p('python check_m10p.py --port COM11 --seconds 30 --plot --raw reports/new_capture.bin','mono')
p('独立审计脚本audit_frame_boundaries.py读取latest_capture.bin；盒子分析脚本analyze_front_box.py读取同一文件；距离异常脚本analyze_distance_capture.py读取hardware_check_verified.bin。绘图分析脚本还需要numpy与matplotlib，本次在已有分析环境执行，未加入最小m10p采集环境。','small')
p('依据资料：M10P_20K_数据输出格式.pdf、M10P_20KHz_用户手册.pdf及厂家M10P-uart-Python3.py。手册与实机差异尚未得到厂家最终解释。','small')

def footer(c,doc):
    c.setFont('CN',8);c.setFillColor(colors.HexColor('#64748b'))
    c.drawString(48,28,'M10P雷达实测数据与协议核对报告  2026年9月28日')
    c.drawRightString(W-48,28,str(doc.page))
doc=SimpleDocTemplate(str(OUT),pagesize=(W,H),leftMargin=48,rightMargin=48,topMargin=42,bottomMargin=46,
                      title='M10P雷达实测数据与协议核对报告',author='测试记录整理')
doc.build(story,onFirstPage=footer,onLaterPages=footer)
print(OUT)
print('Frames',frames,'totals',dict(sorted(totals.items())),'non160_pct',variable)
