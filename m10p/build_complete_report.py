"""Build the consolidated M10P test report from recorded JSON and figures."""
from pathlib import Path
from collections import Counter
import json
import hashlib
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
OUT=ROOT/'M10P雷达测试关键结论完整记录_20260929.pdf'
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
assert frames == 46861
assert Counter(r['status'] for r in records) == {'PASS':9,'WARN':1}
assert all(r['bad_frames']==0 and r['angle_jumps']==0 for r in records)
assert hashlib.sha256((R/'latest_capture.bin').read_bytes()).hexdigest()==audit['sha256']==box['sha256']
assert audit['all_three_methods_identical'] and audit['complete_frames']==8635
assert len({trial['decoded_sha256'] for trial in audit['read_chunk_trials']})==1

p('M10P雷达测试<br/>关键结论完整记录','title')
p('测试日期 2026年9月27日至28日　复核日期 2026年9月29日<br/>用途 雷达到货验证  接收程序适配  商家沟通与后续核验','small')
p('本报告整合串口采集、帧边界独立核对、距离异常复查、盒子移动测试及用户转述的商家说明。<b>基本测距与旋转扫描有实测支持；电脑收到的原始数据确实包含不同长度的包，不能仅因不是160字节就判为丢字节。</b>用户转述商家已说明测距区长度不同属正常现象，雷达会剔除无效点；合法最大长度及具体字段布局仍待明确。')
h('主要结论')
p('1　<b>基本功能符合观察。</b>转速约720 rpm，点速率约2万点/秒；10次记录均覆盖24个角度区域，结构异常帧和角度跳变统计均为0。盒子测试中，前方距离从约22.3 cm连续增大至34.6 cm，与用户描述的放置和后移操作吻合。')
p(f'2　<b>变长现象可重复。</b>10次记录共{frames:,}包，出现156、158、160、162、164、166字节。其中158和160字节占绝大多数；非160字节占{variable:.2f}%。164和166字节目前只有报告统计，未保存对应原始包。')
p('3　<b>已排除本次Python分块切帧制造变长的解释。</b>最新原始文件按帧头、帧尾、长度字段三种独立方式划分，均得到相同的8635个完整包；改变输入分块大小后，全部解析结果的哈希仍一致。')
p('4　<b>仍有待核验事项。</b>旧记录发现一个3080 → 394 → 3070 mm的孤立近点；邻圈场景也变化，无法归因为通信错误。没有做精确测距标定、直接TX抓取或STM32板上验证。用户已明确要求暂不修改接收层，现有固件保持原样。')
h('测试条件')
table([['项目','条件'],['雷达与接收链路','M10P → 串口转接模块 → USB → 电脑'],['串口','COM11  CH9102  512000 bps  8N1'],['主机工具','独立conda环境m10p  Python 3.10.20  pyserial 3.5  Tk 8.6'],['采集方式','读取串口原始字节；可选原样保存bin；不发送电机或配置命令']],[105,394])
p('PASS表示通过该自检程序的基本规则，不代表厂家认证、测距精度达标或已排除所有硬件故障。缺少校验和限制了字节完整性的判断。','small')

page('一  结论依据与商家说明')
h('实测已经证明的事实')
p('电脑保存的原始字节流里，156、158、160、162字节包的实际边界、长度字段及帧尾可相互核对。Python读入分块大小改变后，结果不变。多次采集的平均转速约719至720 rpm；盒子测试在前方产生连续、符合操作描述的距离变化。以上结论可用保留文件复现。')
h('商家说明的来源与适用范围')
p('用户在本对话转述：“我问商家说range长度不一样是正常的,雷达把无效点刨除了”。本报告把它作为<b>用户转述的商家说明</b>记录，不将其写成已取得的厂家正式协议或逐项书面确认。该说明支持“测距区长度可以变化、短包不一定是丢字节”的解释。')
p('不能由这句话进一步推断所有字段位置、所有长度上限均已确认。尤其是162、164、166字节超过160字节，仅从“固定70点里剔除无效点”这一机制无法解释变长；如果剔除之前的采样点数本身可变，则可能解释，但商家尚未明确这一点。')
h('截图与手册应如何理解')
p('手册写明FFFF表示无效点，实际点数应减一，角度间隔为15°除以实际点数；同时又用固定160字节、70槽及固定时间区与帧尾偏移举例。单看这段文字，既不能证明雷达发送前删除字节，也不能把有效点数减少直接等同于总包长减少。商家的补充说明为实机短包提供了额外解释，仍需要完整协议澄清其余细节。')
h('需要修正的早期表述')
p('“所有数据都正常”“距离都是准的”应收窄为：基本测距和特定盒子实验有实测支持，未完成绝对精度校准。“转速波动导致包长变化”是待验证假设；“稀少长包就是错误包”没有证据支持；“损坏概率很低”无法给出可靠概率，准确说法是尚未发现足以判定硬件损坏的证据。')
p('此前“实时图保证正确”也应理解为：图确实使用实时测量数据，当前解码在已测场景中表现一致；没有证明所有方向、全部距离及每种包长的角度分配都准确。')
h('当前工程决策')
p('用户已要求“先不修改”。本次仅整理文档并复核现有证据，没有修改STM32接收层、烧录固件或进行新的硬件控制。未来若授权适配，应修改接收层并保留错误检查，再通过真实数据回放与板上验证。')

page('二  多次测试统计')
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

page('三  原始字节和帧边界独立核对')
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

page('四  当前解码规则与实时图的含义')
p('以下是当前Python工具实际采用的规则，<b>字节偏移均从0开始</b>。基础定义来自所供手册；把固定示例推广到变长帧时，保留区长度不变等细节仍是当前实现的假定，须与商家的实际固件协议对齐。')
table([['字段','当前读取方式'],['帧头与长度','A5 5A；第2和3字节是大端总长度L'],['起始角','第4和5字节大端整数除以100，单位度'],['转速','第6和7字节为计数值；rpm = 2500000 / 计数值'],['测距区','第8字节开始，至末尾12字节之前；每槽2字节'],['保留区及帧尾','假定最后12字节由10字节保留区和FA FB两字节组成'],['点数','槽数 = (L - 20) / 2；FFFF槽剔除后得到m'],['距离与标志','先判断FFFF；其余清除bit15得到毫米距离，bit15表示高反'],['点角度','起始角 + 15° × 点索引 / m，再对360°取模']],[100,399])
p('在上述布局假定下，156、158、160、162、164、166字节分别对应68、69、70、71、72、73个槽。这里是由长度推算的槽数，不是独立测得的真实采样次数；0距离保留在角度排序中，但不绘制。详细分析的A1和A10都未读出FFFF槽，此现象与提前剔除相容，单独不足以证明剔除机制。')
h('图像来自实时数据')
p('实时程序直接从串口接收并解析点云，最多保留最近24包，且删除超过0.25秒的显示数据；约每0.1秒重画一次。12 Hz、每圈24包时，显示通常对应最近约一圈，刷新率约10次/秒。它不是模拟数据，也不是自动回放历史文件。')
p('图上方为雷达原生0°，即手册所述出线端对面；角度顺时针增大。只有雷达安装方向一致时，图上方才是车头。绿色为普通点，黄色为高反点。零距离和超过--range显示半径的点不画，所以图上缺口不等于该处一定没有物体。')
h('程序通过意味着什么')
p('主机允许22至512的偶数长度，这只是防止错误长度造成无限等待的实现边界，不能当作厂家认可的包长范围。程序会检查帧头、帧尾、角度和非零转速计数；坏数据会重新同步。无校验和意味着结构正确仍不能保证每个距离字节都正确。')
p('约2万点/秒为当前解析得到的点速率，包含零距离点，不等于每秒2万个有效非零回波。累计覆盖24个15°区域表示扫描角度有覆盖，不代表每个方向均有有效回波。')
p('PASS主要规则：采集至少5秒；平均转速10至14 Hz；24个区域全部出现且至少跨零点两次；角度跳变比例不超过1%。<br/>同时要求结构错误为0，接收间隔不超过0.5秒，帧率与转速推算值之比在0.85至1.15，非零点占解析点数至少10%。提前结束另报WARN。这些是工具的初筛阈值，并非厂家验收标准。','small')

page('五  前方盒子测距与后移验证')
p('用户在雷达0°前方放置盒子，初始估计距离22至25 cm，随后缓慢后移。对A10的原始数据取0°±2°范围，每次扫描用非零距离中位数代表前方距离。359次前方扫描共检查6821个点，零距离点为0。')
picture('M10P_盒子后移_距离时间图.png')
table([['相对时段','测距结果'],['前22秒','中位数22.3 cm；逐次扫描中位数范围22.1至22.7 cm'],['约第23至26秒','每秒中位数约25.7、28.3、30.8、32.7 cm'],['最后约2秒','中位数34.6 cm；逐次扫描中位数范围34.5至35.0 cm'],['首尾变化','约12.3 cm']],[116,383])
p('图中时间按包内电机转速和每包15°跨度估算；原始文件没有逐包主机时间戳，因此不是精确的操作计时。蓝色带是同次扫描窗口内点距离的P10至P90，橙点为每秒中位数。')
p('<b>判断：</b>初始测距落在用户给出的22至25 cm范围，后段距离连续增加并再次稳定，与盒子后移相符。本次证据支持前方目标的基本测距与移动响应；由于没有精确距离基准，不能由此确定绝对误差。')
p('A10同时包含4742个158字节包、3862个160字节包、26个156字节包和5个162字节包。变长包混合出现时仍可得到上述连续趋势，但这并不单独证明每一种长度的全部字段均已正确解释。','small')

page('六  距离异常复查')
p('对A1的4311个包做离线复核：比较包内相邻非零距离，以及前后扫描圈同方向的点。点数不同时按最近角度对齐，避免直接比较同一槽号。发现1个可疑孤立近点，位于158字节包，原始文件偏移459687，起始角248°。')
picture('M10P_158字节_可疑距离跳变.png',470)
p('该包第59、60、61点（从0开始）为<b>3080 → 394 → 3070 mm</b>；原始字节为0C 08 | 01 8A | 0B FE。前一圈同方向约3104 mm，后一圈匹配点变为0，邻近还出现887和906 mm。场景存在变化迹象，不能仅凭此点判断为传输损坏、雷达故障或变长造成的异常。')
table([['包长','包数','零值比例','包内孤立尖峰候选','邻圈较小差值P95'],['156',2,'3.68%',0,'10 mm'],['158',2341,'7.35%',1,'12 mm'],['160',1964,'7.65%',0,'12 mm'],['162',4,'16.55%',0,'10 mm']],[48,58,85,161,147])
p('孤立尖峰阈值为中心点与左右各差超过200 mm，左右相差不超过50 mm，三点均非零。表中P95取当前点对前后圈差值较小者，仅纳入三圈均非零的比较点，是相对稳定性指标，不是精度。阈值为本次筛查规则，不是厂家标准。','small')
p('6个156/162字节包未命中同类尖峰，其包内相邻非零点最大差为17至22 mm。162字节包集中在零值较多的方向；按同方向比较，未见零值明显增加。样本少且现场运动未控制，不能排除罕见异常。','small')

page('七  接收代码适配和后续确认')
h('主机与STM32的适配状态')
p('主机check_m10p.py按长度字段组包、动态定位帧尾，并按当前假定布局计算点数，实时图直接使用串口数据。当前规则仍沿用文档中的8字节头部、10字节保留时间区、2字节帧尾和15°角度跨度，固件布局需厂家确认。')
p('2026年9月29日复核的LXY传承-m10p工程仍固定160字节缓冲与帧长判断，固定70槽解码和第158、159字节帧尾。它会拒绝其他长度的正常结构包，并反复丢弃正在拼接的扫描圈。<b>Python测试通过不代表STM32工程已经适配。</b>用户要求先不修改，本次未改固件。')
p('需在m10p_config.h、m10p.c中区分最大容量与实际包长，动态计算测距槽数和帧尾位置；保留异常重同步、角度连续性、整圈和时效检查。用真实bin回放C解析器，并提供合理的模拟到达时间，再完成构建回归及USART/DMA板上验证。')
h('请厂家确认的问题')
p('1　该M10P型号及固件是否允许156至166字节或其他长度的输出包？总长度字段是否为接收依据，合法上下限是什么？<br/>2　变长时哪些字段变化？测距区、保留时间区的位置如何确定，0xFFFF和高反标志如何解释？<br/>3　每包是否始终覆盖15°，角度是否按有效点数均分？<br/>4　手册的160字节与70点是固定约束还是典型示例，是否有更新协议？<br/>5　对于已提供的孤立近点，是否有近距离、边缘目标或回波无效值方面的说明？')
h('建议的验证顺序')
p('在商家已说明剔除无效点的基础上，继续核对长包上限、保留区及角度规则；再用已知距离和方向的平整静止目标验证距离及角度。若仍怀疑链路，保持供电和雷达不变更换合适的USB转TTL模块，或用逻辑分析仪直接捕获雷达TX作对照，定位源端与接收端是否有差异。')
h('综合判断')
p('现有证据支持基本测距和旋转扫描正常，未呈现大量结构错误或角度断裂；但无法量化损坏概率，也不能保证每个测距点正确。最需要解决的是<b>实机协议与手册及固件接收代码之间的差异</b>，不能把变长直接称为丢字节或硬件损坏。')

page('八  接线与实验边界')
h('接线信息')
p('用户描述转接模块从上到下为5V、GND、TX、RX、NO、NO。以下仅按模块自身收发方向命名解释，不能据此推断雷达插头物理脚序。配套线曾实测可用，应以产品接线定义为准。')
table([['转接模块标注','通常含义与对应'],['5V','电源正极，对应雷达VCC；供电按配套规格'],['GND','地，需与雷达共地'],['TX','模块发送端，通常接雷达RX'],['RX','模块接收端，通常接雷达TX'],['NO  NO','没有确认功能，暂不连接；不能直接认定为GPS脚']],[108,391])
p('手册雷达信号名为VCC、GND、RX、TX、PPS、REC；PPS和REC与GPS同步及信息输入有关，普通串口测距无需这些GPS输入。手册给出的雷达串口为3.3V TTL，5V指供电，不代表TX/RX通信电平。')
h('硬件故障与链路故障尚未被定位')
p('如果只是传输中少了两个字节，通常包内长度字段不会同步由160变为158。现有原始数据却呈现长度字段、真实间距、帧尾及角度连续对应，因此普通随机丢字节与观察结果不吻合。商家关于剔除无效点的说明进一步支持正常变长的解释。')
p('这仍不等于已单独验证转接模块或雷达TX。尚未更换转接模块做受控对比，未测量供电、电平或波特率误差，未用逻辑分析仪直接抓取雷达引脚。不能凭这些记录指定某个器件有故障，也不能给出可靠的损坏概率。')
h('统计与测试判定的边界')
p('10次记录包括9次PASS和1次因提前结束得到的WARN，不能称为10次全部完整通过。没有重新采集2026年9月29日的新实机数据，本次是对既有记录重算和文档复核。164和166字节包仍只有报告计数，不能出具其原始十六进制包。')
p('距离稳定不等于距离绝对准确；前方一个盒子实验也不能证明全角度、全量程和所有材质性能。0距离按当前工具作为无有效测距显示处理，但其具体返回语义应由厂家说明。稀少包不应仅因数量少而被判坏，接收时仍需验证结构、连续性及距离合理性。')
h('文档与代码状态')
p('本报告替代此前总结中尚未纳入商家反馈或表达过强的结论。旧图、旧报告和原始bin保留用于溯源；本次重算了审计与分析文件。没有修改check_m10p.py、厂家原示例或STM32接收层，没有烧录操作。')

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
p('依据资料：M10P_20K_数据输出格式.pdf、M10P_20KHz_用户手册.pdf、厂家M10P-uart-Python3.py，以及本对话中用户对商家“range长度不同正常、雷达剔除无效点”的转述。合法长包范围和完整字段定义尚无正式补充协议。','small')

def footer(c,doc):
    c.setFont('CN',8);c.setFillColor(colors.HexColor('#64748b'))
    c.drawString(48,28,'M10P雷达测试关键结论完整记录  2026年9月29日复核')
    c.drawRightString(W-48,28,str(doc.page))
doc=SimpleDocTemplate(str(OUT),pagesize=(W,H),leftMargin=48,rightMargin=48,topMargin=42,bottomMargin=46,
                      title='M10P雷达测试关键结论完整记录',author='测试记录整理')
doc.build(story,onFirstPage=footer,onLaterPages=footer)
print(OUT)
print('Frames',frames,'totals',dict(sorted(totals.items())),'non160_pct',variable)
