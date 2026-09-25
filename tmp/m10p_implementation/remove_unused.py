from pathlib import Path
import re
P=Path(__file__).resolve().parents[2]/'2-LXY传承/LXY传承-m10p'
comment=r'/\*(?:(?!\*/)[\s\S])*\*/'
for rel in ['HARDWARE/LEIDA_DATA/LEIDA_DATA.c','HARDWARE/LEIDA_DATA/LEIDA_DATA.h',
            'HARDWARE/LEIDA_TIMER/timer.c','HARDWARE/LEIDA_TIMER/timer.h','USER/main.c']:
 p=P/rel;t=p.read_text(encoding='utf-8-sig')
 # 删除已确认停用的块及紧邻的历史说明，不动其他条件编译分支。
 t=re.sub(r'(?:(?:'+comment+r')\s*)?#if 0[^\n]*\n(?:(?!^#if)[\s\S])*?^#endif[^\n]*\n','',t,flags=re.M)
 t=re.sub(r'\n{3,}','\n\n',t)
 p.write_text(t,encoding='utf-8')
for rel in ['HARDWARE/LEIDA_DATA/LEIDA_DATA.c','HARDWARE/LEIDA_DATA/LEIDA_DATA.h']:
 p=P/rel;t=p.read_text(encoding='utf-8')
 t=re.sub(r'^(?:extern )?volatile uint32_t LEIDA_(?:parse_calls|sync_failures|short_inputs|missing_packets)[^\n]*\n','',t,flags=re.M)
 t=re.sub(r'\A/\*[\s\S]*?\*/','''/**
 * @brief M10P工作点云的几何处理：坐标转换、边界、中线、断点和前方墙面。
 * 字节接收由DMA.c负责；协议解析和整圈所有权由m10p.c负责。
 * m10p_vehicle.c生成LEIDA_DATA2后，本模块仅使用有效点数范围内的数据。
 * 坐标约定：0度向右、90度向前；距离和x/y均以毫米计。
 */''',t,count=1)
 p.write_text(t,encoding='utf-8')
for rel in ['.eide/eide.yml','USER/.eide/eide.yml']:
 p=P/rel;p.write_text(''.join(l for l in p.read_text(encoding='utf-8').splitlines(True) if 'LEIDA_PWM' not in l),encoding='utf-8')
p=P/'USER/Template.uvprojx';t=p.read_text(encoding='utf-8-sig')
t=re.sub(r'\s*<File>\s*<FileName>leida_pwm\.[ch]</FileName>(?:(?!</File>)[\s\S])*</File>','',t)
t=t.replace(';..\\HARDWARE\\LEIDA_PWM','');p.write_text(t,encoding='utf-8')
# 移除旧47字节协议专用测试；M10P协议覆盖由test_m10p.c承担。
p=P/'test/check_control_fixes.py';t=p.read_text(encoding='utf-8')
t=t.replace("Tests parser chunk boundaries/corruption, geometry, PD transitions/direction and", "Tests M10P shared geometry, PD transitions/direction and")
t=re.sub(r'^uint32_t LEIDA_parse_calls[^\n]*\n|^static uint8_t lidar_packet[^\n]*\n|^static uint8_t packet\[47\][^\n]*\n','',t,flags=re.M)
t=t.replace("function(radar, 'angle_nearest'),\n             function(diag, 'Diag_RadarPacket')]", "function(radar, 'angle_nearest')]")
t=t.replace("['LEIDA_ParserReset','LEIDA_DATA_HANDLE1',\n    'LEIDA_DATA_HANDLE10'", "['LEIDA_DATA_HANDLE10'")
a=t.index('    CHECK(Diag_RadarPacket(packet));');b=t.index('    CHECK(LEIDA_DATA_HANDLE10(plane,0)==0);',a)
t=t[:a]+t[b:];p.write_text(t,encoding='utf-8')
# 删除只被旧协议调用的CRC8入口，V2遥测自身的CRC16保持不变。
p=P/'HARDWARE/hc-05/ble_diag.c';t=p.read_text(encoding='utf-8')
m=re.search(r'^uint8_t Diag_RadarPacket\([^;]+\)\s*\{',t,re.M);assert m
pos=t.index('{',m.start())+1;depth=1
while depth:depth+=(t[pos]=='{')-(t[pos]=='}');pos+=1
t=t[:m.start()]+t[pos:]
t=t.replace('Diag_detail_u[19] = LEIDA_sync_failures;', 'Diag_detail_u[19] = 0; /* V2保留的LD14P字段；M10P统计在type 7报告。 */')
t=t.replace('Diag_detail_u[20] = LEIDA_missing_packets;', 'Diag_detail_u[20] = 0;')
p.write_text(t,encoding='utf-8')
p=P/'HARDWARE/hc-05/ble_diag.h';t=p.read_text(encoding='utf-8')
t=re.sub(r'^.*\bDiag_RadarPacket\([^\n]*\n','',t,flags=re.M);p.write_text(t,encoding='utf-8')
