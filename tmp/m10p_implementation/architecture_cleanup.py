from pathlib import Path
import re
P=Path(__file__).resolve().parents[2]/'2-LXY传承/LXY传承-m10p'

def wrap_functions(relative,names,reason):
    path=P/relative;t=path.read_text(encoding='utf-8-sig')
    for name in names:
        m=re.search(r'^(?:static )?\w+\s+'+name+r'\([^;]*?\)\s*\{',t,re.M)
        assert m,name
        pos=t.index('{',m.start())+1;depth=1
        while depth:
            depth+=(t[pos]=='{')-(t[pos]=='}');pos+=1
        t=t[:m.start()]+'#if 0 /* '+reason+' */\n'+t[m.start():pos]+'\n#endif /* M10P不编译上述历史实现 */'+t[pos:]
    path.write_text(t,encoding='utf-8')

wrap_functions('HARDWARE/LEIDA_DATA/LEIDA_DATA.c',[
    'LEIDA_ParserReset','LEIDA_DATA_HANDLE1','LEIDA_DATA_HANDLE3','LEIDA_DATA_HANDLE3_2',
    'LEIDA_ANGLE_jiuzheng','LEIDA_DATA_HANDLE12','LEIDA_DATA_HANDLE13',
    'LEIDA_PrintAll','LEIDA_PrintSample','LEIDA_PrintHead'],
    '历史参考代码：M10P运行链路无调用，禁止参与固件编译')
wrap_functions('HARDWARE/LEIDA_TIMER/timer.c',['TIM14_Int_Init'],
    '未使用的预留定时器；M10P只使用TIM5速度环与TIM6诊断时钟')
p=P/'HARDWARE/LEIDA_DATA/LEIDA_DATA.h';t=p.read_text(encoding='utf-8-sig')
names=['LEIDA_ParserReset','LEIDA_DATA_HANDLE1','LEIDA_DATA_HANDLE3','LEIDA_DATA_HANDLE3_2',
       'LEIDA_ANGLE_jiuzheng','LEIDA_DATA_HANDLE12','LEIDA_DATA_HANDLE13',
       'LEIDA_PrintAll','LEIDA_PrintSample','LEIDA_PrintHead']
for name in names:
    t,n=re.subn(r'^(\w+\s+'+name+r'\([^;]+;)',r'#if 0 /* 历史接口已停用，M10P调用新解析/适配层 */\n\1\n#endif',t,flags=re.M)
    assert n==1,name
p.write_text(t,encoding='utf-8')
p=P/'HARDWARE/LEIDA_PWM/leida_pwm.c';t=p.read_text(encoding='utf-8-sig')
p.write_text('#if 0 /* M10P内部驱动电机，不使用LD14P外部PWM；保留源码供旧硬件参考。 */\n'+t+'\n#endif\n',encoding='utf-8')
