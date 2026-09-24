from pathlib import Path
R=Path(__file__).resolve().parents[2];P=R/'2-LXY传承/LXY传承-m10p'
def edit(rel,f):
 p=P/rel;p.write_text(f(p.read_text(encoding='utf-8-sig')),encoding='utf-8')
for rel in ['USER/Template.uvprojx','USER/Template.uvoptx']:
 edit(rel,lambda t:t.replace('STM32F407ZGTx','STM32F407ZG').replace('STM32F407VETx','STM32F407ZG').replace('STM32F407VE$','STM32F407ZG$').replace('STM32F4xx_512','STM32F4xx_1024').replace('-FL080000','-FL0100000').replace('$CMSIS\\Flash\\','$Flash\\'))
for rel,scatter in [('.eide/eide.yml','USER/m10p.sct'),('USER/.eide/eide.yml','m10p.sct')]:
 edit(rel,lambda t:t.replace('STM32F407ZGTx','STM32F407ZG').replace('scatterFilePath: ""',f'scatterFilePath: "{scatter}"').replace('useCustomScatterFile: false','useCustomScatterFile: true').replace('cpuName: "null"','cpuName: STM32F407ZG').replace('vendor: "null"','vendor: STMicroelectronics'))
edit('test/build_m10p.py',lambda t:t.replace("'--map', '--info=sizes'","'--map', '--symbols', '--info=sizes'").replace('existing EIDE scatter','explicit m10p.sct; ZG 1MiB Flash / 128KiB SRAM'))
edit('HARDWARE/CENTRE_LINE/CENTRE_LINE.h',lambda t:t.replace('#define TURN_EXIT_FRAMES 2u','#define TURN_EXIT_FRAMES 2u\n#define TURN_EXIT_MIN_US 100000u').replace('uint32_t observed_us;','uint32_t observed_us, straight_since_us;'))
edit('HARDWARE/CENTRE_LINE/CENTRE_LINE.c',lambda t:t.replace('    state->straight_frames = straight ? state->straight_frames + 1 : 0;','''    if (straight && !state->straight_frames) state->straight_since_us = now;
    state->straight_frames = straight ? (state->straight_frames < 255 ? state->straight_frames + 1 : 255) : 0;''').replace('if (state->straight_frames >= TURN_EXIT_FRAMES)', 'if (state->straight_frames >= TURN_EXIT_FRAMES && (uint32_t)(now-state->straight_since_us) >= TURN_EXIT_MIN_US)'))
edit('HARDWARE/LEIDA_TIMER/timer.h',lambda t:t.replace('void Radar_Invalidate(void);','uint8_t Radar_Permitted(void);\nvoid Radar_Invalidate(void);'))
def timer(t):
 t=t.replace('volatile float Radar_effective_target;','''volatile float Radar_effective_target;
uint8_t Radar_Permitted(void)
{
    return Radar_started && !Radar_stop_latched && observation_valid &&
        observation_epoch == LidarRx_epoch &&
        (uint32_t)(Diag_TimeUs()-observation_us) <= M10P_MAX_AGE_US;
}''')
 t=t.replace('            Radar_effective_target = Speed_mubiao * observation_scale;', '''            {
                float requested = Speed_mubiao * observation_scale;
                /* Native encoder units, 0.25/tick acceleration; deceleration immediate. */
                if (requested > Radar_effective_target + 0.25f) requested = Radar_effective_target + 0.25f;
                Radar_effective_target = requested;
            }''')
 return t
edit('HARDWARE/LEIDA_TIMER/timer.c',timer)
def main(t):
 needle='            straight_evidence = Servo_PD_valid'
 t=t.replace(needle,'''            if (scan->epoch != LidarRx_epoch || (uint32_t)(Diag_TimeUs()-scan->front_us) > M10P_MAX_AGE_US)
                Servo_PD_valid = 0;
'''+needle)
 # Freshness checked separately just before outputs; regression harness mocks
 # scan/RX epoch explicitly rather than bypassing this condition.
 t=t.replace('   1. 等待 DMA_RX_DONE (一帧雷达数据就绪)','   1. 循环DMA收流，M10P按车后切点发布完整扫描')
 t=t.replace('USART2, PA2(TX/被TIM9覆盖)', 'USART2, PA2(TX)')
 t=t.replace('   雷达电机: TIM9 CH1, PA2 (⚠ PA2与USART2_TX共用, 初始化顺序保证TIM9后初始化)','   雷达电机: M10P内部驱动；本工程不初始化TIM9雷达PWM')
 return t
edit('USER/main.c',main)
edit('HARDWARE/hc-05/ble_diag.c',lambda t:t.replace('algorithm=2 input=', 'algorithm=3 input=').replace('motor = Radar_stop_latched ? 3 : Radar_started ? 2','motor = Radar_stop_latched ? 3 : Radar_Permitted() ? 2').replace('Diag_Field(19, Speed_mubiao, 1);','Diag_Field(19, Radar_effective_target, 1);').replace('uint32_t v[24];','uint32_t v[26];').replace('    header(7,112);','    v[24]=Radar_Permitted(); v[25]=(uint32_t)(Radar_effective_target*1000.0f);\n    header(7,120);').replace('for(i=0;i<24;i++)put32(frame+14+4*i,v[i]);','for(i=0;i<26;i++)put32(frame+14+4*i,v[i]);').replace('BLE_Queue(frame,112,0);','BLE_Queue(frame,120,0);').replace('turn=350ms-exit2','turn=350ms-exit2+100ms;spd=effective;request=CFG108'))
edit('上位机/telemetry_v2.py',lambda t:t.replace('right_bins clearance_mm"','right_bins clearance_mm motor_permitted effective_target_milli"').replace('if n!=112: raise ValueError("M10P health length")','if n!=120: raise ValueError("M10P health length")').replace("zip(M10P_FIELDS,struct.unpack('<24I',payload))", "zip(M10P_FIELDS,struct.unpack('<26I',payload))"))
edit('HARDWARE/hc-05/ble_tune.c',lambda t:t.replace('&BLUE_ANGLE_LEFT_RIGHT, 0,  180, 0','&BLUE_ANGLE_LEFT_RIGHT, 0,   90, 0'))
# Adapt old selection test to the added production freshness contract.
edit('test/check_turn_regression.py',lambda t:t.replace('#define BLE_MODE_HOLD 10', '#include "m10p.h"\nstatic M10P_Scan test_scan={0};\nstatic M10P_Scan *scan=&test_scan;\nstatic uint32_t LidarRx_epoch;\n#define BLE_MODE_HOLD 10').replace('selection = main[start:end]', "selection = main[start:end]\n# The old geometry harness advances an artificial clock; keep scan fresh for these tests.\nselection = '            scan->front_us = Diag_TimeUs();\\n' + selection").replace("'/Od',", "'/Od', '/I'+str(HERE.parent/'HARDWARE/LEIDA_DATA'),"))
print('final configuration, guard telemetry and turn timing updated')
