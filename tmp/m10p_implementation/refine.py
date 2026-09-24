from pathlib import Path
import re
R=Path(__file__).resolve().parents[2]; P=R/'2-LXY传承/LXY传承-m10p'
def edit(rel, f):
    p=P/rel; p.write_text(f(p.read_text(encoding='utf-8-sig')),encoding='utf-8')
def func(t,name,new):
    m=re.search(r'^\w+\s+'+name+r'\([^;]*?\)\s*\{',t,re.M); assert m,name
    a=t.index('{',m.start()); i=a+1; depth=1
    while depth: depth+=(t[i]=='{')-(t[i]=='}'); i+=1
    return t[:m.start()]+new+t[i:]
def geometry(t):
    # One linear pass and fixed 181 angle bins: deterministic nearest point,
    # circular zero handling, explicit output capacity.
    start=t.index('uint16_t LEIDA_DATA_HANDLE6(')
    helper='''static uint16_t boundary_extract(_LEIDA_DATA *out, _LEIDA_DATA *in, uint16_t count, uint8_t left)
{
    static int16_t nearest[181];
    uint16_t i, n = 0, span;
    if (count > LEIDA_DATA_COUNTER) count = LEIDA_DATA_COUNTER;
    span = BLUE_ANGLE_LEFT_RIGHT < 0 ? 0 : BLUE_ANGLE_LEFT_RIGHT > 90 ? 90 : (uint16_t)BLUE_ANGLE_LEFT_RIGHT;
    for (i = 0; i <= 180; ++i) nearest[i] = -1;
    for (i = 0; i < count; ++i) {
        int degree = (int)(in[i].angle + 0.5f);
        float diff;
        if (degree == 360) degree = 0;
        if (degree < 0 || degree > 180 || in[i].distance < 100 || in[i].distance > 4000) continue;
        diff = fabsf(in[i].angle - degree);
        if (diff > 180) diff = 360 - diff;
        if (diff <= 0.5f && (nearest[degree] < 0 || in[i].distance < in[nearest[degree]].distance))
            nearest[degree] = (int16_t)i;
    }
    for (i = 0; i <= span && n < LEIDA_DATA_COUNTER / 2; ++i) {
        int16_t ix = nearest[left ? 180-i : i];
        if (ix >= 0) out[n++] = in[ix];
    }
    return n;
}

'''
    t=t[:start]+helper+t[start:]
    t=func(t,'LEIDA_DATA_HANDLE6','''uint16_t LEIDA_DATA_HANDLE6(_LEIDA_DATA out[], _LEIDA_DATA in[], u16 size)
{ return boundary_extract(out, in, size, 1); }''')
    t=func(t,'LEIDA_DATA_HANDLE7','''uint16_t LEIDA_DATA_HANDLE7(_LEIDA_DATA out[], _LEIDA_DATA in[], u16 size)
{ return boundary_extract(out, in, size, 0); }''')
    # Breakpoints require contiguous independent angular support, not array adjacency.
    for name in ['LEIDA_DATA_HANDLE8','LEIDA_DATA_HANDLE9']:
        a=t.index('uint16_t '+name);b=t.index('\n}',a)+2
        q=t[a:b].replace('    for (i = 3; i < size - 4; i++) {','''    if (size < 8 || size > LEIDA_DATA_COUNTER / 2) return 0;
    for (i = 3; i + 4 < size; i++) {
        uint16_t k;
        uint8_t continuous = 1;
        for (k = i - 2; k < i + 3; ++k) {
            float gap = fabsf(arr[k+1].angle - arr[k].angle);
            if (gap < 0.25f || gap > 2.0f) continuous = 0;
        }
        if (!continuous) continue;''')
        # relative criterion prevents distant gradual range changes acting as near corners.
        q=q.replace('>= 400)', '>= fmaxf(400.0f, 0.25f * fminf(arr[i].distance, arr[i+1].distance)))')
        t=t[:a]+q+t[b:]
    a=t.index('uint16_t LEIDA_DATA_HANDLE5(');b=t.index('/* ======================== 工具函数',a)
    q=t[a:b].replace('uint16_t cnt = 0;', 'uint16_t cnt = 0, near_count = 0;')
    q=q.replace('    /* 空旷检测:', '    if (size < 2 || size > LEIDA_DATA_COUNTER) return 0;\n    /* 空旷检测:')
    q=q.replace('for (i = 0; i < size - 1; i++)', 'for (i = 0; i < size; i++)')
    q=q.replace('        if (cnt >= 5)\n            return 0; /* 前方无遮挡，不用前视数据 */', '''        if (fabsf(arr[i].angle - 90) <= 4 && arr[i].distance >= 100 && arr[i].distance <= 1500)
            near_count++;
''')
    q=q.replace('    /* 前方有墙:', '    if (cnt >= 12 && !near_count) return 0;\n    /* 前方有墙:')
    q=q.replace('(arr[i].distance >= 200)', '(arr[i].distance >= 100)')
    q=q.replace('                data[counter]._x', '                if (counter >= 200) return counter;\n                data[counter]._x')
    # Bound configurable scan range and avoid size-1 underflow.
    needle='    for (; start_angle <= end_angle; start_angle += 1) {'
    q=q.replace(needle,'''    if (size < 2 || size > LEIDA_DATA_COUNTER || start_angle < 0 || end_angle > 180 || start_angle > end_angle) return 0;
'''+needle)
    t=t[:a]+q+t[b:]
    return t
edit('HARDWARE/LEIDA_DATA/LEIDA_DATA.c',geometry)
def diag(t):
    t=t.replace('#include "diag_build_id.h"','#include "diag_build_id.h"\n#include "m10p_vehicle.h"')
    t=t.replace('s->target = Speed_mubiao;', 's->target = Radar_effective_target;')
    t=t.replace('frame[97] = 2;', 'frame[97] = 3;')
    t=t.replace('case 4:\n            number = 2;', 'case 4:\n            number = 3;')
    t=t.replace('LD14P nominal4000pts/s@6Hz;PWM97;raw_crc=required;input=DMA-stream','M10P20K 512000 160B/70slots;12Hz;raw_crc=none;input=scan;DMA1024/FIFO8192/raw2048x2/bin720')
    t=t.replace('values[8] = 0;', 'values[8] = M10P_stats.scan_overflow;')
    t=t.replace('values[9] = Diag_input_drop;', 'values[9] = M10P_stats.ready_drop;')
    t=t.replace('values[12] = 0;', 'values[12] = LidarRx_peak;')
    t=t.replace('values[15] = (1u << 5) | (1u << 6) | (1u << 8) | (1u << 12);','values[15] = 0;')
    # Separate sensor stats message; existing DETAIL remains control diagnostics.
    at=t.index('void Diag_Poll(void)')
    extension='''static void m10p_health(uint32_t now)
{
    static uint32_t previous;
    uint32_t v[24];
    uint8_t i;
    if (Diag_mode != 3 || now - previous < 500u || !BLE_NormalSpace()) return;
    previous = now;
    v[0]=1; v[1]=M10P_stats.packets; v[2]=M10P_stats.bad_length; v[3]=M10P_stats.bad_tail;
    v[4]=M10P_stats.bad_angle; v[5]=M10P_stats.bad_speed; v[6]=M10P_stats.invalid_slots;
    v[7]=M10P_stats.high_reflect; v[8]=M10P_stats.discontinuities; v[9]=M10P_stats.scans;
    v[10]=M10P_stats.rejected; v[11]=M10P_stats.scan_overflow; v[12]=M10P_stats.ready_drop;
    v[13]=LidarRx_epoch; v[14]=LidarRx_blocks; v[15]=LidarRx_peak; v[16]=LidarRx_late;
    v[17]=LidarRx_copy_max_cycles; v[18]=M10P_control_seq;
    v[19]=M10P_control_seq ? Diag_TimeUs()-M10P_control_front_us : 0xffffffffu;
    v[20]=M10P_front_bins; v[21]=M10P_left_bins; v[22]=M10P_right_bins;
    v[23]=(uint32_t)M10P_clearance_mm;
    header(7,112);
    for(i=0;i<24;i++)put32(frame+14+4*i,v[i]);
    BLE_Queue(frame,112,0);
}
'''
    t=t[:at]+extension+t[at:]
    t=t.replace('    config_poll();\n    motor_poll();','    config_poll();\n    motor_poll();\n    m10p_health(now);')
    return t
edit('HARDWARE/hc-05/ble_diag.c',diag)
edit('HARDWARE/hc-05/ble_diag.h',lambda t:t.replace('#define DIAG_LIDAR_ID 1u','#define DIAG_LIDAR_ID 3u').replace('#define DIAG_INPUT_KIND 0u','#define DIAG_INPUT_KIND 1u'))
def pc(t):
    fields='schema packets bad_length bad_tail bad_angle bad_speed invalid_slots high_reflect discontinuities scans rejected scan_overflow ready_drop epoch rx_blocks rx_peak rx_late copy_max_cycles scan_seq front_age_us front_bins left_bins right_bins clearance_mm'
    t=t.replace('def detail_key(row):',f'M10P_FIELDS = "{fields}".split()\n\ndef detail_key(row):')
    t=t.replace('        elif kind==2:', '''        elif kind==7:
            if n!=112: raise ValueError("M10P health length")
            msg.update(zip(M10P_FIELDS,struct.unpack('<24I',payload)))
            if msg['schema']!=1: raise ValueError("M10P schema")
            msg['raw_crc_available']=False
        elif kind==2:''')
    t=t.replace("('.motor.csv',motors,", "('.m10p.csv',[m for m in parser.v2.messages if m['type']==7],['session','tx_seq','host_t']+M10P_FIELDS),\n        ('.motor.csv',motors,")
    t=t.replace('radar_crc_bad=sum(m[\'radar_crc_bad\'] for m in details),',"radar_crc_bad=sum(m['radar_crc_bad'] for m in details) if any(r['lidar_id']==1 for r in rows) else None,")
    return t
edit('上位机/telemetry_v2.py',pc)
# PI actual telemetry (the old auxiliary Speed_PID had logging, active loop didn't).
def pi(t):
    a=t.index('float PID_realize('); before,body=t[:a],t[a:]
    body=body.replace('    /* 输出限幅 [0, 100] */','    Diag_motor_integral = speed_pid->err_sum;\n    Diag_motor_prelimit = moto_pwm;\n    /* 输出限幅 [0, 100] */')
    body=body.replace('    pid->err_sum = pid->err = pid->err_l = 0;','    pid->err_sum = pid->err = pid->err_l = 0;\n    Diag_motor_integral = Diag_motor_prelimit = 0;')
    return before+body
edit('HARDWARE/CENTRE_LINE/CENTRE_LINE.c',pi)
print('geometry and versioned sensor diagnostics integrated')
