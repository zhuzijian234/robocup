from pathlib import Path
import re, xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[2]
P=ROOT/'2-LXY传承/LXY传承-m10p'
def edit(rel, fn):
    p=P/rel; text=p.read_text(encoding='utf-8-sig'); p.write_text(fn(text),encoding='utf-8')
def main(t):
    t=t.replace('#include "test.h"','#include "test.h"\n#include "m10p_vehicle.h"')
    t=t.replace('STM32F407VET6','STM32F407ZGT6').replace('230400bps','512000bps')
    a=t.index('    uint32_t input_seq,'); b=t.index('    uint8_t forward_fit_ok',a)
    t=t[:a]+'    const M10P_Scan *scan;\n'+t[b:]
    a=t.index('    /* 注意初始化顺序:'); b=t.index('    Servo_Init',a)
    t=t[:a]+'''    /* M10P: PA2 stays USART2_TX; no LD14P TIM9 PWM. */
    M10P_Init();
    uart2_init(M10P_BAUD);
    DMA_Initializes();
    Bluetooth_Init();

'''+t[b:]
    a=t.index('        /* 等待DMA接收完'); b=t.index('            /* ===== 第3步:',a)
    t=t[:a]+'''        M10P_Poll();
        scan = M10P_Acquire();
        if (scan) {
            /* Diagnostic timestamps refer to reception, never parse time. */
            Diag_input_seq = scan->seq;
            Diag_input_us = scan->end_us;
            Diag_input_ms = Diag_TimeMs() - (uint32_t)(Diag_TimeUs() - scan->end_us) / 1000u;
            Diag_Begin(Diag_input_ms, scan->end_us);
            Diag_detail_u[3] = scan->seq;
            telemetry_mode = BLE_MODE_INVALID;
            Servo_PD_valid = 0;
            forward_fit_ok = side_fit_ok = 0;
            danbian_flag = 0;
            parsed_points = scan->count;
            LEIDA_raw_count = scan->count;
            LEIDA_speed_dps = scan->dps;
            valid_couter = M10P_Build(scan, LEIDA_DATA2, LEIDA_DATA_COUNTER);
            Diag_Field(12, valid_couter, 1);
            if (valid_couter <= 20 || !M10P_perception_ok) {
                Radar_invalid_inputs++;
                Radar_Invalidate();
                (void)TurnGuard_Apply(&turn_guard, BLE_MODE_INVALID, 0, 0,
                                      (uint16_t)TIM3->CCR1, Diag_TimeUs(), &turn_held);
                Midline_PD_Reset();
                Diag_Submit(BLE_MODE_INVALID, parsed_points, LEIDA_speed_dps, 0);
                M10P_Release(scan);
                continue;
            }
'''+t[b:]
    t=t.replace('                Radar_ControlCompleted();','''                Radar_Observe(scan->seq, scan->front_us, scan->epoch,
                              M10P_speed_scale * ((pid_select == 0 || pid_select == 5 || pid_select == 7) ? 1.0f : 0.6f));''')
    t=t.replace('                Radar_invalid_inputs++;\n                Midline_PD_Reset();','                Radar_invalid_inputs++;\n                Radar_Invalidate();\n                Midline_PD_Reset();')
    needle='            BLE_Tune_Telemetry(Servo_pd.err, (float)TIM3->CCR1, telemetry_mode);'
    t=t.replace(needle,needle+'\n            M10P_Release(scan);')
    # Drop obsolete unused locals, not user tuning values.
    for s in ['    u32 t = 0;\n','    uint16_t ceshi_cnt = 0;\n','    float jiaodu_piancha;\n','    float qulu_forward;\n','    float qulu_jinduan;\n','    uint16_t tubian = 0;\n']:
        t=t.replace(s,'')
    return t
edit('USER/main.c',main)
def uart(t):
    t=t.replace('#include "usart.h"','#include "usart.h"\n#include "DMA.h"\n#include "ble_diag.h"')
    needle='    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);'
    pos=t.index(needle)
    t=t[:pos]+'''    {
        NVIC_InitTypeDef n;
        n.NVIC_IRQChannel = USART2_IRQn;
        n.NVIC_IRQChannelPreemptionPriority = 0;
        n.NVIC_IRQChannelSubPriority = 0;
        n.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&n);
        USART_ITConfig(USART2, USART_IT_ERR, ENABLE);
    }
'''+t[pos:]
    t+='''
/* Read SR then DR to clear ORE/FE/NE. Loss is declared even if DMA took DR. */
void USART2_IRQHandler(void)
{
    volatile uint32_t status = USART2->SR;
    if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        status = USART2->DR;
        Diag_uart_errors++;
        LidarRx_Fault();
    }
}
'''
    return t
edit('SYSTEM/usart/usart.c',uart)
def timer(t):
    t=t.replace('#include "centre_line.h"','#include "centre_line.h"\n#include "m10p.h"')
    a=t.index('void Radar_ControlCompleted(void)'); b=t.index('/**',a)
    t=t[:a]+'''static volatile uint32_t observation_us, observation_epoch, observation_seq, completed_us;
static volatile float observation_scale;
static volatile uint8_t observation_valid, warmup;
volatile float Radar_effective_target;
void Radar_Invalidate(void)
{
    uint32_t p = __get_PRIMASK(); __disable_irq();
    observation_valid = 0; warmup = 0;
    __set_PRIMASK(p);
}
void Radar_Observe(uint32_t seq, uint32_t front_us, uint32_t epoch, float scale)
{
    uint32_t p = __get_PRIMASK(), now = Diag_TimeUs();
    __disable_irq();
    if (Radar_stop_latched || epoch != LidarRx_epoch ||
        (uint32_t)(now-front_us) > M10P_MAX_AGE_US || seq == observation_seq) {
        observation_valid = 0; warmup = 0;
    } else {
        if (seq != observation_seq + 1u || now - completed_us > M10P_MAX_AGE_US) warmup = 0;
        observation_seq = seq; observation_us = front_us; observation_epoch = epoch;
        completed_us = now;
        observation_scale = scale < 0 ? 0 : scale > 1 ? 1 : scale;
        if (warmup < 3) warmup++;
        observation_valid = warmup >= 3;
        if (observation_valid) { Radar_age_ticks = 0; Radar_started = 1; }
    }
    __set_PRIMASK(p);
}
void Radar_GuardTick(void)
{
    if (Radar_started && !Radar_stop_latched) {
        if (Radar_age_ticks < RADAR_TIMEOUT_TICKS) Radar_age_ticks++;
        if (Radar_age_ticks >= RADAR_TIMEOUT_TICKS) {
            Radar_stop_latched = 1; Radar_timeout_count++;
        }
    }
    if (observation_epoch != LidarRx_epoch ||
        (uint32_t)(Diag_TimeUs()-observation_us) > M10P_MAX_AGE_US) {
        observation_valid = 0; warmup = 0;
    }
}

'''+t[b:]
    t=t.replace('if (!Radar_started || Radar_stop_latched) {','if (!Radar_started || Radar_stop_latched || !observation_valid) {')
    t=t.replace('            moto_pwm = 0;','            Radar_effective_target = 0;\n            Speed_PID_Reset(&Speed_pid);\n            moto_pwm = 0;')
    t=t.replace('moto_pwm = PID_realize(Speed_now, Speed_mubiao, &Speed_pid);','''Radar_effective_target = Speed_mubiao * observation_scale;
            moto_pwm = PID_realize(Speed_now, Radar_effective_target, &Speed_pid);''')
    return t
edit('HARDWARE/LEIDA_TIMER/timer.c',timer)
edit('HARDWARE/LEIDA_TIMER/timer.h',lambda t:t.replace('void Radar_ControlCompleted(void);','void Radar_Invalidate(void);\nvoid Radar_Observe(uint32_t seq, uint32_t front_us, uint32_t epoch, float scale);\nextern volatile float Radar_effective_target;'))
edit('HARDWARE/CENTRE_LINE/CENTRE_LINE.h',lambda t:t.replace('#endif','void Speed_PID_Reset(pid_type *pid);\n#endif'))
def pi(t):
    t=t.replace('    static float err_sum = 0;','')
    a=t.index('float PID_realize(')
    pre=t[:a]; body=t[a:]
    body=re.sub(r'\berr_sum\b','speed_pid->err_sum',body)
    return pre+body+'\nvoid Speed_PID_Reset(pid_type *pid)\n{\n    pid->err_sum = pid->err = pid->err_l = 0;\n}\n'
edit('HARDWARE/CENTRE_LINE/CENTRE_LINE.c',pi)
edit('HARDWARE/LEIDA_DATA/m10p.c',lambda t:t.replace('pending == 2 &&','pending >= 2 &&'))
print('integrated M10P main/UART/guard/PI')
