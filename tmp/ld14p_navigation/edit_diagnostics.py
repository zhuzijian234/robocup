from pathlib import Path
p=Path('2-LXY传承/LXY传承-ld14p')
def edit(f,fn):
 q=p/f;s=q.read_text(encoding='utf-8-sig');q.write_text(fn(s),encoding='utf-8',newline='\r\n')
edit('HARDWARE/hc-05/ble_diag.h',lambda s:s.replace('extern float Diag_detail_f[16];','extern float Diag_detail_f[16];\nextern uint32_t Diag_nav_u[10]; /* DETAIL schema2：拒绝、候选、覆盖、限速原因 */'))
def diag(s):
 s=s.replace('float Diag_detail_f[16];','float Diag_detail_f[16];\nuint32_t Diag_nav_u[10];')
 s=s.replace('s->target = Speed_mubiao;', 's->target = Speed_effective;')
 s=s.replace('    memset(Diag_detail_f, 0, sizeof Diag_detail_f);','    memset(Diag_detail_f, 0, sizeof Diag_detail_f);\n    memset(Diag_nav_u, 0, sizeof Diag_nav_u);')
 s=s.replace('    Diag_detail_u[0] = 1;', '    Diag_detail_u[0] = 2; /* 保留旧160B字段，末尾追加导航诊断 */')
 s=s.replace('header(5, 176);','header(5, 232);')
 s=s.replace('    BLE_Queue(frame, 176, 0);','''    Diag_nav_u[3] = Navigation.action;
    Diag_nav_u[4] = Navigation.front_bins;
    Diag_nav_u[5] = Navigation.left_bins;
    Diag_nav_u[6] = Navigation.right_bins;
    Diag_nav_u[7] = Navigation.max_age_ms;
    Diag_nav_u[8] = Radar_limit_reason;
    for (i = 0; i < 10; i++) put32(frame + 174 + i * 4, Diag_nav_u[i]);
    memcpy(frame + 214, &Speed_mubiao, 4);
    { float effective = Speed_effective; memcpy(frame + 218, &effective, 4); }
    memcpy(frame + 222, &Navigation.obstacle_x, 4);
    memcpy(frame + 226, &Navigation.obstacle_y, 4);
    BLE_Queue(frame, 232, 0);''')
 s=s.replace('(TIM3->CCR1 == SERVO_PWM_MAX)', '(TIM3->CCR1 > SERVO_PWM_MID)')
 s=s.replace('Diag_Field(19, Speed_mubiao, 1);','Diag_Field(19, Speed_effective, 1);')
 s=s.replace('BLE_Queue(frame, 108, 0);','BLE_Queue(frame, 108, 1);')
 s=s.replace('    motor_poll();','    /* HEALTH/CONFIG先入队，MOTOR不能挤占它们的发送机会。 */')
 s=s.replace('    if (Diag_mode != 3 || now - last_health < (Radar_stop_latched ? 200u : 1000u))\n        return;', '    if (Diag_mode != 3 || now - last_health < (Radar_stop_latched ? 200u : 1000u)) {\n        motor_poll();\n        return;\n    }')
 s=s.replace('    last_health = now;','    /* 入队成功后才更新时间；队列拥堵时下一轮重试。 */')
 s=s.replace('    BLE_Queue(frame, 80, 1);','    if (BLE_Queue(frame, 80, 2)) last_health = now;\n    motor_poll();')
 s=s.replace('PD=mode-reset-Dcap150;entry=min(75,abs(P));turn=350ms-exit2','PD=wall-fit-Dcap150;turn=reverse2;cache=200ms;speed=effective')
 s=s.replace('center=paired','center=paired;vehicle=250x200;nav=schema2')
 return s
edit('HARDWARE/hc-05/ble_diag.c',diag)
edit('USER/main.c',lambda s:s.replace('            /* 本帧雷达处理完:', '            Diag_nav_u[9] = turn_guard.pending_direction;\n\n            /* 本帧雷达处理完:'))
# 旧帧解码保持兼容，新帧才读取追加字段。
def host(s):
 pos=s.index('MOTOR_FIELDS =')
 s=s[:pos]+'''NAV_U = "reject_reason candidate_mode candidate_pwm navigation_action front_bins left_bins right_bins max_point_age_ms speed_limit_reason pending_direction".split()
NAV_F = "requested_speed effective_speed obstacle_x obstacle_y".split()
'''+s[pos:]
 s=s.replace('if n!=176:raise ValueError("DETAIL length")','if n not in (176,232):raise ValueError("DETAIL length")')
 s=s.replace('if msg[\'schema\']!=1 or msg[\'detail_flags\']&~4095:raise ValueError("DETAIL schema/flags")', '''if (msg['schema'],n) not in ((1,176),(2,232)) or msg['detail_flags']&~4095:
                raise ValueError("DETAIL schema/flags")
            if msg['schema']==2:
                msg.update(zip(NAV_U,struct.unpack_from('<10I',payload,160)))
                msg.update(zip(NAV_F,struct.unpack_from('<4f',payload,200)))''')
 s=s.replace('+DETAIL_U+DETAIL_F)', '+DETAIL_U+DETAIL_F+NAV_U+NAV_F)')
 s=s.replace('    preroll=getattr(parser,"meta",{}).get("pre_roll_s",0)', '''    meta=getattr(parser,"meta",{})
    preroll=meta.get("pre_roll_s",0)
    all_rows=[r for r in parser.records if r.get("version")==2]''')
 s=s.replace('return dict(v2=True,n_frames=len(rows),', '''return dict(v2=True,n_frames=len(rows),all_frames=len(all_rows),
        formal_known="pre_roll_s" in meta,all_actions=dict(Counter(r['action_reason'] for r in all_rows)),
        all_details=sum(m['type']==5 for m in parser.v2.messages),''')
 s=s.replace('CONTROL（正式区间）：', 'CONTROL（统计区间）：')
 s=s.replace('    if dur:lines +=', '''    lines += [f"全文件CONTROL：{result.get('all_frames',result['n_frames'])}；全文件DETAIL：{result.get('all_details',0)}；全文件动作：{result.get('all_actions',{})}",
              "已按协商完成时刻区分正式区间。" if result.get('formal_known') else "缺少协商完成标记；本报告不能称为正式采集区间。"]
    if dur:lines +=''')
 return s
edit('上位机/telemetry_v2.py',host)
# 发送完成立即衔接下一整包，避免主循环计算期间蓝牙闲置。
def tx(s):
 s=s.replace('uint8_t i,limit=priority==2?6:4;', 'uint8_t i,limit=priority==2?6:priority==1?5:4;')
 a=s.index('void BLE_TxPoll(void){');b=s.index('void BLE_TxIRQ(void){',a)
 s=s[:a]+'''/* 仅在关中断或USART6 ISR内调用；不会打断正在发送的完整消息。 */
static void tx_start_next(void)
{
    uint8_t i;
    int8_t best = -1;
    if (active >= 0) return;
    for (i = 0; i < 6; i++)
        if (pool[i].ready && (best < 0 || pool[i].priority > pool[best].priority ||
            (pool[i].priority == pool[best].priority && (int32_t)(pool[i].order - pool[best].order) < 0)))
            best = i;
    if (best < 0) return;
    Diag_Finalize(pool[best].data, pool[best].len);
    position = 0;
    active = best;
    __DMB();
    USART_ITConfig(BSP_BLUETOOTH, USART_IT_TXE, ENABLE);
}
void BLE_TxPoll(void)
{
    static uint8_t connected;
    uint32_t mask;
    Bluetooth_Mode();
    if (!Get_Bluetooth_ConnectFlag()) {
        if (connected) { BLE_FlushPending(); Diag_session = 0; }
        connected = 0;
        return;
    }
    connected = 1;
    mask = __get_PRIMASK();
    __disable_irq();
    tx_start_next();
    __set_PRIMASK(mask);
}
'''+s[b:]
 s=s.replace('USART_ITConfig(BSP_BLUETOOTH,USART_IT_TXE,DISABLE);}\n}', 'USART_ITConfig(BSP_BLUETOOTH,USART_IT_TXE,DISABLE);tx_start_next();}\n}')
 return s
edit('HARDWARE/hc-05/ble_tx.c',tx)
