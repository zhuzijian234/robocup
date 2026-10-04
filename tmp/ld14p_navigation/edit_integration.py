from pathlib import Path
p=Path('2-LXY传承/LXY传承-ld14p')
def edit(f,fn):
 q=p/f;s=q.read_text(encoding='utf-8-sig');q.write_text(fn(s),encoding='utf-8',newline='\r\n')
edit('HARDWARE/LEIDA_DATA/LEIDA_DATA.h',lambda s:s.replace('车宽/前悬尚待实车标定；未标定前只减速停车，不自动绕行。','用户测量：含轮车宽约250mm、雷达到车头180～200mm，取200mm。').replace('#define NAV_ALLOW_AVOID 0 /* 确认车宽、前悬及停止距离后才可改为1 */','#define NAV_FRONT_MM 200.0f\n#define NAV_ALLOW_AVOID 1 /* 已按用户尺寸配置；仍须低速验证实际转角/停止距离 */').replace('float obstacle_y, obstacle_x;', 'float obstacle_y, obstacle_x, obstacle_width;'))
def nav(s):
 s=s.replace('uint16_t i, age, left_free = 0, right_free = 0;', 'uint16_t i, age, nearest_bin = 0, lo, hi;')
 s=s.replace('            closest = y;\n            Navigation.obstacle_x', '            closest = y;\n            nearest_bin = i;\n            Navigation.obstacle_x')
 a=s.index('    /* 自动绕行默认关闭:') if '    /* 自动绕行默认关闭:' in s else s.index('    /* 自动绕行默认关闭：')
 b=s.index('\n}\n/* ===== 导航缓存结束',a)
 s=s[:a]+'''    /* 扩展相邻回波簇，宽墙交给弯道规划；小目标才尝试绕行。 */
    lo = hi = nearest_bin;
    while (lo > 0 && navigation_fresh(lo - 1, now) &&
           fabs(navigation_bins[lo - 1].x - navigation_bins[lo].x) < 120 &&
           fabs(navigation_bins[lo - 1].y - navigation_bins[lo].y) < 120) lo--;
    while (hi < 359 && navigation_fresh(hi + 1, now) &&
           fabs(navigation_bins[hi + 1].x - navigation_bins[hi].x) < 120 &&
           fabs(navigation_bins[hi + 1].y - navigation_bins[hi].y) < 120) hi++;
    Navigation.obstacle_width = fabs(navigation_bins[hi].x - navigation_bins[lo].x);
    if (!NAV_ALLOW_AVOID || closest <= NAV_STOP_MM || Navigation.obstacle_width > 250) return;
    /* 左右分别验证一条带车宽余量的折线路径，未知扇区直接否决。 */
    x = Navigation.obstacle_x + NAV_HALF_WIDTH_MM + NAV_MARGIN_MM +
        Navigation.obstacle_width * 0.5f + 50;
    if (navigation_path_clear(x, closest, now))
        Navigation.action = NAV_AVOID_RIGHT;
    else if (navigation_path_clear(Navigation.obstacle_x - NAV_HALF_WIDTH_MM - NAV_MARGIN_MM -
                                  Navigation.obstacle_width * 0.5f - 50, closest, now))
        Navigation.action = NAV_AVOID_LEFT;'''+s[b:]
 pos=s.index('void LEIDA_InspectNavigation(')
 helper='''/* 检查目标前后扫掠走廊。此处是低速几何可行性，不替代实车转角标定。 */
static uint8_t navigation_path_clear(float lateral, float target_y, uint32_t now)
{
    uint16_t i, bin;
    float y, x, angle, reach, path_x;
    if (fabs(lateral) > 450 || target_y < 450) return 0;
    /* 每100mm采样中心线及左右边缘；所有方向均须有更新的远端回波。 */
    for (y = 200; y <= target_y + NAV_FRONT_MM; y += 100) {
        path_x = lateral * (y < target_y ? y / target_y : 1);
        for (i = 0; i < 3; i++) {
            x = path_x + ((int)i - 1) * (NAV_HALF_WIDTH_MM + NAV_MARGIN_MM);
            angle = atan2f(y, x) * 180 / PI;
            bin = (uint16_t)(angle + 0.5f);
            reach = sqrtf(x * x + y * y);
            if (!navigation_fresh(bin, now) || navigation_bins[bin].point.distance < reach + 50)
                return 0;
        }
    }
    /* 同时检查走廊内部，防止只查三条射线漏掉窄锥桶。 */
    for (i = 0; i < 180; i++) {
        if (!navigation_fresh(i, now)) continue;
        y = navigation_bins[i].y;
        if (y < 100 || y > target_y + NAV_FRONT_MM) continue;
        path_x = lateral * (y < target_y ? y / target_y : 1);
        if (fabs(navigation_bins[i].x - path_x) < NAV_HALF_WIDTH_MM + NAV_MARGIN_MM)
            return 0;
    }
    return 1;
}

'''
 return s[:pos]+helper+s[pos:]
edit('HARDWARE/LEIDA_DATA/LEIDA_DATA.c',nav)
# 限速发布只在临界区更新；TIM5仍是唯一电机输出方。
edit('HARDWARE/LEIDA_TIMER/timer.h',lambda s:s.replace('void Radar_ControlCompleted(void);','''void Radar_ControlCompleted(void);
extern volatile float Speed_effective; /* 真正送入PI的速度，Speed_mubiao仍是用户请求 */
extern volatile uint8_t Radar_limit_reason;
void Radar_SetSpeedLimit(float limit, uint8_t reason);'''))
edit('HARDWARE/LEIDA_TIMER/timer.c',lambda s:s.replace('volatile uint16_t Radar_age_ticks = 0;', '''volatile float Speed_effective = 0;
static volatile float navigation_speed_limit = 0;
volatile uint8_t Radar_limit_reason = 1;
/* reason：0正常，1感知不足，2障碍，3换向确认，4弯道，5旧观测超时。 */
void Radar_SetSpeedLimit(float limit, uint8_t reason)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    navigation_speed_limit = limit;
    Radar_limit_reason = reason;
    __set_PRIMASK(mask);
}

volatile uint16_t Radar_age_ticks = 0;''').replace('        Radar_GuardTick();','''        Radar_GuardTick();
        /* 无新有效控制时100ms开始限速，200ms撤驱动；500ms锁停仍保留。 */
        {
            float next = Speed_mubiao < navigation_speed_limit ? Speed_mubiao : navigation_speed_limit;
            if (Radar_age_ticks >= 10 && next > 4) next = 4;
            if (Radar_age_ticks >= 20) { next = 0; Radar_limit_reason = 5; }
            if (next < 0) next = 0;
            if (next < Speed_effective - 0.5f) Speed_PID_Reset(&Speed_pid);
            Speed_effective = next;
        }''').replace('if (!Radar_started || Radar_stop_latched) {','if (!Radar_started || Radar_stop_latched || Speed_effective <= 0) {').replace('            moto_pwm = 0;', '            Speed_PID_Reset(&Speed_pid);\n            moto_pwm = 0;').replace('PID_realize(Speed_now, Speed_mubiao, &Speed_pid)', 'PID_realize(Speed_now, Speed_effective, &Speed_pid)'))
# 接入主流程：缓存→几何→仲裁→限速→遥测。
def main(s):
 s=s.replace('Speed_mubiao = 10;', 'Speed_mubiao = 8;')
 s=s.replace('            if (last_input_seq && input_seq - last_input_seq != 1u)\n                LEIDA_ParserReset();','''            if (last_input_seq && input_seq - last_input_seq != 1u) {
                LEIDA_ParserReset();
                LEIDA_ScanReset(); /* 输入缺块后不拼接旧空间 */
            }''')
 s=s.replace('valid_couter = parsed_points ? LEIDA_DATA_HANDLE3_2(LEIDA_DATA2, LEIDA_DATA, LEIDA_DATA_COUNTER) : 0;', '''LEIDA_ScanUpdate(LEIDA_DATA, parsed_points, input_us);
            valid_couter = LEIDA_ScanSnapshot(LEIDA_DATA2, Diag_TimeUs());
            LEIDA_InspectNavigation(Diag_TimeUs());''')
 s=s.replace('                Radar_invalid_inputs++;','                Radar_SetSpeedLimit(0, 1);\n                Radar_invalid_inputs++;',1)
 s=s.replace('                    count = LEIDA_DATA_HANDLE10(boundary, count);','                    /* 使用整段外墙，让PD按物理y跨度与残差验证参考。 */')
 s=s.replace('ref_start = count >= 10 ? (uint16_t)(count * 0.75f) : 0;','ref_start = 0;').replace('ref_end = count >= 10 ? (uint16_t)(count * 0.95f) : count;','ref_end = count;')
 s=s.replace('            candidate_pwm = TurnGuard_Apply(','''            turn_guard.evidence_us = (pid_select == 1 || pid_select == 3 || pid_select == 8)
                                       ? Navigation.left_stamp : Navigation.right_stamp;
            Diag_nav_u[0] = Servo_reject_reason;
            Diag_nav_u[1] = pid_select;
            Diag_nav_u[2] = candidate_pwm; /* 保留仲裁前的原始候选 */
            candidate_pwm = TurnGuard_Apply(''')
 # Override must before final write block to maintain only one servo write.
 needle='            if (!Servo_PD_valid)\n                telemetry_mode = BLE_MODE_INVALID;'
 s=s.replace(needle,'''            /* 感知异常立刻限速，不能靠旧舵角维持原速到500ms才锁停。 */
            if (!Servo_PD_valid || Navigation.action == NAV_UNKNOWN) {
                Radar_SetSpeedLimit(0, 1);
            } else if (Navigation.action == NAV_AVOID_LEFT || Navigation.action == NAV_AVOID_RIGHT) {
                candidate_pwm = Navigation.action == NAV_AVOID_LEFT ? SERVO_PWM_MID + 150 : SERVO_PWM_MID - 150;
                Radar_SetSpeedLimit(4, 2);
                /* 独立避障优先于循迹，不让旧弯道锚点在绕行结束后再次拉回。 */
                memset(&turn_guard, 0, sizeof turn_guard);
                turn_held = 0;
                Midline_PD_Reset();
                Servo_PD_valid = 1;
                Diag_detail_u[4] &= ~1u;
            } else if (Navigation.action == NAV_OBSTACLE) {
                /* 远处宽墙可以是弯道外墙；近墙或窄障碍必须撤驱动。 */
                if (Navigation.obstacle_width > 250 && Navigation.obstacle_y > NAV_STOP_MM &&
                    (pid_select == 1 || pid_select == 2 || pid_select == 3 || pid_select == 4 || pid_select == 8 || pid_select == 9))
                    Radar_SetSpeedLimit(4, 2);
                else Radar_SetSpeedLimit(0, 2);
            } else if (turn_guard.pending_direction) {
                Radar_SetSpeedLimit((uint32_t)(Diag_TimeUs() - turn_guard.pending_us) < 200000u ? 4 : 0, 3);
            } else {
                Radar_SetSpeedLimit((pid_select == 1 || pid_select == 2 || pid_select == 3 || pid_select == 4 || pid_select == 8 || pid_select == 9) ? 6 : 8, 4);
            }
            if (!Servo_PD_valid)
                telemetry_mode = BLE_MODE_INVALID;''')
 s=s.replace('telemetry_mode = turn_held ? BLE_MODE_HOLD : pid_select;', '''telemetry_mode = (Navigation.action == NAV_AVOID_LEFT || Navigation.action == NAV_AVOID_RIGHT)
                                 ? BLE_MODE_FORCED : turn_held ? BLE_MODE_HOLD : pid_select;''')
 return s
edit('USER/main.c',main)
