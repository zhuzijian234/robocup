from pathlib import Path
p=Path('2-LXY传承/LXY传承-ld14p')
def edit(f,fn):
 q=p/f;s=q.read_text(encoding='utf-8-sig');q.write_text(fn(s),encoding='utf-8',newline='\r\n')
helper=r'''
/* 外侧墙使用 x=a*y+b，竖直墙也可拟合；拒绝短小锥桶簇和过远外推。 */
static uint8_t wall_reference(_LEIDA_DATA_plane *points, uint16_t start, uint16_t end,
                              uint16_t mode, float target, float *reference)
{
    uint16_t i, n = 0;
    float sx = 0, sy = 0, yy = 0, xy = 0, residual = 0;
    float low = FLT_MAX, high = -FLT_MAX, a, b, x, y;
    for (i = start; i < end; i++) {
        x = points[i]._x; y = points[i]._y;
        if (!(fabs(x) <= FLT_MAX && y >= 150 && y <= 1800)) continue;
        if ((mode == 1 && x > -80) || (mode == 2 && x < 80)) continue;
        sx += x; sy += y; n++;
        if (y < low) low = y;
        if (y > high) high = y;
    }
    Diag_detail_u[9] = n;
    if (n < 6 || high - low < 250 || target < low - 250 || target > high + 250)
        return 0;
    sx /= n; sy /= n;
    for (i = start; i < end; i++) {
        x = points[i]._x; y = points[i]._y;
        if (!(fabs(x) <= FLT_MAX && y >= 150 && y <= 1800)) continue;
        if ((mode == 1 && x > -80) || (mode == 2 && x < 80)) continue;
        yy += (y - sy) * (y - sy); xy += (y - sy) * (x - sx);
    }
    if (yy < 1) return 0;
    a = xy / yy; b = sx - a * sy;
    if (fabs(a) > 2) return 0;
    for (i = start; i < end; i++) {
        x = points[i]._x; y = points[i]._y;
        if (!(fabs(x) <= FLT_MAX && y >= 150 && y <= 1800)) continue;
        if ((mode == 1 && x > -80) || (mode == 2 && x < 80)) continue;
        residual += (x - a * y - b) * (x - a * y - b);
    }
    if (residual / n > 60 * 60) return 0;
    *reference = a * target + b;
    Diag_detail_f[8] = *reference;
    Diag_detail_f[9] = target;
    /* ref_dy现在表示外推距离，0表示目标位于真实覆盖区间内。 */
    Diag_detail_f[10] = target < low ? low - target : target > high ? target - high : 0;
    Diag_detail_u[4] |= 2;
    return 1;
}
'''
def steering(s):
 s=s.replace('uint8_t Servo_PD_valid;','uint8_t Servo_PD_valid;\nuint8_t Servo_reject_reason; /* 0有效，1窗口，2中线，3外墙参考，4非有限数 */')
 s=s.replace('static uint8_t turn_direction(uint16_t mode);',helper+'\nstatic uint8_t turn_direction(uint16_t mode);')
 s=s.replace('    Servo_PD_valid = 0;\n    Diag_detail_u[6]', '    Servo_PD_valid = 0;\n    Servo_reject_reason = 0;\n    Diag_detail_u[6]')
 s=s.replace('        if (mode == 1 || mode == 2)\n            Diag_detail_u[4] |= 8;', '        Servo_reject_reason = 1;\n        if (mode == 1 || mode == 2)\n            Diag_detail_u[4] |= 8;')
 a=s.index('        for (i = start; i < end; i++) {',s.index('} else if (mode == 1 || mode == 2) {'));b=s.index('        /* 期望:',a)
 s=s[:a]+'''        if (!wall_reference(points, start, end, mode, target, &x)) {
            Servo_reject_reason = 3;
            Diag_detail_u[4] |= 8;
            return pd_reject();
        }
'''+s[b:]
 a=s.index('        if (BLUE_Y_STRA_SEL != 1) {');b=s.index('    } else if (mode == 1',a)
 s=s[:a]+'''        /* 保留全局±500保护，不再将正常中线纠偏永久压成±200。
         * 固定前视不许远超实际覆盖，否则应降级而非盲目外推。 */
        if (BLUE_Y_STRA_SEL == 1 && fabs(target - points[end - 1]._y) > 250) {
            Servo_reject_reason = 2;
            return pd_reject();
        }
'''+s[b:]
 # opposite confirmation before expiry clears state; preserve last direction beyond hold expiry.
 needle='    *held = 0;\n    /* unsigned差值'
 repl='''    *held = 0;
    if (!valid) state->pending_direction = 0;
    if (valid && direction && turn_direction(state->mode) &&
        direction != turn_direction(state->mode)) {
        /* 两个独立的侧墙观测才能确认反向；重复缓存不会给确认计数。
         * 待确认只短时保舵，主循环同步限速；紧急障碍绕过此仲裁。 */
        if (state->pending_direction != direction) {
            state->pending_direction = direction;
            state->pending_us = now;
            state->pending_evidence = state->evidence_us;
            *held = 1;
            return state->pwm;
        }
        if (state->evidence_us == state->pending_evidence ||
            (uint32_t)(now - state->pending_us) < 60000u) {
            *held = 1;
            return state->pwm;
        }
        state->pending_direction = 0;
    } else if (valid) state->pending_direction = 0;
    /* unsigned差值'''
 s=s.replace(needle,repl)
 # inactive anchor must be valid when mode starts default0 and pwm0. reject invalid does not update.
 s=s.replace('        state->active = 0;\n        state->straight_frames = 0;\n        return pwm;', '        state->active = 0;\n        state->straight_frames = 0;\n        state->mode = 0;\n        return pwm;')
 s=s.replace('float e = 0, kp, kd, p, d, original_d, output, mag, x = 0, best = FLT_MAX, target, entry = 0;', 'float e = 0, kp, kd, p, d, original_d, output, mag, x = 0, target, entry = 0;')
 # true main PI now uses struct integral so inhibition can reset it.
 a=s.index('float PID_realize(')
 s=s[:a]+'''/* 停驱动/明显降速时调用，避免停车后残留积分重新顶满油门。 */
void Speed_PID_Reset(pid_type *pid)
{
    pid->err_sum = 0;
    pid->err_l = 0;
    Diag_motor_integral = 0;
    Diag_motor_prelimit = 0;
}
float PID_realize(float speed_now, float speed_mubiao, pid_type *speed_pid)
{
    float output, next_sum;
    speed_pid->err = speed_mubiao - speed_now;
    next_sum = speed_pid->err_sum + speed_pid->err;
    if (next_sum > 200) next_sum = 200;
    if (next_sum < -200) next_sum = -200;
    output = speed_pid->kp * speed_pid->err + speed_pid->ki * next_sum +
             speed_pid->kd * (speed_pid->err - speed_pid->err_l);
    /* 饱和时只允许有助于退出饱和的积分，避免长期贴墙仍积累驱动力。 */
    if (!((output > 100 && speed_pid->err > 0) || (output < 0 && speed_pid->err < 0)))
        speed_pid->err_sum = next_sum;
    speed_pid->err_l = speed_pid->err;
    Diag_motor_integral = speed_pid->err_sum;
    Diag_motor_prelimit = output;
    if (output > 100) output = 100;
    if (output < 0) output = 0;
    return output;
}
'''
 return s
edit('HARDWARE/CENTRE_LINE/CENTRE_LINE.c',steering)
edit('HARDWARE/CENTRE_LINE/CENTRE_LINE.h',lambda s:s.replace('extern uint8_t Servo_PD_valid;', 'extern uint8_t Servo_PD_valid;\nextern uint8_t Servo_reject_reason;\nvoid Speed_PID_Reset(pid_type *pid);').replace('    uint32_t observed_us;', '    uint32_t observed_us;\n    uint32_t evidence_us, pending_evidence, pending_us; /* 独立观测时间，均支持回绕 */\n    uint8_t pending_direction; /* 0无，1右，2左；等待期间必须限速 */'))
# 实车宽度约250mm，加入每侧50mm余量。
edit('HARDWARE/LEIDA_DATA/LEIDA_DATA.h',lambda s:s.replace('NAV_HALF_WIDTH_MM 200.0f','NAV_HALF_WIDTH_MM 125.0f'))
