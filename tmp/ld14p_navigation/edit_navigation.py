from pathlib import Path
p=Path('2-LXY传承/LXY传承-ld14p')
def edit(f,fn):
 q=p/f;s=q.read_text(encoding='utf-8-sig');q.write_text(fn(s),encoding='utf-8',newline='\r\n')
header=r'''
/* ===== LD14P 导航配置：长度为毫米，速度仍为原编码器单位 =====
 * 车宽/前悬尚待实车标定；未标定前只减速停车，不自动绕行。
 * 缓存不是整圈快照：每个角度单独限龄，缺测永远不表示空旷。 */
#define NAV_CACHE_US 200000u
#define NAV_HALF_WIDTH_MM 200.0f
#define NAV_MARGIN_MM 50.0f
#define NAV_STOP_MM 400.0f
#define NAV_LOOK_MM 1000.0f
#define NAV_ALLOW_AVOID 0 /* 确认车宽、前悬及停止距离后才可改为1 */
#define NAV_CLEAR 0u
#define NAV_UNKNOWN 1u
#define NAV_OBSTACLE 2u
#define NAV_AVOID_LEFT 3u
#define NAV_AVOID_RIGHT 4u
typedef struct {
    uint16_t front_bins, left_bins, right_bins, max_age_ms;
    uint32_t left_stamp, right_stamp;
    float obstacle_y, obstacle_x;
    uint8_t action; /* 上述NAV_*；0不是“没有数据”，而是前方覆盖足够 */
} NavigationObservation;
extern NavigationObservation Navigation;
void LEIDA_ScanReset(void);
void LEIDA_ScanUpdate(const _LEIDA_DATA *raw, uint16_t count, uint32_t input_us);
uint16_t LEIDA_ScanSnapshot(_LEIDA_DATA *out, uint32_t now);
void LEIDA_InspectNavigation(uint32_t now);
'''
edit('HARDWARE/LEIDA_DATA/LEIDA_DATA.h',lambda s:s.replace('/* ============ 雷达数据处理流水线 ============ */',header+'\n/* ============ 雷达数据处理流水线 ============ */'))
nav=r'''
/* ===== 带年龄的角度缓存与独立近障碍检测 ===== */
typedef struct {
    _LEIDA_DATA point;
    float x, y;             /* 转换一次，多处复用 */
    uint32_t measured_us;   /* 用包内扫描角差估算；不是主机接收时刻 */
    uint8_t seen;
} NavigationBin;
static NavigationBin navigation_bins[360];
NavigationObservation Navigation;

void LEIDA_ScanReset(void)
{
    memset(navigation_bins, 0, sizeof navigation_bins);
    memset(&Navigation, 0, sizeof Navigation);
    Navigation.action = NAV_UNKNOWN;
}

void LEIDA_ScanUpdate(const _LEIDA_DATA *raw, uint16_t count, uint32_t input_us)
{
    uint16_t i, bin;
    float last_angle, delta;
    uint32_t measured;
    /* 异常转速下不能可靠估计点年龄，宁可清空，也不沿用旧空间。 */
    if (!count || LEIDA_speed_dps < 1500 || LEIDA_speed_dps > 3000) {
        LEIDA_ScanReset();
        return;
    }
    last_angle = raw[count - 1].angle;
    for (i = 0; i < count; i++) {
        if (!(raw[i].angle >= 0 && raw[i].angle < 360))
            continue;
        bin = (uint16_t)(raw[i].angle + 0.5f) % 360;
        /* 车体坐标角随扫描递减；最末点前的角差对应较早的测量。
         * 额外扣3ms覆盖DMA尾部未完成包；仅作为本机年龄估计。 */
        delta = raw[i].angle - last_angle;
        if (delta < 0) delta += 360;
        measured = input_us - (uint32_t)(delta * 1000000.0f / LEIDA_speed_dps) - 3000u;
        /* 同一角度只保留较新观测；近乎同时的点取最近，避免远背景盖住锥桶。 */
        if (navigation_bins[bin].seen &&
            (uint32_t)(measured - navigation_bins[bin].measured_us) < 2000u &&
            navigation_bins[bin].point.distance >= 100 &&
            navigation_bins[bin].point.distance < raw[i].distance)
            continue;
        navigation_bins[bin].point = raw[i];
        navigation_bins[bin].measured_us = measured;
        navigation_bins[bin].seen = raw[i].distance >= 100 && raw[i].distance <= 4000;
        if (navigation_bins[bin].seen) {
            navigation_bins[bin].x = raw[i].distance * arm_cos_f32(raw[i].angle * PI / 180);
            navigation_bins[bin].y = raw[i].distance * arm_sin_f32(raw[i].angle * PI / 180);
        }
    }
}

static uint8_t navigation_fresh(uint16_t bin, uint32_t now)
{
    return navigation_bins[bin].seen &&
           (uint32_t)(now - navigation_bins[bin].measured_us) <= NAV_CACHE_US;
}

uint16_t LEIDA_ScanSnapshot(_LEIDA_DATA *out, uint32_t now)
{
    uint16_t i, count = 0;
    for (i = 0; i < 360; i++)
        if (navigation_fresh(i, now))
            out[count++] = navigation_bins[i].point;
    return count;
}

void LEIDA_InspectNavigation(uint32_t now)
{
    uint16_t i, age, left_free = 0, right_free = 0;
    uint32_t left_age = 0xffffffffu, right_age = 0xffffffffu;
    float closest = NAV_LOOK_MM + 1, x, y;
    uint8_t clustered;
    memset(&Navigation, 0, sizeof Navigation);
    for (i = 0; i < 360; i++) {
        if (!navigation_fresh(i, now)) continue;
        age = (uint16_t)((now - navigation_bins[i].measured_us) / 1000u);
        if (age > Navigation.max_age_ms) Navigation.max_age_ms = age;
        if (i >= 70 && i <= 110) Navigation.front_bins++;
        if (i >= 105 && i <= 180) {
            Navigation.left_bins++;
            if (now - navigation_bins[i].measured_us < left_age) {
                left_age = now - navigation_bins[i].measured_us;
                Navigation.left_stamp = navigation_bins[i].measured_us;
            }
        }
        if (i <= 75) {
            Navigation.right_bins++;
            if (now - navigation_bins[i].measured_us < right_age) {
                right_age = now - navigation_bins[i].measured_us;
                Navigation.right_stamp = navigation_bins[i].measured_us;
            }
        }
        x = navigation_bins[i].x;
        y = navigation_bins[i].y;
        if (y < 100 || y > NAV_LOOK_MM || fabs(x) > NAV_HALF_WIDTH_MM + NAV_MARGIN_MM)
            continue;
        /* 两个相邻角度形成小簇；400mm以内单个可信回波也立即停车。
         * 这里不做“远点数量足够就清空近点”的判断。 */
        clustered = y <= NAV_STOP_MM;
        if (i > 0 && navigation_fresh(i - 1, now) &&
            fabs(navigation_bins[i - 1].x - x) < 120 &&
            fabs(navigation_bins[i - 1].y - y) < 120)
            clustered = 1;
        if (i < 359 && navigation_fresh(i + 1, now) &&
            fabs(navigation_bins[i + 1].x - x) < 120 &&
            fabs(navigation_bins[i + 1].y - y) < 120)
            clustered = 1;
        if (clustered && y < closest) {
            closest = y;
            Navigation.obstacle_x = x;
            Navigation.obstacle_y = y;
        }
    }
    Navigation.action = Navigation.front_bins >= 29 ? NAV_CLEAR : NAV_UNKNOWN;
    if (closest > NAV_LOOK_MM) return;
    Navigation.action = NAV_OBSTACLE;
    /* 自动绕行默认关闭：未经车体尺寸标定，不能把“有空隙”当成“车能通过”。 */
    if (!NAV_ALLOW_AVOID || closest <= NAV_STOP_MM) return;
    for (i = 55; i <= 125; i++) {
        if (!navigation_fresh(i, now) || navigation_bins[i].y < closest + 400) continue;
        if (i <= 80) right_free++;
        if (i >= 100) left_free++;
    }
    if (left_free >= 24 && Navigation.left_bins >= 55)
        Navigation.action = NAV_AVOID_LEFT;
    else if (right_free >= 24 && Navigation.right_bins >= 55)
        Navigation.action = NAV_AVOID_RIGHT;
}
/* ===== 导航缓存结束 ===== */
'''
edit('HARDWARE/LEIDA_DATA/LEIDA_DATA.c',lambda s:s+'\n'+nav)
# 前墙提取不再使用远背景提前清空；近障碍由独立通道保留。
def forward(s):
 a=s.index('    /* 空旷检测:',s.index('uint16_t LEIDA_DATA_HANDLE5('));b=s.index('    /* 前方有墙:',a)
 s=s[:a]+'    /* 远背景与近锥桶可以同时存在，禁止用远点数量否定近障碍。 */\n'+s[b:]
 s=s.replace('i < size - 1; i++','i < size; i++')
 return s
edit('HARDWARE/LEIDA_DATA/LEIDA_DATA.c',forward)
