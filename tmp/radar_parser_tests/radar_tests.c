
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef uint8_t u8;
typedef uint16_t u16;
typedef struct { float angle, distance; } _LEIDA_DATA;
#define LEIDA_DATA_COUNTER 800
#define LEIDA_ANGLE_CENTER 90.0f
#define RADAR_TIMEOUT_TICKS 50u
#define SET 1
#define TIM5 5
#define TIM2 2
#define TIM_IT_Update 1
uint16_t LEIDA_speed_dps;
volatile uint16_t LEIDA_raw_count;
volatile uint32_t LEIDA_parse_calls, LEIDA_sync_failures;
volatile uint32_t LEIDA_short_inputs, LEIDA_missing_packets;
uint32_t irq_mask;
#define __get_PRIMASK test_get_primask
#define __disable_irq test_disable_irq
#define __set_PRIMASK test_set_primask
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1; }
void __set_PRIMASK(uint32_t value) { irq_mask = value; }
uint16_t daoche_flag, moto_pwm, motor_ccr;
unsigned pi_calls, encoder_calls;
float Speed_now, Speed_mubiao;
int Speed_pid;
int TIM_GetITStatus(int timer, int flag) { return SET; }
void TIM_ClearITPendingBit(int timer, int flag) {}
void TIM_SetCompare1(int timer, uint16_t v) { motor_ccr = v; }
void Get_Encoder(void) { encoder_calls++; }
float PID_realize(float now, float target, int *pid) { pi_calls++; return 73; }
void Moto_Speed(uint16_t v) { motor_ccr = v; }
uint16_t LEIDA_DATA_HANDLE1(_LEIDA_DATA data[], u8 arr[], u16 size)
{
    int i, j, k;
    int parsed = 0;   /* 本次真正解析写出的数据点数(每命中一个帧头 +12)。bit26"原始点数"用它, 与"47字节步进遍历槽数"区分 */
    float start_angle;
    float end_angle;

    LEIDA_parse_calls++;
    LEIDA_raw_count = 0;
    /* Three header bytes at offsets 0, 47, 94 need at least 95 bytes.
     * Clear on every unsuccessful return; caller owns COUNTER output slots. */
    if (size < 95) {
        LEIDA_short_inputs++;
        for (k = 0; k < LEIDA_DATA_COUNTER; k++) {
            data[k].angle = 0.0f;
            data[k].distance = 0.0f;
        }
        return 0;
    }

    /* 寻找同步模式: 间隔47字节的三个连续0x54帧头 */
    for (i = 0; i + 94 < size; i++) {
        if (arr[i] == 0x54) {
            if (arr[i + 47] == 0x54) {
                if (arr[i + 94] == 0x54) {
                    break;
                }
            }
        }
    }
    if (i + 94 >= size) {
        LEIDA_sync_failures++;
        for (k = 0; k < LEIDA_DATA_COUNTER; k++) {
            data[k].angle = 0.0f;
            data[k].distance = 0.0f;
        }
        return 0;
    }

    /* 解析数据包: 每包47字节 -> 12个数据点 */
    for (j = 0; i + 47 <= size && j + 12 <= LEIDA_DATA_COUNTER; i += 47, j += 12) {
        if (arr[i] == 0x54) {
            start_angle = (((u16)arr[i + 5] << 8) + (u16)arr[i + 4]) / 100.0f;
            end_angle   = (((u16)arr[i + 43] << 8) + (u16)arr[i + 42]) / 100.0f;
            /* 转速: Byte2~3 (低字节在前), 单位 度/秒。只读不影响任何算法 */
            LEIDA_speed_dps = (((u16)arr[i + 3] << 8) + (u16)arr[i + 2]);
            parsed += 12;   /* 只统计真正写出数据的点: 帧头缺失时 47 字节步进跳过的整包不计入 */

            /* 处理角度回绕: 结束角度<起始角度，说明扫描跨过了0度 */
            if (start_angle > end_angle)
                end_angle += 360;
 /*数据是怎么存的

arr[i+7+3*k]              arr[i+6+3*k]
  u8 (1字节)                u8 (1字节)
  例: 0x02                 例: 0x64
     │                        │
     ▼ 强转 (u16)             ▼ 强转 (u16)
  0x0002                   0x0064
  (2字节)                  (2字节)
     │                        │
     ▼ << 8                   │
  0x0200                      │
  (512)                       │
     │                        │
     └───────── + ────────────┘
                 │
                 ▼
          u16: 0x0264 = 612
          (2字节, 最大值 65535)
                 │
                 ▼ × 1.0f
          float: 612.0f
          (4字节, IEEE 754)
                 │
                 ▼
    data[j+k].distance = 612.0f*/
            for (k = 0; k < 12; k++) {
                /* 距离: byte 6+3k, 7+3k (低字节在前) 转为16字节->左移八位,不然溢出,低字节直接转为16字节 */
                data[j + k].distance = 1.0f * (((u16)arr[i + 7 + 3 * k] << 8) + (u16)arr[i + 6 + 3 * k]);
                /*i:包的帧头数组下标,每次+47;j:data数组/包索引,每个包12个数据点;size:字节总数 ,k:,data数组/数据点索引,每3字节一个数据点*/
                /* 角度: 起始和结束之间线性插值 */
                data[j + k].angle = start_angle + (end_angle - start_angle) / 12 * k;

                /* 角度归一化到 [0, 360) */
                if (data[j + k].angle > 360.0f) data[j + k].angle -= 360.0f;
                if (data[j + k].angle < 0.0f)   data[j + k].angle += 360.0f;

                /* 坐标系旋转: 0°->右侧(x+), 90°->前方(y+) ,原本:0°->正前,90°->右侧*/
                data[j + k].angle = -1.0f * data[j + k].angle + 360.0f + LEIDA_ANGLE_CENTER;

                if (data[j + k].angle > 360.0f) data[j + k].angle -= 360.0f;
                if (data[j + k].angle < 0.0f)   data[j + k].angle += 360.0f;
            }
        } else {
            LEIDA_missing_packets++;
            /* 帧头缺失: 这 12 个槽位本帧没有写入, 清零。
             * 不清零的话它们会保留上一帧的旧点, 被 HANDLE3_2 当作有效点计入 valid_couter。 */
            for (k = 0; k < 12; k++) {
                data[j + k].angle    = 0.0f;
                data[j + k].distance = 0.0f;
            }
        }
    }

    /* 块尾未覆盖到的槽位同样清零: 每块的整包数随块相位变化(当前 1798B/47B 时是 37 或 38),
     * 上一帧写过的尾部槽位本帧可能不再被写到, 残留旧点会混进 HANDLE3_2 的筛选结果。
     * 上界用输出容量 LEIDA_DATA_COUNTER；本函数仍仅适用于47字节/12点协议，换雷达必须更换解析器。 */
    for (k = j; k < LEIDA_DATA_COUNTER; k++) {
        data[k].angle    = 0.0f;
        data[k].distance = 0.0f;
    }

    LEIDA_raw_count = (uint16_t)parsed;
    return (uint16_t)parsed;   /* 成功=本次解析出的点数; 未找到有效帧头已在上面返回 0 */
}

uint16_t LEIDA_DATA_HANDLE3_2(_LEIDA_DATA data[], _LEIDA_DATA arr[], u16 size)
{
    int i, j;
    j = 0;
    for (i = 10; i < size - 10; i++) {
        if ((arr[i].distance >= 100)) {
            data[j].angle    = arr[i].angle;
            data[j].distance = arr[i].distance;
            j++;
        }
    }
    return j;
}

volatile uint16_t Radar_age_ticks = 0;
volatile uint8_t Radar_started = 0;
volatile uint8_t Radar_stop_latched = 0;
volatile uint32_t Radar_timeout_count = 0;
volatile uint32_t Radar_invalid_inputs = 0;

void Radar_ControlCompleted(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* Never clear a timeout by receiving another scan. */
    if (!Radar_stop_latched) {
        Radar_age_ticks = 0;
        Radar_started = 1;
    }
    __set_PRIMASK(primask);
}

void Radar_GuardTick(void)
{
    if (Radar_started && !Radar_stop_latched) {
        if (Radar_age_ticks < RADAR_TIMEOUT_TICKS) Radar_age_ticks++;
        if (Radar_age_ticks >= RADAR_TIMEOUT_TICKS) {
            Radar_stop_latched = 1;
            Radar_timeout_count++;
        }
    }
}

void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET) {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

        Radar_GuardTick();
        if (!Radar_started || Radar_stop_latched) {
            /* Zero duty removes propulsion; it is not an active brake.
             * Skip PI so its integral cannot accumulate during inhibition. */
            Get_Encoder();
            moto_pwm = 0;
            Moto_Speed(0);
        } else if (daoche_flag == 1) {
            TIM_SetCompare1(TIM2, (uint16_t)(100 * 0.5));  /* 50%制动,不足以驱动小车 */
        } else {
            Get_Encoder();
            moto_pwm = PID_realize(Speed_now, Speed_mubiao, &Speed_pid);
            Moto_Speed(moto_pwm);
        }
    }
}

/* Result 0 means all checks passed; otherwise result is the failing line. */
volatile int radar_test_result = -1;
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
struct { uint32_t before; _LEIDA_DATA points[800]; uint32_t after; } dst;
_LEIDA_DATA filtered[800];
u8 input[4096];
void stale(void) {
    int i;
    dst.before = 0x12345678; dst.after = 0x87654321;
    for (i = 0; i < 800; ++i) { dst.points[i].angle = 123; dst.points[i].distance = 999; }
}
void packets(int offset, int size) {
    int i, k;
    for (i = 0; i < 4096; ++i) input[i] = 0;
    for (i = offset; i + 47 <= size; i += 47) {
        input[i] = 0x54; input[i+1] = 0x2c;
        for (k = 0; k < 12; ++k) input[i+6+3*k] = 100;
    }
}
int run_tests(void) {
    int i, offset, expected, n;
    for (i = 0; i < 4096; ++i) input[i] = 0;
    for (n = 0; n < 95; ++n) {
        stale(); CHECK(LEIDA_DATA_HANDLE1(dst.points, input, n) == 0);
        for (i = 0; i < 800; ++i) CHECK(dst.points[i].distance == 0);
    }
    CHECK(LEIDA_short_inputs == 95);
    stale(); CHECK(LEIDA_DATA_HANDLE1(dst.points, input, 1798) == 0);
    CHECK(LEIDA_sync_failures == 1 && LEIDA_raw_count == 0);
    for (i = 0; i < 800; ++i) CHECK(dst.points[i].distance == 0);
    for (offset = 0; offset < 47; ++offset) {
        stale(); packets(offset, 1798);
        expected = ((1798-offset)/47)*12;
        CHECK(LEIDA_DATA_HANDLE1(dst.points, input, 1798) == expected);
        CHECK(LEIDA_raw_count == expected);
        CHECK(LEIDA_DATA_HANDLE3_2(filtered, dst.points, 800) == expected-10);
        for (i = expected; i < 800; ++i) CHECK(dst.points[i].distance == 0);
        CHECK(dst.before == 0x12345678 && dst.after == 0x87654321);
    }
    /* Exact block end, missing interior header, capacity bound. */
    stale(); packets(0, 141);
    CHECK(LEIDA_DATA_HANDLE1(dst.points, input, 141) == 36);
    stale(); packets(0, 1798); input[4*47] = 0;
    CHECK(LEIDA_DATA_HANDLE1(dst.points, input, 1798) == 444);
    CHECK(LEIDA_missing_packets == 1);
    for (i = 48; i < 60; ++i) CHECK(dst.points[i].distance == 0);
    stale(); packets(0, 4096);
    CHECK(LEIDA_DATA_HANDLE1(dst.points, input, 4096) == 792);
    for (i = 792; i < 800; ++i) CHECK(dst.points[i].distance == 0);
    CHECK(dst.after == 0x87654321);
    /* Start inhibited indefinitely, including reverse branch. */
    daoche_flag = 1;
    for (i = 0; i < 100; ++i) TIM5_IRQHandler();
    CHECK(motor_ccr == 0 && pi_calls == 0 && Radar_stop_latched == 0);
    daoche_flag = 0; irq_mask = 1;
    Radar_ControlCompleted(); CHECK(irq_mask == 1);
    irq_mask = 0;
    for (i = 0; i < 49; ++i) TIM5_IRQHandler();
    CHECK(!Radar_stop_latched && motor_ccr == 73);
    Radar_ControlCompleted(); CHECK(irq_mask == 0 && Radar_age_ticks == 0);
    for (i = 0; i < 50; ++i) TIM5_IRQHandler();
    CHECK(Radar_stop_latched && motor_ccr == 0 && Radar_timeout_count == 1);
    n = pi_calls;
    Radar_ControlCompleted(); daoche_flag = 1;
    for (i = 0; i < 100; ++i) TIM5_IRQHandler();
    CHECK(motor_ccr == 0 && pi_calls == n && Radar_timeout_count == 1);
    return 0;
}
int main(void) { radar_test_result = run_tests(); return radar_test_result; }
