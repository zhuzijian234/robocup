
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
    uint16_t i,j=0,k,skip;float start,end,angle;
    LEIDA_parse_calls++;LEIDA_raw_count=0;
    Diag_detail_u[4]|=64;Diag_detail_u[17]=0xffffffffu;
    memset(data,0,LEIDA_DATA_COUNTER*sizeof(*data));
    if(!size){LEIDA_short_inputs++;return 0;}
    for(i=0;i<size;i++){
        if(!lidar_pending && arr[i]!=0x54)continue;
        lidar_packet[lidar_pending++]=arr[i];
        if(lidar_pending<47)continue;
        if(Diag_RadarPacket(lidar_packet)){
            if(Diag_detail_u[17]==0xffffffffu)Diag_detail_u[17]=i>=46?i-46:0;
            LEIDA_speed_dps=(uint16_t)(lidar_packet[2]|lidar_packet[3]<<8);
            start=(lidar_packet[4]|lidar_packet[5]<<8)/100.0f;
            end=(lidar_packet[42]|lidar_packet[43]<<8)/100.0f;
            if(end<start)end+=360.0f;
            for(k=0;k<12 && j<LEIDA_DATA_COUNTER;k++,j++){
                data[j].distance=(float)(lidar_packet[6+3*k]|lidar_packet[7+3*k]<<8);
                /* Twelve measurements include both endpoints: divisor is 11. */
                angle=start+(end-start)*k/11.0f;
                angle=360.0f-angle+LEIDA_ANGLE_CENTER;
                while(angle>=360.0f)angle-=360.0f;
                while(angle<0)angle+=360.0f;
                data[j].angle=angle;
                if(data[j].distance>0)Diag_detail_u[18]|=1u<<(uint16_t)(angle/30.0f);
            }
            lidar_pending=0;
        }else{
            LEIDA_missing_packets++;Diag_detail_u[16]++;
            /* Resynchronize bytewise; preserve a potential header inside a bad packet. */
            for(skip=1;skip<47 && lidar_packet[skip]!=0x54;skip++){}
            lidar_pending=(uint16_t)(47-skip);
            if(lidar_pending)memmove(lidar_packet,lidar_packet+skip,lidar_pending);
        }
    }
    if(!j && size>=47)LEIDA_sync_failures++;
    LEIDA_raw_count=j;return j;
}

uint16_t LEIDA_DATA_HANDLE3_2(_LEIDA_DATA data[], _LEIDA_DATA arr[], u16 size)
{
    int i, j;
    j = 0;
    for (i = 0; i < size; i++) {
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

        uint8_t encoder_fresh=0,pi_fresh=0;
        extern uint16_t Encoder_cnt_temp;
        Radar_GuardTick();
        if (!Radar_started || Radar_stop_latched) {
            /* Zero duty removes propulsion; it is not an active brake.
             * Skip PI so its integral cannot accumulate during inhibition. */
            Get_Encoder();encoder_fresh=1;
            moto_pwm = 0;
            Moto_Speed(0);
        } else if (daoche_flag == 1) {
            TIM_SetCompare1(TIM2, (uint16_t)(100 * 0.5));  /* 50%制动,不足以驱动小车 */
        } else {
            Get_Encoder();encoder_fresh=1;
            moto_pwm = PID_realize(Speed_now, Speed_mubiao, &Speed_pid);pi_fresh=1;
            Moto_Speed(moto_pwm);
        }
        Diag_MotorTick(Encoder_cnt_temp,encoder_fresh,pi_fresh);
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
