"""Build tests against extracted production C; run with host cc if available.

With only AC5 installed, produces radar_tests.axf for debugger/simulator use.
An AC5 compile/link is NOT reported as a runtime test pass.
"""
from pathlib import Path
import shutil
import subprocess

PROJECT = Path(__file__).resolve().parents[1]
ROOT = PROJECT.parents[1]
OUT = ROOT / "tmp/radar_parser_tests"
OUT.mkdir(parents=True, exist_ok=True)


def section(path, begin, end):
    text = path.read_text(encoding="utf-8-sig")
    return text[text.index(begin):text.index(end, text.index(begin))]


parser = section(PROJECT / "HARDWARE/LEIDA_DATA/LEIDA_DATA.c",
                 "uint16_t LEIDA_DATA_HANDLE1(", "/* ======================== HANDLE2")
filter_fn = section(PROJECT / "HARDWARE/LEIDA_DATA/LEIDA_DATA.c",
                    "uint16_t LEIDA_DATA_HANDLE3_2(", "/* ======================== 边界")
timer = PROJECT / "HARDWARE/LEIDA_TIMER/timer.c"
guard = section(timer, "volatile uint16_t Radar_age_ticks", "/**")
irq = timer.read_text(encoding="utf-8-sig").split("void TIM5_IRQHandler(void)", 1)[1]

prefix = r'''
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
'''

tests = r'''
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
'''

source = OUT / "radar_tests.c"
source.write_text(prefix + parser + filter_fn + guard +
                  "void TIM5_IRQHandler(void)" + irq + tests, encoding="utf-8")
cc = shutil.which("gcc") or shutil.which("clang")
if cc:
    exe = OUT / "radar_tests.exe"
    subprocess.run([cc, "-std=c99", "-O0", str(source), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
    print("Production C regression tests executed and passed")
else:
    ac5 = Path("D:/Keil5/ARM/ARMCC/bin")
    obj, axf = OUT / "radar_tests.o", OUT / "radar_tests.axf"
    stack = OUT / "stack.s"
    stack.write_text("    AREA STACK, NOINIT, READWRITE, ALIGN=3\n"
                     "    SPACE 4096\n    EXPORT __initial_sp\n"
                     "__initial_sp\n    END\n", encoding="ascii")
    stack_obj = OUT / "stack.o"
    subprocess.run([str(ac5/"armasm.exe"), "--cpu=Cortex-M4.fp.sp",
                    str(stack), "-o", str(stack_obj)], check=True)
    subprocess.run([str(ac5/"armcc.exe"), "--cpu=Cortex-M4.fp.sp", "--c99", "-O0",
                    "--library_type=microlib", "-g", "-c", str(source), "-o", str(obj)], check=True)
    subprocess.run([str(ac5/"armlink.exe"), "--cpu=Cortex-M4.fp.sp",
                    "--library_type=microlib", "--entry=__main", "--ro_base=0x08000000",
                    "--rw_base=0x20000000", str(obj), str(stack_obj), "-o", str(axf)], check=True)
    print("AC5 test harness compiled/linked; NOT EXECUTED. Inspect radar_test_result after main in simulator.")
