# 谢露 vs LXY — 雷达小车全部外设引脚复用完整对比

## 一、串口 (USART)

| 功能 | 谢露 (robocup) | LXY (robo_s弯) |
|------|:---:|:---:|
| **Debug printf** | USART1, PA9(TX) PA10(RX) @115200 | USART1, PA9(TX) PA10(RX) @115200 |
| **雷达数据** | **USART2**, PA2(TX) PA3(RX) @230400 | **USART6**, PC6(TX) PC7(RX) @230400 |
| **雷达DMA** | DMA1 Stream5 Channel4, 缓冲1798B | DMA2 Stream1 Channel5, 缓冲2000B |
| **雷达串口初始化** | `uart2_init()` | `uart6_init()` |
| **蓝牙** | USART6, PC6(TX) PC7(RX) @115200 | **USART3**, PB10(TX) PB11(RX) @9600 |
| **蓝牙STATE** | 无 | PC2 (输入, 高=已连接) |
| **蓝牙中断方式** | RXNE单字节中断 | RXNE + IDLE双中断 |
| **蓝牙状态** | ❌ main.c中已注释, 硬件未初始化 | ⚠️ 硬件已初始化, 但主循环未使用 |

```
串口互换了:
  谢露: 雷达→USART2, 蓝牙→USART6(关)
  LXY:  雷达→USART6, 蓝牙→USART3(开)
```

---

## 二、PWM / 定时器 / 编码器

| 功能 | 谢露 | LXY |
|------|------|-----|
| **舵机PWM** | TIM3 CH1 → **PA6** | TIM3 CH1 → **PA6** |
| 舵机频率 | 84MHz/84/20000 = **50Hz** | 84MHz/84/20000 = **50Hz** |
| 舵机中位 | **1445** | **1560** (156.5×10) |
| 舵机左限 | 1720 | 1800 |
| 舵机右限 | 1170 | 1360 |
| 舵机初始化 | `Servo_Init(84, 20000, 1445)` | `Servo_Init(84, 20000, 1565)` |
| | | |
| **电机PWM** | TIM2 **CH4** → **PB11** | TIM2 **CH3** → **PA2** |
| 电机频率 | 84MHz/1/4200 = **20kHz** | 84MHz/42/100 = **20kHz** |
| 电机初始化 | `Moto_Init(1, 4200, 0)` | `Moto_Init(42, 100, 90)` |
| 设置占空比 | `TIM_SetCompare4(TIM2, ...)` | `TIM_SetCompare3(TIM2, ...)` |
| | | |
| **电机方向** | **PB10** (单IO) | **PE5**(高=正转) + **PE4**(低=使能) |
| 方向控制 | `GPIO_SetBits(GPIOB, PB10)` = 正转 | PE5控制方向, PE4控制使能 |
| | | |
| **编码器** | TIM4 → **PD12, PD13** | TIM4 → **PD12, PD13** |
| 编码器模式 | 正交编码 TI12, 滤波0xF | 正交编码 TI12, 滤波0xF |
| 速度公式 | `Speed = Encoder_cnt/11*100` | `Speed = (Encoder_cnt*100)/(4*11*6.25)` |
| | | |
| **雷达电机PWM** | TIM9 **CH1** → **PA2** | TIM9 **CH2** → **PE6** |
| 雷达电机频率 | 未知 (psc/arr未传) | 168MHz/1680/100 = **1kHz** |
| 雷达电机初始化 | ⚠️ 代码存在但main中未调用 | `PWM_Init_leida()` + `PWM_SetCompare_leida(97)` |
| 雷达电机备用 | 无 | TIM1 CH4 → PA11 (`LEIDA_PWM_Init`, 预留) |
| | | |
| **速度定时器** | TIM5, **10ms** | TIM5, **10ms** |
| 初始化 | `TIM5_Int_Init(99, 8399)` | `TIM5_Int_Init(99, 8399)` |
| ISR内容 | 读编码器→直接设Moto_pwm | 读编码器→`PID_realize()`→`Moto_Speed()` |

> ⚠️ 谢露的 PA2 同时被用作 USART2_TX 和 TIM9_CH1(雷达电机PWM) 和 TIM2_CH3(预留)。代码注释说"只配置PA3也行 避免PA2 TIM2_CH3 冲突"，实际上因为电机用了PB11(TIM2_CH4)所以没炸。

---

## 三、预留/未启用的外设

| 功能 | 谢露 | LXY |
|------|------|-----|
| 互补PWM | TIM1 CH1(P/N) → PA8/PA7 (预留) | TIM1 CH1(P/N) → PA8/PA7 (预留) |
| SPWM定时器 | TIM14 (预留) | TIM14 (预留) |
| 通用PWM | TIM11 → PB9 (预留, 已注释) | TIM10 → PF6, TIM11 → PB9 (预留) |
| TIM9 CH1 | — | → PE5 (预留) |
| PWMI舵机反馈 | TIM4 CH1 → PB6 (已注释) | 无 |
| LED | PF9, PF10 (推挽输出) | 无 |
| 看门狗 | IWDG, 预分频64, 1s超时 | 无 |

---

## 四、main() 初始化调用顺序对比

```
谢露 main():                           LXY main():
─────────────────────────────────      ─────────────────────────────────
NVIC_PriorityGroupConfig(2)            NVIC_PriorityGroupConfig(2)
delay_init(84)                         uart_init(115200)        ← debug
//BLUE_init         (蓝牙-注释)        delay_init(84)
uart_init           (USART1 debug)     PWM_Init_leida()         ← 雷达电机
uart2_init          (USART2 雷达)      Bluetooth_Init()         ← 蓝牙启用!
DMA_Initializes     (DMA1 雷达)        PWM_SetCompare_leida(97) ← 初始转速
Servo_Init(84,20000,1445)              uart6_init       (USART6 雷达)
//PWMI_Init         (舵机反馈-注释)    DMA_Initializes  (DMA2 雷达)
Moto_Init(1,4200,0)                    Servo_Init(84,20000,1565)
Encoder_Init                           Midline_PD_Init(...)     ← PID参数1
TIM5_Int_Init(99,8399)                 Speed_PID_Init(...)
LED_Init                               Moto_Init(42,100,90)
IWDG_Init                              Encoder_Init
                                       TIM5_Int_Init(99,8399)
Handle3_TEST()       ← 只跑测试!       Midline_PD_Init(...)     ← PID参数2(覆盖)
while(1) {                             蓝牙参数配置...
    printf("ok");                      while(1) {
}                                          if(DMA_RX_DONE) {
                                              完整循线逻辑:
                                              雷达解析→中线拟合
                                              →9模式PD→舵机
                                          }
                                       }
```

---

## 五、结构差异

| | 谢露 | LXY |
|------|------|-----|
| 中线拟合在哪 | `LEIDA_DATA.c` 里的 `Midline_fit_2()` | `CENTRE_LINE.c` 独立模块 |
| PID控制在哪 | 可能在 `Test.c` 或分散在各处 | `CENTRE_LINE.c` 独立模块 |
| CENTRE_LINE.c/h | **空文件** | 完整实现(320行) |
| 额外数据处理模块 | `LEIDA_HANDLE/Leida_handle.c` | 无 (合并在LEIDA_DATA中) |
| 测试模块 | `Test/Test.c` (286行) | 无 (测试宏 HW_TEST_MOTOR_SERVO 在main里) |
| 工程状态 | 开发中测试版本 | 完整循线版本 |

---

## 六、一图总结

```
                         谢露                          LXY
                       ─────────                    ─────────

    Debug ────────── USART1 (PA9/PA10) ────────── USART1 (PA9/PA10)
    雷达 ────────── USART2 (PA2/PA3)              USART6 (PC6/PC7) ───── 雷达
                   DMA1_S5_C4                    DMA2_S1_C5
    蓝牙 ────────── USART6 (PC6/PC7)  关了        USART3 (PB10/PB11) ─── 蓝牙(初始化了但没用)

    舵机 ────────── TIM3_CH1 PA6 ──────────────── TIM3_CH1 PA6 ───── 舵机
    电机 ────────── TIM2_CH4 PB11                 TIM2_CH3 PA2 ─────── 电机
    方向 ────────── PB10 (单脚)                   PE5+PE4 (双脚) ─── 方向
    编码器 ──────── TIM4 PD12/PD13 ────────────── TIM4 PD12/PD13 ─── 编码器
    雷达电机 ────── TIM9_CH1 PA2 (未初始化)        TIM9_CH2 PE6 ─────── 雷达电机
    速度定时器 ──── TIM5 10ms ─────────────────── TIM5 10ms ──────── 速度定时器

    额外 ────────── LED×2, IWDG, PWMI             TIM3中断, 互补PWM
    状态 ────────── 测试版 (printf ok)             完整循线版
```
