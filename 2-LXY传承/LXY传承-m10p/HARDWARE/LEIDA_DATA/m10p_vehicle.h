/**
 * @file    m10p_vehicle.h
 * @brief   M10P 适配层对外接口 — 取数据(M10P_Poll)与转换打包(M10P_Build)
 *
 * 谁在用: main.c 主循环。典型顺序:
 *     M10P_Poll();                         // 收 + 解析成扫描帧
 *     scan = M10P_Acquire();               // 取一帧(m10p.c)
 *     valid_couter = M10P_Build(scan, LEIDA_DATA2, LEIDA_DATA_COUNTER);
 *     ... 循线计算 ...
 *     M10P_Release(scan);                  // 还回去
 */
#ifndef M10P_VEHICLE_H
#define M10P_VEHICLE_H
#include "m10p.h"
#include "LEIDA_DATA.h"
/* 本帧来源标记: 帧号 / 正前方首个有效点的时刻 / 链路代号 —— 遥测里用来对上"这帧是哪来的" */
extern uint32_t M10P_control_seq, M10P_control_front_us, M10P_control_epoch;
/* 三个扇区的点数(按 0.5° 桶计): 正前 70°~110° / 左 110°~170° / 右 10°~70° */
extern uint16_t M10P_front_bins, M10P_left_bins, M10P_right_bins;
/* M10P_clearance_mm: 正前方走廊(半宽180mm)内最近的障碍距离(mm), 越大越空
 * M10P_speed_scale:  限速系数 0.25~1, 净空近就压低目标速度 */
extern float M10P_clearance_mm, M10P_speed_scale;
extern uint8_t M10P_perception_ok; /* 本帧感知是否可信; 0 = 撤销驱动许可，主循环清PD状态，TIM5清PI并归零PWM */
void M10P_Poll(void); /* 主循环每圈调一次: 收块 + 喂解析器 + 处理断流/过期/跳变 */
uint16_t M10P_Build(const M10P_Scan *scan, _LEIDA_DATA *out, uint16_t capacity);
/* 纯转换 + 安全判据都在本模块里; 扫描帧由 main 一直持有到用完为止。 */
#endif
