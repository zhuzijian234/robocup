/**
 * @file m10p_config.h
 * @brief M10P协议、缓冲容量与感知门限的单一配置入口。
 * 不包含STM32头文件，主机解析回归可直接复用。
 * 调整容量必须复核普通SRAM；调整门限必须用实车回波与停车距离验证。
 */
#ifndef M10P_CONFIG_H
#define M10P_CONFIG_H
/* ======================== 协议 / 尺度常量 ======================== */
#define M10P_BAUD 512000u          /* USART2 波特率, 20K 型固定 512000 (8N1) */
#define M10P_PACKET_BYTES 160u     /* 单包长度: A5 5A + 长度(2) + 角度(2) + 转速(2) + 70槽×2 + 保留(10) + FA FB 尾 */
#define M10P_SCAN_CAPACITY 2048u   /* 每圈容量上限；24包×70槽=1680槽，实际有效点数受FFFF影响 */
#define M10P_BINS 720u             /* 0.5° 一桶: 360/0.5 = 720 桶; 前方盲区、左右侧点数都按桶统计 */
#define M10P_MAX_AGE_US 150000u    /* 一帧的保鲜期 150ms: 比最慢允许转速(8.3Hz, 120ms)还宽, 超期当"没数据" */
#define M10P_FRONT_MAX_MISSING_BINS 10u /* 正前方允许的最大连续空桶数; 10 桶 = 5° 盲区, 超过就判感知无效 */
#define M10P_MIN_PERIOD_US 60000u  /* 接受一圈扫描的周期下限 60ms (= 16.7Hz), 超出当前稳定工作窗口则拒绝控制，并非断言协议数据为假 */
#define M10P_MAX_PERIOD_US 120000u /* 周期上限 120ms (= 8.3Hz), 超出当前稳定工作窗口则拒绝控制，需结合实测诊断原因 */
#define M10P_BETA_CDEG 0           /* 安装角补偿(单位 0.01°): 雷达装歪了改这里; 目前按正装取 0 */
#define M10P_MIN_MM 100u           /* 测距下限(mm): 仅墙面/分桶过滤下限；原始非零近点仍参与近障停车 */
#define M10P_MAX_MM 10000u         /* 测距上限(mm): 比这远的也丢弃(20K 型量程内, 挡掉无效值) */
/* 前向安全包络 —— 从雷达原点量起, 是保守取值, 实车落地前必须按车体实际尺寸复测。
 * 用法: |x| < 走廊半宽 且 y > 0 的点算"挡在正前方"的障碍, 取最近的 y 当净空。 */
#define M10P_CORRIDOR_HALF_MM 180.0f /* 走廊半宽(mm), 约等于半个车宽 */
#define M10P_STOP_Y_MM 350.0f        /* 净空小于它 -> M10P_perception_ok 置 0 (= 停车线) */
#define M10P_SLOW_Y_MM 1000.0f       /* 净空到它就允许跑满速(中间线性降速, 见 speed_scale) */

#endif
