/**
 * @file m10p_config.h
 * @brief M10P协议、缓冲容量与感知门限的单一配置入口。
 * 不包含STM32头文件，主机解析回归可直接复用。
 * 调整容量必须复核普通SRAM；调整门限必须用实车回波与转向表现验证。
 */
#ifndef M10P_CONFIG_H
#define M10P_CONFIG_H
/* ======================== 协议 / 尺度常量 ======================== */
#define M10P_BAUD 512000u          /* USART2 波特率, 20K 型固定 512000 (8N1) */
/* 20260929实测：包长会变化，160字节只是示例，不能作为分包步长。
 * 以下沿用测试工具的布局假定：8字节头部 + 测距槽 + 10字节保留区 + 2字节尾。
 * 22~512为软件接收边界，并非厂家已确认的合法范围；超过整圈容量仍拒绝发布。
 * 原始bin已验证156/158/160/162；164/166只有统计记录，另用合成包回归。 */
#define M10P_HEADER_BYTES 8u
#define M10P_TRAILER_BYTES 12u
#define M10P_PACKET_OVERHEAD (M10P_HEADER_BYTES + M10P_TRAILER_BYTES)
#define M10P_PACKET_MIN_BYTES (M10P_PACKET_OVERHEAD + 2u) /* 至少一个测距槽 */
#define M10P_PACKET_MAX_BYTES 512u /* 暂存容量，不是实际包长或DMA分块长度 */
#define M10P_PACKET_SPAN_CDEG 1500u /* 每包15°，按非FFFF槽数均分；仍待厂家确认 */
#define M10P_SCAN_CAPACITY 2048u   /* 24包×73槽=1752槽可容纳；超容量整圈丢弃，绝不截断后发布 */
#define M10P_BINS 720u             /* 0.5° 一桶: 360/0.5 = 720 桶; 前方盲区、左右侧点数都按桶统计 */
#define M10P_MAX_AGE_US 150000u    /* 新帧准入：前方点年龄超过150ms，不接纳为新控制输入 */
#define M10P_FRONT_MAX_MISSING_BINS 10u /* 正前方允许的最大连续空桶数; 10 桶 = 5° 盲区, 超过就判感知无效 */
#define M10P_MIN_PERIOD_US 60000u  /* 接受一圈扫描的周期下限 60ms (= 16.7Hz), 超出当前稳定工作窗口则拒绝控制，并非断言协议数据为假 */
#define M10P_MAX_PERIOD_US 120000u /* 周期上限 120ms (= 8.3Hz), 超出当前稳定工作窗口则拒绝控制，需结合实测诊断原因 */
#define M10P_BETA_CDEG 0           /* 安装角补偿(单位 0.01°): 雷达装歪了改这里; 目前按正装取 0 */
#define M10P_MIN_MM 100u           /* 测距下限(mm): 仅墙面/分桶过滤下限；原始非零近点仅参与净空诊断 */
#define M10P_MAX_MM 10000u         /* 测距上限(mm): 比这远的也丢弃(20K 型量程内, 挡掉无效值) */
/* 前向净空诊断范围，从雷达原点量起，不触发停车。
 * 用法: |x| < 走廊半宽 且 y > 0 的点算"挡在正前方"的障碍, 取最近的 y 当净空。 */
#define M10P_CORRIDOR_HALF_MM 180.0f /* 走廊半宽(mm), 约等于半个车宽 */

#endif
