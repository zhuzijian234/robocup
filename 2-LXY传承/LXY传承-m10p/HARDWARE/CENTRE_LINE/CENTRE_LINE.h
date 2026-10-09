#ifndef CENTRE_LINE_H
#define CENTRE_LINE_H
#include "stm32f4xx.h"
#include "path_track.h"
#include "m10p.h"

#define SERVO_PWM_MIN 1170 /* 右打满 */
#define SERVO_PWM_MID 1445
#define SERVO_PWM_MAX 1720 /* 左打满 */

/* 舵机三组增益：双侧直道、双侧弯道、单侧路径；速度环只使用 kp/ki/kd。 */
typedef struct
{
    float kp, kp_2, kp_3, ki, kd, kd_2, kd_3;
    float err, err_l, err_sum;
} pid_type;
extern pid_type Servo_pd, Speed_pid;
extern uint8_t Servo_PD_valid; /* 本帧确实采用新 PD 输出，不包含保舵 */
extern float CENTER_X_TARGET_MM, MODE0_ERR_CLAMP_MM;
extern float PATH_PREVIEW_MM, PATH_WIDTH_MM;
extern PathObservation Steering_path;
extern PathCommand Steering_command;
void Steering_Init(void);
void Steering_Update(const M10P_Scan *scan, const PathPoint *points, uint16_t count);
void Speed_PID_Init(pid_type *pid, float kp, float ki, float kd);
float PID_realize(float speed_now, float speed_mubiao, pid_type *speed_pid);
#endif
