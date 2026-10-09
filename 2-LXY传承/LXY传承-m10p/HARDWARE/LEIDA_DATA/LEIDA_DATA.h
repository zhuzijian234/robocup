#ifndef LEIDA_DATA_H
#define LEIDA_DATA_H
#include "sys.h"
#include "usart.h"
#include "arm_math.h"
#include "path_track.h"
typedef struct
{
    float angle, distance;
} _LEIDA_DATA;
#define LEIDA_DATA_COUNTER 800
#define LEIDA_ANGLE_piancha 0.5f
extern _LEIDA_DATA LEIDA_DATA2[LEIDA_DATA_COUNTER];
extern PathPoint LEIDA_front_points[LEIDA_DATA_COUNTER / 2];
extern uint16_t valid_couter, LEIDA_speed_dps;
extern volatile uint16_t LEIDA_raw_count;
uint16_t LEIDA_FrontPoints(const _LEIDA_DATA *points, uint16_t count);
/* 开口仅作诊断，不以左/右断点直接命令舵机方向。 */
extern float duandian_MIN_Y, BLUE_ANGLE_LEFT_RIGHT;
uint16_t LEIDA_DATA_HANDLE6(_LEIDA_DATA *out, _LEIDA_DATA *points, u16 count);
uint16_t LEIDA_DATA_HANDLE7(_LEIDA_DATA *out, _LEIDA_DATA *points, u16 count);
uint16_t LEIDA_DATA_HANDLE8(_LEIDA_DATA *points, u16 count);
uint16_t LEIDA_DATA_HANDLE9(_LEIDA_DATA *points, u16 count);
void LEIDA_Opening(const _LEIDA_DATA *points, uint16_t count, uint16_t *left, uint16_t *right);
#endif
