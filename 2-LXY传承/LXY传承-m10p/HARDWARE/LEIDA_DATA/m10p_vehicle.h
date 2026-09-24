#ifndef M10P_VEHICLE_H
#define M10P_VEHICLE_H
#include "m10p.h"
#include "LEIDA_DATA.h"
extern uint32_t M10P_control_seq, M10P_control_front_us, M10P_control_epoch;
extern uint16_t M10P_front_bins, M10P_left_bins, M10P_right_bins;
extern float M10P_clearance_mm, M10P_speed_scale;
extern uint8_t M10P_perception_ok;
void M10P_Poll(void);
uint16_t M10P_Build(const M10P_Scan *scan, _LEIDA_DATA *out, uint16_t capacity);
/* Pure conversion/safety logic is in this module; main keeps the scan until done. */
#endif
