#ifndef __SERVO_H
#define	__SERVO_H	 

#define  servo_Midpwm1 ((uint16_t)1445)
#define  servo_left    ((uint16_t)1720)
#define  servo_right   ((uint16_t)1170)

void Servo_Init(uint16_t psc,uint16_t arr,uint16_t puse);

void Servo_ChangePwm(uint16_t servo_pwm);


#endif
