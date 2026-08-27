#ifndef __MOTO_H
#define	__MOTO_H	 

void Moto_Init(uint16_t psc,uint16_t arr,uint16_t puse);
void Encoder_Init(void);

void Moto_Speed(uint16_t Compare);
void Get_Encoder(void);


extern float Speed_now;
extern u16 Moto_pwm;

#endif
