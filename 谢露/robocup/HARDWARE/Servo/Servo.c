#include "stm32f4xx.h"    
#include "PWM.h"
#include "stdio.h"
#include "Servo.h"
//舵机初始化 servo_midpwm = 144  输出频率50hz 20ms		
//PA6     TIM3_CH1    pwm0,5~2.5ms 90度：pwm=1.5ms puse=150（根据实际情况浮动）puse:50~250 
void Servo_Init(uint16_t psc,uint16_t arr,uint16_t puse)
{
	TIM3_PWM_Init(psc-1,arr-1,puse);//113 183
}

void Servo_ChangePwm(uint16_t servo_pwm)//如果改servo定时器这段代码也要改
{
	printf("final servo_pwm=%d\n\n",servo_pwm);
//	printf("测试Servo_f= %d\r\n",IC_GetFreq());//PA6 duoji
//	printf("测试Servo_DUTY= %d\r\n\r\n",IC_GetDuty());
	if(servo_pwm>servo_left)servo_pwm = servo_left;
	if(servo_pwm<servo_right)servo_pwm = servo_right;//
	
	TIM_SetCompare1(TIM3,servo_pwm);
}




