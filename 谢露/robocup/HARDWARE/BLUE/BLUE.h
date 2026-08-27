#ifndef __BLUE_H
#define __BLUE_H

#include "sys.h" 
#include "stdio.h"	
#include "LEIDA_DATA.h" 
#include "timer.h"

#define start ((uint8_t) 1)
#define stop  ((uint8_t) 0)
extern u8 BLUE_change_sign;
extern u8 BLUE_BUFFER_STA ;

void BLUE_init(u32 bound);//PC6-TX  PC7-RX   USART6
void canshu_gengxin(u8 canshu[],u16 size);

#endif

