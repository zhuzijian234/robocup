#ifndef __LWDG_H
#define	__LWDG_H	 

 
#include "usart.h"
void IWDG_Init(u8 prer,u16 rlr);//IWDG³õÊ¼»¯
void IWDG_Feed(void);  //Î¹¹·º¯Êý
#endif
