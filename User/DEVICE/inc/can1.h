#ifndef __CAN1_H__
#define __CAN1_H__

#include "stm32f4xx.h" 
			
void Init_Can1(void);

void Can1_Send_4Msg(int16_t fl, int16_t fr, int16_t bl, int16_t br);
void Set_Cap_Power(uint16_t target_power);
#endif
