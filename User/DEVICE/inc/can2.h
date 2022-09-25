#ifndef __CAN2_H
#define __CAN2_H

#include "stm32f4xx.h" 

void Init_Can2(void);
void Can2_Send2_Gimbal(int16_t yaw, int16_t pitch);
void Can2_Send2_Wave(int16_t wave);
void Can2_Send2_Shoot(int16_t left, int16_t right);

#endif /*__CAN2_H*/
