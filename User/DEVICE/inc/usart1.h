#ifndef __USART1_H__
#define __USART1_H__

#include "stm32f4xx.h"

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u

void Usart1_Init(void);
const uint8_t* Get_Rc_Bufferx(uint8_t index);
void Reset_Rc(void);
#endif
