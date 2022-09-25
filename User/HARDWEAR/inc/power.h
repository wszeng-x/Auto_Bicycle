#ifndef POWER_CTRL_H
#define POWER_CTRL_H

#include "stm32f4xx.h"
#include "delay.h"

#define POWER1_CTRL_SWITCH 0
#define POWER2_CTRL_SWITCH 1
#define POWER3_CTRL_SWITCH 2
#define POWER4_CTRL_SWITCH 3

#define POWER1_CTRL_ON GPIO_SetBits(GPIOH, GPIO_Pin_2)
#define POWER2_CTRL_ON GPIO_SetBits(GPIOH, GPIO_Pin_3)
#define POWER3_CTRL_ON GPIO_SetBits(GPIOH, GPIO_Pin_4)
#define POWER4_CTRL_ON GPIO_SetBits(GPIOH, GPIO_Pin_5)

#define POWER1_CTRL_OFF GPIO_ResetBits(GPIOH, GPIO_Pin_2)
#define POWER2_CTRL_OFF GPIO_ResetBits(GPIOH, GPIO_Pin_3)
#define POWER3_CTRL_OFF GPIO_ResetBits(GPIOH, GPIO_Pin_4)
#define POWER4_CTRL_OFF GPIO_ResetBits(GPIOH, GPIO_Pin_5)

void Power_Ctrl_Config(void);
void Power_Ctrl_On(uint8_t num);
void Power_Ctrl_Off(uint8_t num);


#endif /*POWER_CTRL_H*/
