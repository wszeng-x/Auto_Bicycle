#ifndef __UASRT3_H
#define __UASRT3_H

#include "stm32f4xx.h"
#include "stdio.h"

#define DEBUG 1

#if DEBUG
	#define USART3_PRINT(format, ...) printf(format, ##__VA_ARGS__)
#endif

void Usart3_Init(void);

#endif /*__UASRT3_H*/
