#ifndef __SPI_H__
#define __SPI_H__

#include "stm32f4xx.h"
#include "sys.h"

#define REST  PBout(10)
#define DC    PBout(9)


#define MPU_NSS_LOW     GPIO_ResetBits(GPIOF, GPIO_Pin_6)  //BSRRH ��ʾBSRR�Ĵ�����16λ��BRy����һ��BRy��1����������͵�ƽ��
#define MPU_NSS_HIGH    GPIO_SetBits(GPIOF, GPIO_Pin_6)


void InitSPI5(void);
uint8_t SPI5_Read_Write_Byte(uint8_t TxData);

#endif
