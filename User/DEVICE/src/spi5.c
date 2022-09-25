#include "spi5.h"


void InitSPI5(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI5, ENABLE);  //90M
	
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource7,GPIO_AF_SPI5);
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource8,GPIO_AF_SPI5);
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_SPI5);
  
	/*SCK、MISO、MOSI*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_SetBits(GPIOF,GPIO_Pin_7 | GPIO_Pin_8 |GPIO_Pin_9);
	
	/*NSS*/
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_SetBits(GPIOF,GPIO_Pin_6);
 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	  //发送接收均为8位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;	        //10
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	      //01
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;	
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI5, &SPI_InitStructure);
 
	SPI_Cmd(SPI5, ENABLE);
	/*SPI5DR寄存器低8位置1*/
	SPI5_Read_Write_Byte(0xff);			
}

uint8_t SPI5_Read_Write_Byte(uint8_t TxData)
{		 			 
	uint8_t retry = 0;
	
	while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET)
	{
		retry++;
		if(retry > 250)	return 0;
	}			  
	SPI_I2S_SendData(SPI5, TxData);      //发送数据到DR寄存器
	retry = 0;

	while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_RXNE) == RESET)
	{
		retry++;
		if(retry > 250) return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI5);    //从DR寄存器接收数据		    
}
