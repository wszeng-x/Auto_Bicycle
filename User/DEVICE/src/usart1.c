#include "usart1.h"

#define NULL ((void*) 0)
	
static uint8_t rc_rx_buf0[RC_FRAME_LENGTH]; //18字节
static uint8_t rc_rx_buf1[RC_FRAME_LENGTH]; 

extern uint8_t remoter_data_coming;

void Usart1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;   
	USART_InitTypeDef USART_InitStructure;  
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;     //DMA传输中断
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);  //PB7  usart1 rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;      
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = 100000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_Even; //偶校验
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &USART_InitStructure);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);   //使能MDA接收USART数据
	USART_Cmd(USART1, ENABLE);

	//DMA2 stream5 ch4  or (DMA2 stream2 ch4)    !!!!!!! P206 of the datasheet
	//串口接收DMA传输
	DMA_DeInit(DMA2_Stream2);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR); 
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rc_rx_buf0; //存储器地址，数组
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;       //外设到存储器
	DMA_InitStructure.DMA_BufferSize = RC_FRAME_LENGTH;      //18
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设数据寄存器只有一个，失能
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据宽度，1字节
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //存储器数据宽度，1字节
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;         //循环传送
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; //数据直接传送，不缓冲,
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;  //FIFO阈值，配置无效
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; 
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);
	
	DMA_DoubleBufferModeConfig(DMA2_Stream2, (uint32_t)rc_rx_buf1, DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA2_Stream2, ENABLE);  //开启双缓冲模式
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  
	DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE); //先关闭中断
	DMA_Cmd(DMA2_Stream2, ENABLE);
}


void DMA2_Stream2_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream2,DMA_IT_TCIF2) != RESET)
	{
		DMA_ClearITPendingBit(DMA2_Stream2,DMA_IT_TCIF2);
		//遥控数据已准备
		remoter_data_coming = 1;
	}
}

const uint8_t* Get_Rc_Bufferx(uint8_t index)
{
	if(index == 0)
	{
		return rc_rx_buf0;
	}
	else if(index == 1)
	{
		return rc_rx_buf1;
	}
	return NULL;
}

/**
  * @brief          重启遥控，数据出错时调用
  * @author         HLL
  * @param[in]      遥控器数据结构体变量地址
  * @retval         返回空
  */
void Reset_Rc(void)
{
	USART_Cmd(USART1, DISABLE);
	DMA_Cmd(DMA2_Stream2, DISABLE);
	//写入要在DMA2 Stream2上传输的数据单元数,单位字节
	DMA_SetCurrDataCounter(DMA2_Stream2, SBUS_RX_BUF_NUM);

	DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
	DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2); //清除中断挂起位
	DMA_Cmd(DMA2_Stream2, ENABLE);
	USART_Cmd(USART1, ENABLE);
}
