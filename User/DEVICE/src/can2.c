#include "can2.h"

void Init_Can2(void)
{
	  GPIO_InitTypeDef       GPIO_InitStructure;
	  CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    NVIC_InitTypeDef       NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
		CAN_DeInit(CAN2);
    CAN_StructInit(&CAN_InitStructure);
		
		CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;    
    CAN_InitStructure.CAN_AWUM = DISABLE;    
    CAN_InitStructure.CAN_NART = DISABLE;    
    CAN_InitStructure.CAN_RFLM = DISABLE;    
    CAN_InitStructure.CAN_TXFP = ENABLE; 

    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = 8;
    CAN_InitStructure.CAN_BS2 = 4;
    CAN_InitStructure.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN2, &CAN_InitStructure);

		CAN_FilterInitStructure.CAN_FilterNumber=14;
		CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
		CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
		CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
		CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
		CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
		CAN_FilterInit(&CAN_FilterInitStructure);
    
		NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
    CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);	
}

void Can2_Send2_Gimbal(int16_t yaw, int16_t pitch)
{
		CanTxMsg tx_message;
	  tx_message.StdId = 0x2FF;  //0x2FF, 发送到云台6020
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;           //数据长度为8字节,高位先行
    tx_message.Data[0] = (uint8_t)(yaw >> 8); //ID5数据
    tx_message.Data[1] = (uint8_t)yaw;        
    tx_message.Data[2] = (uint8_t)(pitch >> 8); //ID6数据
    tx_message.Data[3] = (uint8_t)pitch;
    CAN_Transmit(CAN2,&tx_message);
}

void Can2_Send2_Wave(int16_t wave)
{
		CanTxMsg tx_message;
	  tx_message.StdId = 0x200;  //0x200, 发送到拨轮3508
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;           //数据长度为8字节,高位先行
    tx_message.Data[0] = (uint8_t)(wave >> 8); //ID1数据
    tx_message.Data[1] = (uint8_t)wave;        
    CAN_Transmit(CAN2,&tx_message);
}

void Can2_Send2_Shoot(int16_t left, int16_t right)
{
		CanTxMsg tx_message;
	  tx_message.StdId = 0x1FF;  //0x1FF, 发送到摩擦轮3508
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;           //数据长度为8字节,高位先行
    tx_message.Data[0] = (uint8_t)(left >> 8); //ID5数据
    tx_message.Data[1] = (uint8_t)left;      
    tx_message.Data[2] = (uint8_t)(right >> 8); //ID5数据
    tx_message.Data[3] = (uint8_t)right;      
    CAN_Transmit(CAN2,&tx_message);
}
