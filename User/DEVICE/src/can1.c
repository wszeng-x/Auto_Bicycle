#include "can1.h"

void Init_Can1(void)
{
	GPIO_InitTypeDef       GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	NVIC_InitTypeDef       NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);
  	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = ENABLE; //报文发送优先级
	
	//配置位时序寄存器CAN_BTR,及工作模式
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq; //同步段
  CAN_InitStructure.CAN_BS1 = 8;  //用于重新同步，相位超前
  CAN_InitStructure.CAN_BS2 = 4;  //用于重新同步，相位滞后
  CAN_InitStructure.CAN_Prescaler = 3;   //CAN BaudRate 45/(1+9+5)/3=1Mbps
  CAN_Init(CAN1, &CAN_InitStructure);

	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh= 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
  
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
  CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
}

void Can1_Send_4Msg(int16_t fl, int16_t fr, int16_t bl, int16_t br)
{
		CanTxMsg tx_message;
	  tx_message.StdId = 0x00;  //0x200, 发送到底盘3508电机
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;           //数据长度为8字节,高位先行
    tx_message.Data[0] = (uint8_t)(fl >> 8); //fl高8位
    tx_message.Data[1] = (uint8_t)fl;        //fl低8位
    tx_message.Data[2] = (uint8_t)(fr >> 8);
    tx_message.Data[3] = (uint8_t)fr;
    tx_message.Data[4] = (uint8_t)(bl >> 8);
    tx_message.Data[5] = (uint8_t)bl;
    tx_message.Data[6] = (uint8_t)(br >> 8);
    tx_message.Data[7] = (uint8_t)br;
    CAN_Transmit(CAN1,&tx_message);
}

//超级电容功率设置
void Set_Cap_Power(uint16_t target_power)
{
	CanTxMsg tx_message;
	tx_message.StdId = 0x210;  // 0x210,发送到超级电容
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x08;
	tx_message.Data[0] = target_power >> 8;
	tx_message.Data[1] = target_power;
	CAN_Transmit(CAN1, &tx_message);
}



