#ifndef __MPU_H__
#define __MPU_H__

#include "delay.h"
#include "stm32f4xx.h"

extern __IO uint32_t imu_tick;

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t mx;
	int16_t my;
	int16_t mz;

	int16_t temp;

	int16_t gx;
	int16_t gy;
	int16_t gz;
	
	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;

	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
} Mpu_Data_t;

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t mx;
	int16_t my;
	int16_t mz;

	float temp;

	float wx; /*!< omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s */
	float wy;
	float wz;

	float vx;
	float vy;
	float vz;

	float rol;
	float pit;
	float yaw;
} IMU_t;

uint8_t Mpu_Init(void);
void Mpu_Offset_Call(void);
void Get_MPU_Data(void);
float Get_16Bit_Data(uint8_t addr_h, uint8_t addr_l);
void MPU_Read_Bytes(uint8_t reg_addr, uint8_t* buff_addr, uint8_t len);
//void Get_MPU6500_Data(void);

void Init_Quaternion(void);
void MPU_Get_Data(void);
void MPU_Ahrs_Update(void); 
void MPU_Attitude_Update(void);

#endif


