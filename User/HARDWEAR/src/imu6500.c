#include "imu6500.h"

#include "ist8310_reg.h" 
#include <math.h>
#include "mpu6500_reg.h"
#include "spi5.h"
#include <string.h>
#include "usart3.h"


#define Kp 2.0f                                              /* 
                                                              * proportional gain governs rate of 
                                                              * convergence to accelerometer/magnetometer 
																															*/
#define Ki 0.01f                                             /* 
                                                              * integral gain governs rate of 
                                                              * convergence of gyroscope biases 
																															*/
#define BOARD_DOWN (1)  
#define IST8310

/*数据存储*/
Mpu_Data_t       mpu_data;
IMU_t    imu={0};

volatile float        q0 = 1.0f;
volatile float        q1 = 0.0f;
volatile float        q2 = 0.0f;
volatile float        q3 = 0.0f;
volatile float        exInt, eyInt, ezInt;                   /* error integral */
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;
volatile uint32_t     last_update, now_update;               /* Sampling cycle count, ubit ms */
__IO uint32_t imu_tick;

uint8_t id;   
uint8_t     mpu_buff[14];                          /* buffer to save imu raw data */
uint8_t     ist_buff[6];                           /* buffer to save IST8310 raw data */


/*读寄存器值*/
uint8_t MPU_Read_Byte(uint8_t reg)
{
	uint8_t reg_val;        //寄存器值
	MPU_NSS_LOW;            //片选拉低
	spi_delay();         
	SPI5_Read_Write_Byte(reg|0x80);     //写入寄存器地址，忽略收到的字节
	reg_val=SPI5_Read_Write_Byte(0xff);  //主机发送一个空字节引起MPU6500传送寄存器值
	MPU_NSS_HIGH;
	spi_delay();
	return(reg_val); 
}

/*读多个寄存器值*/
void MPU_Read_Bytes(uint8_t reg_addr, uint8_t* buff_addr, uint8_t len)
{
	MPU_NSS_LOW;
	SPI5_Read_Write_Byte(reg_addr|0x80);
	if(len != 0)
	{
		uint8_t i;
    for (i = 0; i < len; i++)
    {
      *buff_addr = SPI5_Read_Write_Byte(0xFF);
       buff_addr++;
    }
	}
	
	MPU_NSS_HIGH;
}



/*给寄存器写值*/
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data)
{
	uint8_t status;
	MPU_NSS_LOW;
	spi_delay();
	status = SPI5_Read_Write_Byte(reg); //寄存器当前状态（值）
	SPI5_Read_Write_Byte(data);
	MPU_NSS_HIGH;
	spi_delay();
	return status;
}

/**
	* @brief  set imu 6500 gyroscope measure range
  * @param  fsr: range(0,±250dps;1,±500dps;2,±1000dps;3,±2000dps)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t Mpu_Set_Gyro_Fsr(uint8_t fsr)
{
  return MPU_Write_Byte(MPU6500_GYRO_CONFIG, fsr << 3);
}

/**
	* @brief  set imu 6050/6500 accelerate measure range
  * @param  fsr: range(0,±2g;1,±4g;2,±8g;3,±16g)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t Mpu_Set_Accel_Fsr(uint8_t fsr)
{
  return MPU_Write_Byte(MPU6500_ACCEL_CONFIG, fsr << 3); 
}

/**
	* @brief  write IST8310 register through MPU6500's I2C master
  * @param  addr: the address to be written of IST8310's register
  *         data: data to be written
	* @retval   
  * @usage  call in ist8310_init() function
	*/
static void Ist_Reg_Write_By_MPU(uint8_t addr, uint8_t data)
{
    /* turn off slave 1 at first */
    MPU_Write_Byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    delay_ms(2);
    MPU_Write_Byte(MPU6500_I2C_SLV1_REG, addr);
    delay_ms(2);
    MPU_Write_Byte(MPU6500_I2C_SLV1_DO, data);
    delay_ms(2);
    /* turn on slave 1 with one byte transmitting */
    MPU_Write_Byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* wait longer to ensure the data is transmitted from slave 1 */
    delay_ms(10);
}

/**
	* @brief  write IST8310 register through MPU6500's I2C Master
	* @param  addr: the address to be read of IST8310's register
	* @retval 
  * @usage  call in ist8310_init() function
	*/
static uint8_t Ist_Reg_Read_By_MPU(uint8_t addr)
{
    uint8_t retval;
    MPU_Write_Byte(MPU6500_I2C_SLV4_REG, addr);
    delay_ms(10);
    MPU_Write_Byte(MPU6500_I2C_SLV4_CTRL, 0x80);
    delay_ms(10);
    retval = MPU_Read_Byte(MPU6500_I2C_SLV4_DI);
    /* turn off slave4 after read */
    MPU_Write_Byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    delay_ms(10);
    return retval;
}

/**
	* @brief    initialize the MPU6500 I2C Slave 0 for I2C reading.
* @param    device_address: slave device address, Address[6:0]
	* @retval   void
	* @note     
	*/
static void Mpu_Master_I2c_Auto_Read_Config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    /* 
	   * configure the device address of the IST8310 
     * use slave1, auto transmit single measure mode 
	   */
    MPU_Write_Byte(MPU6500_I2C_SLV1_ADDR, device_address);
    delay_ms(2);
    MPU_Write_Byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    delay_ms(2);
    MPU_Write_Byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    delay_ms(2);

    /* use slave0,auto read data */
    MPU_Write_Byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    delay_ms(2);
    MPU_Write_Byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    delay_ms(2);

    /* every eight mpu6500 internal samples one i2c master read */
    MPU_Write_Byte(MPU6500_I2C_SLV4_CTRL, 0x03);
    delay_ms(2);
    /* enable slave 0 and 1 access delay */
    MPU_Write_Byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    delay_ms(2);
    /* enable slave 1 auto transmit */
    MPU_Write_Byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
		/* Wait 6ms (minimum waiting time for 16 times internal average setup) */
    delay_ms(6); 
    /* enable slave 0 with data_num bytes reading */
    MPU_Write_Byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    delay_ms(2);
}



uint8_t Ist8310_Init(void)
{
	  /* 使能IIC主模式，并重置I2C从模块，并将串行接口仅置于SPI模式*/
    MPU_Write_Byte(MPU6500_USER_CTRL, 0x30);
    delay_ms(10);
	/*IIC主设备时钟配置为400kHz*/
    MPU_Write_Byte(MPU6500_I2C_MST_CTRL, 0x0d); 
    delay_ms(10);

    /* 从1写，从4读 */
    MPU_Write_Byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);  
    delay_ms(10);
    MPU_Write_Byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    delay_ms(10);

    /* IST8310_R_CONFB 0x01 = device rst */
    Ist_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01);
    delay_ms(10);
    if (IST8310_DEVICE_ID_A != Ist_Reg_Read_By_MPU(IST8310_WHO_AM_I))
        return 1;

		/* soft reset */
    Ist_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01); 
    delay_ms(10);

		/* config as ready mode to access register */
    Ist_Reg_Write_By_MPU(IST8310_R_CONFA, 0x00); 
    if (Ist_Reg_Read_By_MPU(IST8310_R_CONFA) != 0x00)
        return 2;
    delay_ms(10);

		/* normal state, no int */
    Ist_Reg_Write_By_MPU(IST8310_R_CONFB, 0x00);
    if (Ist_Reg_Read_By_MPU(IST8310_R_CONFB) != 0x00)
        return 3;
    delay_ms(10);
		
    /* config low noise mode, x,y,z axis 16 time 1 avg */
    Ist_Reg_Write_By_MPU(IST8310_AVGCNTL, 0x24); //100100
    if (Ist_Reg_Read_By_MPU(IST8310_AVGCNTL) != 0x24)
        return 4;
    delay_ms(10);

    /* Set/Reset pulse duration setup,normal mode */
    Ist_Reg_Write_By_MPU(IST8310_PDCNTL, 0xc0);
    if (Ist_Reg_Read_By_MPU(IST8310_PDCNTL) != 0xc0)
        return 5;
    delay_ms(10);

    /* turn off slave1 & slave 4 */
    MPU_Write_Byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    delay_ms(10);
    MPU_Write_Byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    delay_ms(10);

    /* configure and turn on slave 0 */
    Mpu_Master_I2c_Auto_Read_Config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    delay_ms(100);
    return 0;

}

/**
	* @brief  get the offset data of MPU6500
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void Mpu_Offset_Call(void)
{
	int i;
	for (i=0; i<300;i++)
	{
		MPU_Read_Bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

		mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
		mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
		mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];
	
		mpu_data.gx_offset += mpu_buff[8]  << 8 | mpu_buff[9];
		mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
		mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];

		delay_ms(5);
	}
	mpu_data.ax_offset=mpu_data.ax_offset / 300;
	mpu_data.ay_offset=mpu_data.ay_offset / 300;
	mpu_data.az_offset=mpu_data.az_offset / 300;
	mpu_data.gx_offset=mpu_data.gx_offset / 300;
	mpu_data.gy_offset=mpu_data.gx_offset / 300;
	mpu_data.gz_offset=mpu_data.gz_offset / 300;
}

void Get_Ist8310_Data(uint8_t* buff)
{
    MPU_Read_Bytes(MPU6500_EXT_SENS_DATA_00, buff, 6); 
}



/**
	* @brief  initialize imu mpu6500 and magnet meter ist3810
  * @param  
	* @retval 
  * @usage  call in main() function
	*/

uint8_t Mpu_Init(void)
{
	delay_ms(100);

	id                               = MPU_Read_Byte(MPU6500_WHO_AM_I);
	uint8_t i                        = 0;
	uint8_t MPU6500_Init_Data[10][2] = {{ MPU6500_PWR_MGMT_1, 0x80 },     /* Reset Device */ 
																			{ MPU6500_PWR_MGMT_1, 0x03 },     /* Clock Source - Gyro-Z */ 
																			{ MPU6500_PWR_MGMT_2, 0x00 },     /* Enable Acc & Gyro */ 
																			{ MPU6500_CONFIG, 0x04 },         /* LPF 41Hz */ 
																			{ MPU6500_GYRO_CONFIG, 0x18 },    /* +-2000dps */ 
																			{ MPU6500_ACCEL_CONFIG, 0x10 },   /* +-8G */ 
																			{ MPU6500_ACCEL_CONFIG_2, 0x02 }, /* enable LowPassFilter  Set Acc LPF */ 
																			{ MPU6500_USER_CTRL, 0x20 },};    /* Enable AUX */ 
	for (i = 0; i < 10; i++)
	{
		MPU_Write_Byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		delay_ms(1);
	}


	Ist8310_Init();
	
	Mpu_Offset_Call();
	
	return 0;
}

float Get_16Bit_Data(uint8_t addr_h, uint8_t addr_l)
{
	static uint8_t buf[2];
	static short data;
	
  buf[0]=MPU_Read_Byte(addr_l);
  buf[1]=MPU_Read_Byte(addr_h);
  data =	(buf[1]<<8)|buf[0];
	
	return data;   
}

/**
	* @brief  Initialize quaternion
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void Init_Quaternion(void)
{
	int16_t hx, hy;//hz;
	
	hx = imu.mx;
	hy = imu.my;
	//hz = imu.mz;
	
	#ifdef BOARD_DOWN
	if (hx < 0 && hy < 0) 
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = -0.005;
			q1 = -0.199;
			q2 = 0.979;
			q3 = -0.0089;
		}
		else
		{
			q0 = -0.008;
			q1 = -0.555;
			q2 = 0.83;
			q3 = -0.002;
		}
		
	}
	else if (hx < 0 && hy > 0)
	{
		if (fabs(hx / hy)>=1)   
		{
			q0 = 0.005;
			q1 = -0.199;
			q2 = -0.978;
			q3 = 0.012;
		}
		else
		{
			q0 = 0.005;
			q1 = -0.553;
			q2 = -0.83;
			q3 = -0.0023;
		}
		
	}
	else if (hx > 0 && hy > 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0012;
			q1 = -0.978;
			q2 = -0.199;
			q3 = -0.005;
		}
		else
		{
			q0 = 0.0023;
			q1 = -0.83;
			q2 = -0.553;
			q3 = 0.0023;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0025;
			q1 = 0.978;
			q2 = -0.199;
			q3 = 0.008;			
		}
		else
		{
			q0 = 0.0025;
			q1 = 0.83;
			q2 = -0.56;
			q3 = 0.0045;
		}		
	}
	#else
		if (hx < 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.195;
			q1 = -0.015;
			q2 = 0.0043;
			q3 = 0.979;
		}
		else
		{
			q0 = 0.555;
			q1 = -0.015;
			q2 = 0.006;
			q3 = 0.829;
		}
		
	}
	else if (hx < 0 && hy > 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.193;
			q1 = -0.009;
			q2 = -0.006;
			q3 = 0.979;
		}
		else
		{
			q0 = -0.552;
			q1 = -0.0048;
			q2 = -0.0115;
			q3 = 0.8313;
		}
		
	}
	else if (hx > 0 && hy > 0)
	{
		if(fabs((hx / hy)) >= 1)
		{
			q0 = -0.9785;
			q1 = 0.008;
			q2 = -0.02;
			q3 = 0.195;
		}
		else
		{
			q0 = -0.9828;
			q1 = 0.002;
			q2 = -0.0167;
			q3 = 0.5557;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.979;
			q1 = 0.0116;
			q2 = -0.0167;
			q3 = -0.195;			
		}
		else
		{
			q0 = -0.83;
			q1 = 0.014;
			q2 = -0.012;
			q3 = -0.556;
		}		
	}
	#endif
}


/**
	* @brief  get the data of imu
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void MPU_Get_Data(void)
{
    MPU_Read_Bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];
    mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
    mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];
    mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];

    mpu_data.gx = ((mpu_buff[8]  << 8 | mpu_buff[9])  - mpu_data.gx_offset);
    mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
    mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);

    Get_Ist8310_Data(ist_buff);
    memcpy(&mpu_data.mx, ist_buff, 6);

    memcpy(&imu.ax, &mpu_data.ax, 6 * sizeof(int16_t));
	
    imu.temp = 21 + mpu_data.temp / 333.87f;
	  /* 2000dps -> rad/s */
	  imu.wx   = mpu_data.gx / 16.384f / 57.3f; 
    imu.wy   = mpu_data.gy / 16.384f / 57.3f; 
    imu.wz   = mpu_data.gz / 16.384f / 57.3f;
}

/**
  * @brief  fast inverse square-root, to calculate 1/Sqrt(x)
  * @param  x: the number need to be calculated
  * @retval 1/Sqrt(x)
  * @usage  call in imu_ahrs_update() function
  */
float inv_sqrt(float x) 
{
	float halfx = 0.5f * x;
	float y     = x;
	long  i     = *(long*)&y;
	
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	
	return y;
}

/**
	* @brief  update imu AHRS
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void MPU_Ahrs_Update(void) 
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez, halfT;
	float tempq0,tempq1,tempq2,tempq3;

	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;   
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;   

	gx = imu.wx;
	gy = imu.wy;
	gz = imu.wz;
	ax = imu.ax;
	ay = imu.ay;
	az = imu.az;
	mx = imu.mx;
	my = imu.my;
	mz = imu.mz;

	now_update  = imu_tick; //ms, 记得初始化滴答定时器
	halfT       = ((float)(now_update - last_update) / 2000.0f);
	last_update = now_update;
	
	/* Fast inverse square-root */
	norm = inv_sqrt(ax*ax + ay*ay + az*az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	
	#ifdef IST8310
		norm = inv_sqrt(mx*mx + my*my + mz*mz);          
		mx = mx * norm;
		my = my * norm;
		mz = mz * norm; 
	#else
		mx = 0;
		my = 0;
		mz = 0;		
	#endif
	/* compute reference direction of flux */
	hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
	hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
	hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz; 
	
	/* estimated direction of gravity and flux (v and w) */
	vx = 2.0f*(q1q3 - q0q2);
	vy = 2.0f*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
	wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
	wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
	
	/* 
	 * error is sum of cross product between reference direction 
	 * of fields and direction measured by sensors 
	 */
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

	/* PI */
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;	
		ezInt = ezInt + ez * Ki * halfT;
		
		gx = gx + Kp*ex + exInt;
		gy = gy + Kp*ey + eyInt;
		gz = gz + Kp*ez + ezInt;
	}
	
	tempq0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
	tempq1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
	tempq2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
	tempq3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;  

	/* normalise quaternion */
	norm = inv_sqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
	q0 = tempq0 * norm;
	q1 = tempq1 * norm;
	q2 = tempq2 * norm;
	q3 = tempq3 * norm;
}

void MPU_Attitude_Update(void)
{
	/* yaw    -pi----pi */
	imu.yaw = -atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1)* 57.3; 
	/* pitch  -pi/2----pi/2 */
	imu.pit = -asin(-2*q1*q3 + 2*q0*q2)* 57.3;   
	/* roll   -pi----pi  */	
	imu.rol =  atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)* 57.3;
}
