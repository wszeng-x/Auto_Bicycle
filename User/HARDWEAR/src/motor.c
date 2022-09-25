#include "motor.h"

int32_t cur_angle_integral;

/*将当前角度范围从0~8191映射到-2^31~+2^31*/
int32_t Motor_Angle_Encoder(Motor_Measure_t* self)
{
	int16_t angle_dev;
	uint8_t first_power_on = 1;
	
	if(first_power_on)
	{
		self->last_angle = self->angle;
		first_power_on = 0;
	}
	if(self->angle - self->last_angle > 4096)  //0->8191突变
	{
		angle_dev = self->angle - self->last_angle - 8192; //突然多了一个周期，要减掉
	}
	else if(self->angle - self->last_angle < -4096) //8191->0突变
	{
		angle_dev = self->angle - self->last_angle + 8192; //突然少了一个周期，要补回
	}
	else
	{
		angle_dev = self->angle - self->last_angle;
	}
	
	self->last_angle = self->angle;  //保存上次角度
	cur_angle_integral += angle_dev;    //角度差积分

	return cur_angle_integral;
}

/* 角度Pid时，在获取tar和cur之后紧接着调用 */
/*将目标值与当前值之间的角度差变化到一个周期范围内*/
void Angle_8191_Over_Zero(float *tar, float *cur)
{
	if(*tar - *cur > 4096)   //4096 ：半圈机械角度
	{
		*cur += 8192;
	}
	else if(*tar - *cur < -4096)
	{
		*cur = *cur - 8192;
	}
	else
	{
		//noting to do 
	}
}
