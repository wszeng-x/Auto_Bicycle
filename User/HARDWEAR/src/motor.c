#include "motor.h"

int32_t cur_angle_integral;

/*����ǰ�Ƕȷ�Χ��0~8191ӳ�䵽-2^31~+2^31*/
int32_t Motor_Angle_Encoder(Motor_Measure_t* self)
{
	int16_t angle_dev;
	uint8_t first_power_on = 1;
	
	if(first_power_on)
	{
		self->last_angle = self->angle;
		first_power_on = 0;
	}
	if(self->angle - self->last_angle > 4096)  //0->8191ͻ��
	{
		angle_dev = self->angle - self->last_angle - 8192; //ͻȻ����һ�����ڣ�Ҫ����
	}
	else if(self->angle - self->last_angle < -4096) //8191->0ͻ��
	{
		angle_dev = self->angle - self->last_angle + 8192; //ͻȻ����һ�����ڣ�Ҫ����
	}
	else
	{
		angle_dev = self->angle - self->last_angle;
	}
	
	self->last_angle = self->angle;  //�����ϴνǶ�
	cur_angle_integral += angle_dev;    //�ǶȲ����

	return cur_angle_integral;
}

/* �Ƕ�Pidʱ���ڻ�ȡtar��cur֮������ŵ��� */
/*��Ŀ��ֵ�뵱ǰֵ֮��ĽǶȲ�仯��һ�����ڷ�Χ��*/
void Angle_8191_Over_Zero(float *tar, float *cur)
{
	if(*tar - *cur > 4096)   //4096 ����Ȧ��е�Ƕ�
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
