#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h"

#define ONE_BALL_MECH_ANGLE 3000  //5个坑，减速比19，拨一个球角度：8191*19/5 = 31125.8

extern int32_t cur_angle_integral; 

typedef struct
{
    uint16_t angle;     //转子角度 0~8191
    int16_t speed_rpm;  //转速 额定情况下，3508：469rpm   6020：0-132rpm
    int16_t current;    //电流 额定情况下，3508：10A      6020：1.62A
    uint8_t temperate;  //温度
    int16_t last_angle;   //上次机械角度
} Motor_Measure_t;


#define Calc_Motor_Measure(ptr, rx_message)                                                \
{                                                                                          \
		(ptr)->last_angle = (ptr)->angle;                                                      \
		(ptr)->angle = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);         \
		(ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
		(ptr)->current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);       \
		(ptr)->temperate = (rx_message)->Data[6];                                              \
}

int32_t Motor_Angle_Encoder(Motor_Measure_t* self);
void Angle_8191_Over_Zero(float *tar, float *cur);
#endif /*__MOTOR_H*/
