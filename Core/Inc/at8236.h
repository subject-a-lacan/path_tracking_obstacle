//注：ARR的值是999 因此设置占空比要乘10
#ifndef __AT8236_H
#define __AT8236_H

#include "main.h"
#include "i2c.h"

extern TIM_HandleTypeDef htim1;


void Motor_Init(void);
void Motor_SetPWM(int16_t left_pwm, int16_t right_pwm);
void Motor_Test(int16_t left_pwm, int16_t right_pwm);
void Motor_Test_IO(void);
#endif
