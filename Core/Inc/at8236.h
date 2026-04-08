//注：ARR的值是999 因此设置占空比要乘10
#ifndef __AT8236_H
#define __AT8236_H

#include "main.h"
#include "i2c.h"
#include "encoder.h"
extern TIM_HandleTypeDef htim1;


void Motor_Init(void);
void Motor_SetPWM(int16_t left_pwm, int16_t right_pwm);
void Motor_Test(int16_t left_pwm, int16_t right_pwm);
uint8_t ClosedLoop_Turn(float target_yaw, int16_t *tgt_L, int16_t *tgt_R);
uint8_t ClosedLoop_Straight(float target_yaw, int16_t base_spd, int16_t *tgt_L, int16_t *tgt_R);
uint8_t Avoidance_Run(int16_t *tgt_L, int16_t *tgt_R, uint8_t reset, uint8_t sensor_digital) ;
void Avoidance_Speed_Test( void );
#endif
