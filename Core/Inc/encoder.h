#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "task.h"
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void Encoder_Init(void);
int16_t Read_Encoder_Left(void);
int16_t Read_Encoder_Right(void);
float Calc_Physical_Speed(int16_t pulse_count);
void Encoder_Test(void);
void Left_Speed_Proc(int16_t* left_speed);
void Right_Speed_Proc(int16_t* right_speed);
#endif
