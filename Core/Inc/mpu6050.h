#ifndef __MPU6050_H
#define __MPU6050_H
#include "main.h"
void MPU6050_Init(void);
void MPU6050_Update(void);
float MPU6050_GetAx(void);
float MPU6050_GetAy(void);
float MPU6050_GetAz(void);
float MPU6050_GetTemperature(void);
float MPU6050_GetGx(void);
float MPU6050_GetGy(void);
float MPU6050_GetGz(void);
void APP_MPU6050_Proc(void);
float MPU6050_GetYaw(void);
float MPU6050_GetPitch(void);
float MPU6050_GetRoll(void);
#endif



