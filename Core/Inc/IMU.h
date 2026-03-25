#ifndef __IMU_H
#define __IMU_H

#include "stm32f1xx_hal.h"
#include "math.h"
#define M_PI        (float)3.1415926535

typedef struct
{
    float x;
    float y;
    float z;
} xyz_f_t;
extern xyz_f_t north,west;
extern volatile float imu_yaw_values[5];   //处理航向的增值
extern float motion6[7];

/* 函数原型声明 */
uint8_t IMU_init(void);
void IMU_getValues(float * values);
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void IMU_getQ(float * q);
void IMU_getYawPitchRoll(float * angles);
void calGyroVariance(float data[], int length, float sqrResult[], float avgResult[]);
void IMU_TT_getgyro(float * zsjganda);
float invSqrt1(float x);

#endif
