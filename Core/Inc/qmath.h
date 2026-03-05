#ifndef __QMATH_H__
#define __QMATH_H__

/* 移植关键：替换为 F1 系列 HAL 库头文件 */
#include "stm32f1xx_hal.h" 
#include "math.h"
#include <stdint.h>
#include "main.h"


// 快速查表函数接口
float qsin(float x);
float qcos(float x);
float qtan(float x);
float qasin(float x);
float qacos(float x);
float qatan(float x);
float qatan2(float y, float x);

#endif