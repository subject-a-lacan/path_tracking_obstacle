#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "main.h"
// 引脚宏定义，方便修改
#define SR04_TRIG_PORT   GPIOA
#define SR04_TRIG_PIN    GPIO_PIN_0

#define SR04_ECHO_PORT   GPIOA
#define SR04_ECHO_PIN    GPIO_PIN_1

void SR04_Init(void);
uint16_t SR04_GetDistance(void);
void SR04_Test(void);


#endif
