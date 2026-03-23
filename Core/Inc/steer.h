#ifndef STEER_H
#define STEER_H

#include "main.h"
#include "tim.h"

uint32_t Servo_AngleToDuty(uint16_t angle);
void Steer_SetAngle(uint16_t angle);
void Steer_Stop(void);

#endif 
