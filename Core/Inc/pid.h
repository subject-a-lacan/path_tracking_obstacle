#ifndef PID_H
#define PID_H

#include "main.h"
typedef struct {
	float Target;
	float Actual;
	float Out;
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error_now;
	float Error_last;
	float ErrorInt;
	
	float OutMax;
	float OutMin;
    float KdOut;	//不完全微分
} PID_t;
void PID_Update(PID_t *p);

#endif // PID_H

