#include "pid.h"
#define filter 0.1	//微分滤波系数，取值范围为0~1，值越大微分滤波效果越明显，但响应速度越慢
/**
  * 函    数：PID计算及结构体变量值更新
  * 参    数：PID_t * 指定结构体的地址
  * 返 回 值：无
  */
void PID_Update(PID_t *p)
{
	/*获取本次误差和上次误差*/
	p->Error_last = p->Error_now;					//获取上次误差
	p->Error_now = p->Target - p->Actual;		//获取本次误差，目标值减实际值，即为误差值
	
	/*外环误差积分（累加）*/
	/*如果Ki不为0，才进行误差积分，这样做的目的是便于调试*/
	/*因为在调试时，我们可能先把Ki设置为0，这时积分项无作用，误差消除不了，误差积分会积累到很大的值*/
	/*后续一旦Ki不为0，那么因为误差积分已经积累到很大的值了，这就导致积分项疯狂输出，不利于调试*/
	if (p->Ki != 0)					//如果Ki不为0
	{
		p->ErrorInt += p->Error_now;	//进行误差积分
        /*积分限幅*/
	    if (p->ErrorInt > (1000/p->Ki)) {p->ErrorInt = 1000/p->Ki;}		//限制误差积分最大为1000/Ki
	    if (p->ErrorInt < -(1000/p->Ki)) {p->ErrorInt = -(1000/p->Ki);}		//限制误差积分最小为-1000/Ki
	}
	else							//否则
	{
		p->ErrorInt = 0;			//误差积分直接归0
	}
    /*不完全微分*/
    float Kdout_now=(1-filter)*p->Kd*(p->Error_now - p->Error_last) + filter*p->KdOut;	//计算本次微分项输出，使用一阶低通滤波对微分项进行滤波
    p->KdOut = Kdout_now;
	/*PID计算*/
	/*使用位置式PID公式，计算得到输出值*/
	p->Out = p->Kp * p->Error_now
		   + p->Ki * p->ErrorInt
		   + p->KdOut;
	
	/*输出限幅*/
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}	//限制输出值最大为结构体指定的OutMax
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}	//限制输出值最小为结构体指定的OutMin
}