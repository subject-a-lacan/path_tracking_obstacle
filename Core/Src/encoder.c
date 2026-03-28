#include "encoder.h"
#include <stdio.h>

//@brief：注意！20是调控周期，如果你改了定时器的周期，这个系数也要改！
// 系数 = (PI * 0.048) / (13 * 4 * 20) / 0.02 = 0.00724981f
#define PULSE_TO_SPEED_COEF  0.00724981f


/**
 * @brief  初始化并启动编码器定时器
 * @note   左电机接 TIM4 (PB6, PB7)，右电机接 TIM3 (PB4, PB5)
 */
void Encoder_Init(void) {
    // 启动左轮编码器定时器 (TIM4)
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    
    // 启动右轮编码器定时器 (TIM3)
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

/**
 * @brief  读取左电机编码器脉冲数并清零 (配合周期定时器使用)
 * @retval 增量脉冲数 (有符号 16 位整数)  这样子即使翻转突变65535 返回的也是正确的负值
 */
int16_t Read_Encoder_Right(void) {
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);    //由于代码在中断函数里执行 因此返回的是20ms内的增量脉冲数
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    return -count;
}

/**
 * @brief  读取右电机编码器脉冲数并清零 (配合周期定时器使用)
 * @retval 增量脉冲数 (有符号 16 位整数) 不然可能会爆
 */
int16_t Read_Encoder_Left(void) {
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    return count;
}
float Calc_Physical_Speed(int16_t pulse_count) {
    // 直接用脉冲数乘以预先算好的常数系数
    return (float)pulse_count * PULSE_TO_SPEED_COEF;
}

void Encoder_Test(void) {
    Encoder_Init();
    while (1) {
        float left_speed  = Calc_Physical_Speed(Read_Encoder_Left());
        float right_speed = Calc_Physical_Speed(Read_Encoder_Right());
        printf("L:%.3f m/s  R:%.3f m/s\r\n", left_speed, right_speed);
        HAL_Delay(1000);
    }
}
void Left_Speed_Proc(volatile int16_t* left_speed){
    PERIODIC(20); // 20ms周期
    *left_speed = Read_Encoder_Left();
}
void Right_Speed_Proc(volatile int16_t* right_speed){
    PERIODIC(20); // 20ms周期
    *right_speed = Read_Encoder_Right();
}
