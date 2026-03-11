//注：ARR的值是999 因此设置占空比要乘10
#include "at8236.h"
#include "encoder.h"
#include <stdio.h>

/**
 * @brief  初始化并启动电机 PWM 输出
 * @note   在 main 函数的 USER CODE BEGIN 2 中调用一次即可
 */
void Motor_Init(void) {
    // 启动 TIM1 的四个 PWM 通道
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // PA8  -> AIN2
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // PA9  -> AIN1
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // PA10 -> BIN1
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // PA11 -> BIN2
}

/**
 * @brief  设置左右电机的 PWM 占空比及方向
 * @param  left_pwm: 左电机控制量 (-1000 到 1000)
 * @param  right_pwm: 右电机控制量 (-1000 到 1000)
 */
void Motor_SetPWM(int16_t left_pwm, int16_t right_pwm) {
    
    // 1. 左轮安全限幅 (防止超出 ARR=999 导致定时器溢出异常)
    if (left_pwm > 999)  left_pwm = 999;
    if (left_pwm < -999) left_pwm = -999;
    
    // 2. 右轮安全限幅
    if (right_pwm > 999)  right_pwm = 999;
    if (right_pwm < -999) right_pwm = -999;

    // 3. 左轮控制逻辑 (正数正转，负数反转)
    if (left_pwm >= 0) {
        // 正转：AIN1 输 PWM，AIN2 输 0
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, left_pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    } else {
        // 反转：AIN1 输 0，AIN2 输 PWM (传入绝对值)
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -left_pwm);
    }

    // 4. 右轮控制逻辑
    if (right_pwm >= 0) {
        // 正转：BIN1 输 PWM，BIN2 输 0
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, right_pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    } else {
        // 反转：BIN1 输 0，BIN2 输 PWM (传入绝对值)
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -right_pwm);
    }
}

/**
 * @brief  电机测试函数，设置左右轮 PWM 并每1s发送轮速
 * @param  left_pwm:  左轮 PWM（-999 ~ 999）
 * @param  right_pwm: 右轮 PWM（-999 ~ 999）
 */
void Motor_Test(int16_t left_pwm, int16_t right_pwm) {
    Motor_Init();
    Motor_SetPWM(left_pwm, right_pwm);
    while (1) {
        float left_speed  = Calc_Physical_Speed(Read_Encoder_Left());
        float right_speed = Calc_Physical_Speed(Read_Encoder_Right());
        printf("L:%.3f m/s  R:%.3f m/s\r\n", left_speed, right_speed);
        HAL_Delay(1000);
    }
}
void Motor_Test_IO(void) {
    Motor_Init();
    // PA9 高电平
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    // PA8 低电平
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    // PA10 高电平
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    // PA11 低电平
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    
}