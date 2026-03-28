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
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, left_pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    } else {
        // 反转：AIN1 输 0，AIN2 输 PWM (传入绝对值)
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -left_pwm);
    }

    // 4. 右轮控制逻辑
    if (right_pwm >= 0) {
        // 正转：BIN1 输 PWM，BIN2 输 0
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, right_pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    } else {
        // 反转：BIN1 输 0，BIN2 输 PWM (传入绝对值)
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -right_pwm);
    }
}

/**
 * @brief  电机测试函数，设置左右轮 PWM 并每1s发送轮速
 * @param  left_pwm:  左轮 PWM（-999 ~ 999）
 * @param  right_pwm: 右轮 PWM（-999 ~ 999）
 */
void Motor_Test(int16_t left_pwm, int16_t right_pwm) {
    // Motor_Init();
    Motor_SetPWM(left_pwm, right_pwm);
    // while (1) {
    //     float left_speed  = Calc_Physical_Speed(Read_Encoder_Left());
    //     float right_speed = Calc_Physical_Speed(Read_Encoder_Right());
    //     printf("L:%.3f m/s  R:%.3f m/s\r\n", left_speed, right_speed);
    //     HAL_Delay(1000);
    // }
}
/**
  * @brief  非阻塞式避障机动函数 (专供 20ms 定时器中断调用)
  * @param  left_pwm:  左轮目标PWM的指针
  * @param  right_pwm: 右轮目标PWM的指针
  * @param  digital_val: 灰度传感器8位数字量 (用于视觉回归检测)
  * @param  reset_flag: 外部复位标志位 (传入1强制清零状态机，传入0正常运行)
  * @retval 1: 避障彻底结束(找到线了) | 0: 正在避障中
  */
uint8_t Avoidance_Run(int16_t *left_pwm, int16_t *right_pwm, uint8_t digital_val, uint8_t reset_flag) 
{
    static uint8_t avoid_step = 0;   // 记录当前到哪一步了
    static uint16_t tick_cnt = 0;    // 记录当前动作执行了多少个 20ms

    // =========================================================
    // 状态机外部强制复位: 防止被意外打断后留下脏数据
    // =========================================================
    if (reset_flag == 1) {
        avoid_step = 0;
        tick_cnt = 0;
        flag_avoid_reset=0;
        return 0; // 复位完直接退出
    }

    tick_cnt++; // 时间节拍 +1

    switch (avoid_step) 
    {
        case 0: 
            // 动作一：原地右转躲避障碍
            *left_pwm = 400;     // 左轮正转
            *right_pwm = -400;   // 右轮反转
            
            // 转
            if (tick_cnt >= 12) { 
                avoid_step = 1;  
                tick_cnt = 0;  
            }
            break;

        case 1: 
            // 动作二：直行越过障碍物
            *left_pwm = 400; 
            *right_pwm = 400;
            
            // 直走
            if (tick_cnt >= 86) { 
                avoid_step = 2;
                tick_cnt = 0;
            }
            break;

        case 2: 
            // 动作三：向左前方画弧线，准备切回赛道
            *left_pwm = 150;      // 左轮慢
            *right_pwm = 400;    // 右轮快，向左拐
            
            // : 只要眼睛看到黑线，立刻结束！
            if (digital_val != 0xFF) { 
                avoid_step = 0;  
                tick_cnt = 0;
                return 1;        // 报告：避障彻底完成！
            }
            
            // 防跑飞：如果转了 3s 还没看到线，强行结束
            if (tick_cnt >= 150) {
                avoid_step = 0;
                tick_cnt = 0;
                return 1; 
            }
            break;
    }
    
    return 0; // 仍在避障中
}
// Avoidance_Run 的测试函数 中断函数里avoidance把速度设为PWM 但是传给速度环的必须是脉冲数 对不上 这是测试函数
//用于把avoidance_run里的PWM数字改成脉冲
void Avoidance_Speed_Test(void) 
{
    // 清空一下前期的杂乱脉冲
    Read_Encoder_Left();
    Read_Encoder_Right();

    while(1) 
    {
        HAL_Delay(20); // 严格死等 20ms，模拟定时器中断的周期
        
        // 读取这 20ms 内产生的真实脉冲数 
        int16_t pulses_L = Read_Encoder_Left();
        int16_t pulses_R = Read_Encoder_Right();
        
        // 打印出来
        printf("脉冲/20ms: 右=%d, 左=%d\r\n", pulses_L, pulses_R);
    }
}