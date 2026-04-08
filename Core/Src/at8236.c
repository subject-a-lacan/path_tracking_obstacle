#include "at8236.h"
#include "encoder.h"
#include <stdio.h>
#include "pid.h"
#include "main.h"
#include "math.h"

extern PID_t pid_pianhang;
extern volatile float pianhang;

static float NormalizeAngle180(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void Motor_SetPWM(int16_t left_pwm, int16_t right_pwm)
{
    if (left_pwm > 999) {
        left_pwm = 999;
    }
    if (left_pwm < -999) {
        left_pwm = -999;
    }

    if (right_pwm > 999) {
        right_pwm = 999;
    }
    if (right_pwm < -999) {
        right_pwm = -999;
    }

    if (left_pwm >= 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, left_pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -left_pwm);
    }

    if (right_pwm >= 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, right_pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -right_pwm);
    }
}

uint8_t ClosedLoop_Turn(float target_yaw, int16_t *tgt_L, int16_t *tgt_R)
{
    float turn_error = NormalizeAngle180(target_yaw - pianhang);

    pid_pianhang.Target = turn_error;
    pid_pianhang.Actual = 0.0f;
    PID_Update(&pid_pianhang);

    *tgt_L = 20 + (int16_t)pid_pianhang.Out;
    *tgt_R = 20 - (int16_t)pid_pianhang.Out;

    if (fabsf(turn_error) <= 3.0f) {
        *tgt_L = 0;
        *tgt_R = 0;
        return 1;
    }

    return 0;
}

uint8_t ClosedLoop_Straight(float target_yaw, int16_t base_spd, int16_t *tgt_L, int16_t *tgt_R)
{
    float heading_error = NormalizeAngle180(target_yaw - pianhang);

    pid_pianhang.Target = heading_error;
    pid_pianhang.Actual = 0.0f;
    PID_Update(&pid_pianhang);

    *tgt_L = base_spd + (int16_t)pid_pianhang.Out;
    *tgt_R = base_spd - (int16_t)pid_pianhang.Out;
    return 0;
}

// uint8_t Avoidance_Run(int16_t *tgt_L, int16_t *tgt_R, uint8_t reset, uint8_t sensor_digital)
// {
//     static uint8_t step = 0;
//     static float target_angle = 0.0f;
//     static uint16_t delay_cnt = 0;
//     static uint8_t find_line_cnt = 0;

//     if (reset) {
//         step = 0;
//         find_line_cnt = 0;
//         return 0;
//     }

//     switch (step) {
//         case 0:
//             target_angle = NormalizeAngle180(pianhang - 40.0f);
//             pid_pianhang.ErrorInt = 0;
//             pid_pianhang.KdOut = 0;
//             pid_pianhang.Error_last = 0;
//             step = 1;
//             break;

//         case 1:
//             if (ClosedLoop_Turn(target_angle, tgt_L, tgt_R) == 1) {
//                 step = 2;
//             }
//             break;

//         case 2:
//             target_angle = NormalizeAngle180(pianhang);
//             pid_pianhang.ErrorInt = 0;
//             delay_cnt = 0;
//             step = 3;
//             break;

//         case 3:
//             ClosedLoop_Straight(target_angle, 20, tgt_L, tgt_R);
//             if (++delay_cnt > 400) {
//                 step = 4;
//             }
//             break;

//         case 4:
//             target_angle = NormalizeAngle180(pianhang + 80.0f);
//             pid_pianhang.ErrorInt = 0;
//             step = 5;
//             break;

//         case 5:
//             if (ClosedLoop_Turn(target_angle, tgt_L, tgt_R) == 1) {
//                 step = 6;
//             }
//             break;

//         case 6:
//             target_angle = NormalizeAngle180(pianhang);
//             pid_pianhang.ErrorInt = 0;
//             find_line_cnt = 0;
//             step = 7;
//             break;

//         case 7: {
//             uint8_t white_cnt = 0;

//             ClosedLoop_Straight(target_angle, 20, tgt_L, tgt_R);

//             for (int i = 0; i < 8; i++) {
//                 if (sensor_digital & (1 << i)) {
//                     white_cnt++;
//                 }
//             }

//             if (white_cnt < 6) {
//                 find_line_cnt++;
//                 if (find_line_cnt >= 3) {
//                     step = 0;
//                     find_line_cnt = 0;
//                     return 1;
//                 }
//             } else {
//                 find_line_cnt = 0;
//             }
//             break;
//         }
//     }

//     return 0;
// }

uint8_t Avoidance_Run(int16_t *tgt_L, int16_t *tgt_R, uint8_t reset, uint8_t sensor_digital)
{
    static uint8_t step = 0;
    static float target_angle = 0.0f;
    static uint16_t delay_cnt = 0;
    static uint8_t find_line_cnt = 0;

    if (reset) {
        step = 0;
        find_line_cnt = 0;
        return 0;
    }

    switch (step) {
        // ================= 第1阶段：左转 40 度躲避 =================
        case 0:
            target_angle = NormalizeAngle180(pianhang - 40.0f);
            pid_pianhang.ErrorInt = 0;
            pid_pianhang.KdOut = 0;
            pid_pianhang.Error_last = 0;
            step = 1;
            break;

        case 1:
            if (ClosedLoop_Turn(target_angle, tgt_L, tgt_R) == 1) {
                step = 2;
            }
            break;

        // ================= 第2阶段：斜着直行，彻底拉开横向距离 (关键修复!) =================
        case 2:
            target_angle = pianhang; // 锁定当前的斜向角度
            pid_pianhang.ErrorInt = 0;
            delay_cnt = 0;           // 清零定时器
            step = 3;
            break;

        case 3:
            ClosedLoop_Straight(target_angle, 20, tgt_L, tgt_R); // 维持斜向走
            // 【调整这里】：延时计数，1次=20ms。200次就是 4秒。
            // 如果撞到了障碍物的左下角，就把这里的 200 往上加（比如 250 或 300）！
            if (++delay_cnt > 120) {  
                step = 4;
            }
            break;

        // ================= 第3阶段：右转 40 度，让车身平行于赛道 =================
        case 4:
            target_angle = NormalizeAngle180(pianhang + 40.0f); 
            pid_pianhang.ErrorInt = 0;
            pid_pianhang.KdOut = 0;
            pid_pianhang.Error_last = 0;
            step = 5;
            break;

        case 5:
            if (ClosedLoop_Turn(target_angle, tgt_L, tgt_R) == 1) {
                step = 6;
            }
            break;

        // ================= 第4阶段：平行直行，越过障碍物 =================
        case 6:
            target_angle = pianhang; // 锁定平行角度
            pid_pianhang.ErrorInt = 0;
            delay_cnt = 0;
            step = 7;
            break;

        case 7:
            ClosedLoop_Straight(target_angle, 20, tgt_L, tgt_R);
            // 越过障碍物的时间。180次 = 3.6秒。如果过早切回撞到障碍物屁股，就把180加大！
            if (++delay_cnt > 180) { 
                step = 8;
            }
            break;

        // ================= 第5阶段：向右猛打方向，准备切回赛道 =================
        case 8:
            target_angle = NormalizeAngle180(pianhang + 40.0f); // 转大角度狠狠切回赛道
            pid_pianhang.ErrorInt = 0;
            pid_pianhang.KdOut = 0;
            pid_pianhang.Error_last = 0;
            step = 9;
            break;

        case 9:
            if (ClosedLoop_Turn(target_angle, tgt_L, tgt_R) == 1) {
                step = 10;
            }
            break;

        // ================= 第6阶段：直行找线闭环 =================
        case 10:
            target_angle = pianhang; // 锁定切线角度
            pid_pianhang.ErrorInt = 0;
            find_line_cnt = 0;
            step = 11;
            break;

        case 11: {
            uint8_t white_cnt = 0;

            ClosedLoop_Straight(target_angle, 20, tgt_L, tgt_R);

            for (int i = 0; i < 8; i++) {
                if (sensor_digital & (1 << i)) {
                    white_cnt++;
                }
            }

            if (white_cnt <= 6) { // 压到线了
                find_line_cnt++;
                if (find_line_cnt >= 3) {
                    step = 0;
                    find_line_cnt = 0;
                    return 1; // 彻底避障完成
                }
            } else {
                find_line_cnt = 0;
            }
            break;
        }
    }

    return 0; 
}