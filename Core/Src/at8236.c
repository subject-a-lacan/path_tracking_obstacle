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
        case 0:
            target_angle = NormalizeAngle180(pianhang - 30.0f);
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

        case 2:
            target_angle = NormalizeAngle180(pianhang);
            pid_pianhang.ErrorInt = 0;
            delay_cnt = 0;
            step = 3;
            break;

        case 3:
            ClosedLoop_Straight(target_angle, 20, tgt_L, tgt_R);
            if (++delay_cnt > 75) {
                step = 4;
            }
            break;

        case 4:
            target_angle = NormalizeAngle180(pianhang + 30.0f);
            pid_pianhang.ErrorInt = 0;
            step = 5;
            break;

        case 5:
            if (ClosedLoop_Turn(target_angle, tgt_L, tgt_R) == 1) {
                step = 6;
            }
            break;

        case 6:
            target_angle = NormalizeAngle180(pianhang);
            pid_pianhang.ErrorInt = 0;
            find_line_cnt = 0;
            step = 7;
            break;

        case 7: {
            uint8_t white_cnt = 0;

            ClosedLoop_Straight(target_angle, 20, tgt_L, tgt_R);

            for (int i = 0; i < 8; i++) {
                if (sensor_digital & (1 << i)) {
                    white_cnt++;
                }
            }

            if (white_cnt < 6) {
                find_line_cnt++;
                if (find_line_cnt >= 3) {
                    step = 0;
                    find_line_cnt = 0;
                    return 1;
                }
            } else {
                find_line_cnt = 0;
            }
            break;
        }
    }

    return 0;
}
