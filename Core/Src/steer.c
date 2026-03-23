#include "steer.h"

#define STEER_MIN_ANGLE     0
#define STEER_MAX_ANGLE     180

uint32_t Steer_AngleToDuty(uint16_t angle)
{
    // 角度范围保护
    if(angle < STEER_MIN_ANGLE) angle = STEER_MIN_ANGLE;
    if(angle > STEER_MAX_ANGLE) angle = STEER_MAX_ANGLE;
    
    // 计算对应占空比
    return (uint32_t)((angle / 180.0f) * 2000 + 500);
}

/*
 * @brief  控制舵机转到指定角度
 * @param  angle: 目标角度(0-180°)
 * @retval 无
 */
void Steer_SetAngle(uint16_t angle)
{
    uint32_t duty = Steer_AngleToDuty(angle);
    // 设置PWM占空比并启动输出
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, duty);
}

/*
 * @brief  停止舵机PWM输出
 * @retval 无
 */
void Steer_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);
}
