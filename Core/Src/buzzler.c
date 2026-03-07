#include "buzzler.h"
#include "main.h"

void Buzzler_beep(uint16_t duration_ms)
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);
    // 2. 延时指定的时长
    HAL_Delay(duration_ms);
    // 3. 拉低 PA4，三极管截止，蜂鸣器停止
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_RESET);
}