#include "ultrasonic.h"

// 不占用定时器的权宜之计
// 核心原理：读取 Cortex-M3 内核的 SysTick->VAL 递减寄存器
static void SR04_DelayUs(uint32_t us) {
    uint32_t start_val, current_val, delay_ticks;
    // 计算需要的总滴答数 ( 72MHz, 即 1us = 72 个滴答)
    delay_ticks = us * (SystemCoreClock / 1000000); 
    start_val = SysTick->VAL;
    
    while (1) {
        current_val = SysTick->VAL;
        // 判断 SysTick 是否发生了 1ms 的重装载溢出
        if (current_val < start_val) {
            if ((start_val - current_val) >= delay_ticks) break;
        } else {
            if ((SysTick->LOAD - current_val + start_val) >= delay_ticks) break;
        }
    }
}

/**
 * @brief  超声波模块初始化
 * @note   
 */
void SR04_Init(void) {
    // 确保上电时 Trig 为低电平
    HAL_GPIO_WritePin(SR04_TRIG_PORT, SR04_TRIG_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  获取超声波前方距离 (阻塞式 GPIO 读取，带严格超时保护)
 * @retval 距离 (单位: cm)。返回 999 表示没扫到障碍或异常
 */
uint16_t SR04_GetDistance(void) {
    uint32_t start_time = 0;
    uint32_t end_time = 0;
    uint32_t echo_time_us = 0;
    uint32_t timeout_cnt = 0;

    // 1. 给 Trig 引脚发送至少 10us 的高电平脉冲
    HAL_GPIO_WritePin(SR04_TRIG_PORT, SR04_TRIG_PIN, GPIO_PIN_SET);
    SR04_DelayUs(15); 
    HAL_GPIO_WritePin(SR04_TRIG_PORT, SR04_TRIG_PIN, GPIO_PIN_RESET);

    // 2. 等待 Echo 变高 (发波前摇)
    timeout_cnt = 0;
    while(HAL_GPIO_ReadPin(SR04_ECHO_PORT, SR04_ECHO_PIN) == GPIO_PIN_RESET) {
        timeout_cnt++;
        // 防卡死：如果循环次数过多都没变高，说明线掉了，直接退出
        if (timeout_cnt > 100000) return 999; 
    }

    // 3. Echo 变高了，开始掐表计时！
    start_time = HAL_GetTick(); // 记录毫秒级起点
    uint32_t start_val = SysTick->VAL; // 记录微秒级起点

    // 4. 等待 Echo 变低
    while(HAL_GPIO_ReadPin(SR04_ECHO_PORT, SR04_ECHO_PIN) == GPIO_PIN_SET) {
        // // 只要避障，所以只关心近处！
        // // 假设避障安全距离是 20cm，这里设定最多只等 3 毫秒 (对应距离约 50cm)
        // // 超过 3 毫秒说明前方 50cm 内无障碍，直接强行退出，保护20ms PID 周期
        if ((HAL_GetTick() - start_time) > 3) {
            return 999; 
        }
    }

    // 5. 计算经过的时间并转化为距离
    uint32_t current_val = SysTick->VAL;
    
    // 把时间差统一换算成微秒 (处理 SysTick 重装载逻辑)
    if (current_val < start_val) {
        echo_time_us = (start_val - current_val) / (SystemCoreClock / 1000000);
    } else {
        echo_time_us = (SysTick->LOAD - current_val + start_val) / (SystemCoreClock / 1000000);
    }
    
    // 加上跨越的毫秒数
    echo_time_us += (HAL_GetTick() - start_time) * 1000;

    // 距离(cm) = 时间(us) * 340m/s / 2 / 10000 -> 约等于 时间 / 58
    return (uint16_t)(echo_time_us *(331.4 + 0.607*20)/20000);
}


// ...existing code...

/**
 * @brief  超声波测距测试函数
 * @note   实时将检测到的距离通过串口打印输出
 *         串口重定向在 main.c 中已配置 (printf -> USART1)
 *         在主循环中调用即可，内部含 100ms 采样间隔
 */
void SR04_Test(void) {
    while(1){
    uint16_t distance = SR04_GetDistance();

    if (distance == 999) {
        printf("[SR04] 前方无障碍 或 传感器异常\n");
    } else {
        printf("[SR04] 障碍物距离: %d cm\n", distance);
    }

    HAL_Delay(1000); // 采样间隔 1000ms，避免超声波余震干扰
    
}}
void SR04_Proc(volatile uint16_t* distance){
    PERIODIC(50); // 50ms周期
    *distance = SR04_GetDistance();
}
