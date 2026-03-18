#include "mpu6050_test.h" 
#include "mpu6050.h"     
#include "task.h"
void MPU6050_Test(void)
{
    // 1. 初始化 MPU6050
    // 注意：串口初始化已经在 main.c 里由 CubeMX 生成的 MX_USARTx_Init 完成了
    MPU6050_Init();
    
    printf("MPU6050 HAL Test Start...\r\n");

    while(1)
    {
        // 2. 触发传感器读取和数据换算
        MPU6050_Update();
        
        // 3. 通过你封装好的 Getter 函数获取实际数值
        float ax = MPU6050_GetAx();
        float ay = MPU6050_GetAy();
        float az = MPU6050_GetAz();
        
        float temp = MPU6050_GetTemperature();
        
        float gx = MPU6050_GetGx();
        float gy = MPU6050_GetGy();
        float gz = MPU6050_GetGz();
        
        // 4. 使用标准的 printf 进行打印
        // 格式：加速度x,y,z, 温度, 陀螺仪x,y,z
        printf("%f,%f,%f,%f,%f,%f,%f\r\n",  ax, ay, az, temp, gx, gy, gz);
        // 5. HAL 库标准延时 10ms
        HAL_Delay(100);
    }
}



// 内部使用的串口打印进程
static void USART2_Proc(void)
{
    // 每 10ms 执行一次发送
    PERIODIC(10)
    
    // 1. 获取所有最新的物理量数据
    float ax = MPU6050_GetAx();
    float ay = MPU6050_GetAy();
    float az = MPU6050_GetAz();
    
    float temp = MPU6050_GetTemperature();
    
    float gx = MPU6050_GetGx();
    float gy = MPU6050_GetGy();
    float gz = MPU6050_GetGz();
    
    float yaw   = MPU6050_GetYaw();
    float pitch = MPU6050_GetPitch();
    float roll  = MPU6050_GetRoll();
    
    // 2. 按照 VOFA+ 的“JustFloat”或 CSV 格式打印数据
    // 注意：由于重定向已完成，直接用 printf 就会发往 USART2
    printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", ax, ay, az, temp, gx, gy, gz, yaw, pitch, roll);
}

//`
// @简介：用来测试欧拉角（VOFA）
//
void MPU6050_EularAngleTest(void) 
{
    MPU6050_Init();
    MPU6050_Calibrate(5000);
    while(1)
    {
        // 2并行执行两个任务
        
        // 任务 A：每 5ms 更新一次欧拉角计算
        APP_MPU6050_Proc();
        
        // 任务 B：每 10ms 向电脑上报一次数据
        USART2_Proc();
    }
}
