#include "mpu6050.h"
#include "i2c.h"
#include "main.h"
#include "task.h"
#include "math.h"
#include "qmath.h"
#define MPU6050_ADDR 0xD0   
static short acc_offset[3] = {0, 0, 0};
static short gyro_offset[3] = {0, 0, 0};

// @简介：向寄存器写值
// @参数 reg - 要写入的寄存器的地址
// @参数 value - 要写入的值
// @返回值：表示写入操作是否成功，0表示成功，非0表示失败
static char MPU6050_WriteReg(uint8_t reg, uint8_t value) {
    uint8_t ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
    return (char)ret;
}


// @简介：读取寄存器的值
// @参数 reg - 要读取的寄存器的地址
// @返回值：表示读取到的值
static uint8_t MPU6050_ReadReg(uint8_t reg) {
    uint8_t res = 0;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 100);
    return res;
}

// @简介：设置陀螺仪量程
// @参数 scale - 0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
// @返回值：表示设置操作是否成功，0表示成功，非0表示失败
uint8_t MPU_Set_Gyro_Scale(uint8_t scale) {
    uint8_t data=scale << 3; // 将量程值左移3位，放置在寄存器的正确位置
    return MPU6050_WriteReg(0x1B, data); 
}

// @简介：设置加速度量程
// @参数 scale - 0,±2g;1,±4g;2,±8g;3,±16g
// @返回值：表示设置操作是否成功，0表示成功，非0表示失败
uint8_t MPU_Set_Accel_Scale(uint8_t Scale){
    uint8_t data=Scale << 3; // 将量程值左移3位，放置在寄存器的正确位置
    return MPU6050_WriteReg(0x1C, data); 
}

// @简介：设置低通滤波频率
// @参数 lpf - 低通滤波频率，单位Hz
// @返回值：表示设置操作是否成功，0表示成功，非0表示失败
uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data = 0;
    if(lpf >= 188) data = 1;
    else if(lpf >= 98) data = 2;
    else if(lpf >= 42) data = 3;
    else if(lpf >= 20) data = 4;
    else if(lpf >= 10) data = 5;
    else data = 6;
    return MPU6050_WriteReg(0X1A, data);
}

// @简介：设置采样率
// @参数 rate - 采样率，单位Hz
// @返回值：表示设置操作是否成功，0表示成功，非0表示失败
uint8_t MPU_Set_Sample_Rate(uint16_t rate)
{
    uint8_t data = 0;
    if(rate > 1000) rate = 1000;
    if(rate < 4) rate = 4;
    data = 1000 / rate - 1;
    // 先设置采样率，再设置LPF
    uint8_t ret1 = MPU6050_WriteReg(0X19,data);
    uint8_t ret2 = MPU_Set_LPF(rate / 2);
    // 任意一步失败则返回失败
    return (ret1 == 0 && ret2 == 0) ? 0 : 1;
}

// @简介：读取角速度数据
// @参数: gyroData - 存储角速度数据的数组，长度至少为3，分别对应x、y、z轴
void MPU6050ReadGyro(uint16_t *gyroData)
{   
    int16_t raw_x = (int16_t)((MPU6050_ReadReg(0x43) << 8) | MPU6050_ReadReg(0x44));
    int16_t raw_y = (int16_t)((MPU6050_ReadReg(0x45) << 8) | MPU6050_ReadReg(0x46));
    int16_t raw_z = (int16_t)((MPU6050_ReadReg(0x47) << 8) | MPU6050_ReadReg(0x48));
    gyroData[0] = raw_x-gyro_offset[0]; // 减去零偏
    gyroData[1] = raw_y-gyro_offset[1];
    gyroData[2] = raw_z-gyro_offset[2];
}

// @简介：读取加速度数据
// @参数: accData - 存储加速度数据的数组，长度至少为3，分别对应x、y、z轴
void MPU6050ReadAcc(uint16_t *accData){
    int16_t raw_x = (int16_t)((MPU6050_ReadReg(0x3B) << 8) | MPU6050_ReadReg(0x3C));
    int16_t raw_y = (int16_t)((MPU6050_ReadReg(0x3D) << 8) | MPU6050_ReadReg(0x3E));
    int16_t raw_z = (int16_t)((MPU6050_ReadReg(0x3F) << 8) | MPU6050_ReadReg(0x40));
    accData[0] = raw_x-acc_offset[0]; // 减去零偏
    accData[1] = raw_y-acc_offset[1];
    accData[2] = raw_z; // 加速度z轴不减零偏，因为静止时z轴应该有1g的加速度
}

// @简介：获取温度数据
// @返回值：表示温度值，单位摄氏度
float MPU6050GetTemp(void){
    int16_t temp_raw = (int16_t)((MPU6050_ReadReg(0x41) << 8) | MPU6050_ReadReg(0x42));
    return (float)temp_raw / 340.0f + 36.53f; // 温度换算公式
}


uint8_t MPU6050_Init(void) {
    uint8_t data;
    // 1. 复位芯片
    data = 0x80;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    HAL_Delay(5);

    // 2. 唤醒芯片
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // 3. 配置陀螺仪量程 (±2000 deg/s)
    data = 0x18;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // 4. 配置加速度计量程 (±2g)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    MPU_Set_Sample_Rate(1000);// 设置采样率为1000Hz，LPF自动设置为500Hz
    
    // 5. 关闭中断、FIFO，配置INT引脚
    data=0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0X38, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0X6A, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0X23, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    data=0x80;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0X37, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    // 6.电源管理
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, 100); 
    data=0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0X6C, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    MPU_Set_Sample_Rate(1000);// 设置采样率为1000Hz，LPF自动设置为500Hz
    
    HAL_Delay(10);

    long long gyro_sum[3] = {0, 0, 0}; // 用long long避免溢出
    long long acc_sum[3] = {0, 0, 0};
    uint16_t valid_cnt = 0;            // 有效采样次数
    
    for(uint16_t i = 0; i < 500; i++)
    {   uint16_t temp_acc[3] = {0};
        uint16_t temp_gyro[3] = {0};
        MPU6050ReadGyro(temp_gyro);    // 读取单次陀螺仪数据
        MPU6050ReadAcc(temp_acc);
        // 过滤无效数据（读取失败则跳过）
        if(temp_gyro[0] != 0 || temp_gyro[1] != 0 || temp_gyro[2] != 0||temp_acc[0] != 0 || temp_acc[1] != 0 || temp_acc[2] != 0)        {
            gyro_sum[0] += temp_gyro[0];
            gyro_sum[1] += temp_gyro[1];
            gyro_sum[2] += temp_gyro[2];
              acc_sum[0] += temp_acc[0];
            acc_sum[1] += temp_acc[1];
            acc_sum[2] += (temp_acc[2] - 16384); // Z轴理论值为16384（±2g量程）
            valid_cnt++;
        }
        HAL_Delay(10); // 10ms采样一次（对应100Hz采样率）
    }
    
    // 计算零漂平均值（有效次数为0则校准失败）
    if(valid_cnt > 0)
    {
        gyro_offset[0] = (uint16_t)(gyro_sum[0] / valid_cnt);
        gyro_offset[1] = (uint16_t)(gyro_sum[1] / valid_cnt);
        gyro_offset[2] = (uint16_t)(gyro_sum[2] / valid_cnt);
        acc_offset[0] = (uint16_t)(acc_sum[0] / valid_cnt);
        acc_offset[1] = (uint16_t)(acc_sum[1] / valid_cnt);
        acc_offset[2] = (uint16_t)(acc_sum[2] / valid_cnt);
       
    }
    else
    {
        acc_offset[0] = acc_offset[1] = acc_offset[2] = 0;
        gyro_offset[0] = gyro_offset[1] = gyro_offset[2] = 0;
    }
    return 0;

}


// //
// // @简介：更新mpu6050的值
// //
// void MPU6050_Update(void)
// {
//     // --- 1. 读取并拼接原始数据 ---
//     // 使用你封装好的 MPU6050_ReadReg 函数
//     int16_t ax_raw = (int16_t)((MPU6050_ReadReg(0x3B) << 8) | MPU6050_ReadReg(0x3C));
//     int16_t ay_raw = (int16_t)((MPU6050_ReadReg(0x3D) << 8) | MPU6050_ReadReg(0x3E));
//     int16_t az_raw = (int16_t)((MPU6050_ReadReg(0x3F) << 8) | MPU6050_ReadReg(0x40));
    
//     int16_t temp_raw = (int16_t)((MPU6050_ReadReg(0x41) << 8) | MPU6050_ReadReg(0x42));
    
//     int16_t gx_raw = (int16_t)((MPU6050_ReadReg(0x43) << 8) | MPU6050_ReadReg(0x44));
//     int16_t gy_raw = (int16_t)((MPU6050_ReadReg(0x45) << 8) | MPU6050_ReadReg(0x46));
//     int16_t gz_raw = (int16_t)((MPU6050_ReadReg(0x47) << 8) | MPU6050_ReadReg(0x48));

//     // --- 2. 物理量换算 ---
//     // 加速度换算 (±2g 量程: 16384 LSB/g)
//     ax = ax_raw * 6.1035e-5f; 
//     ay = ay_raw * 6.1035e-5f;
//     az = az_raw * 6.1035e-5f;

//     // 温度换算 (改为 MPU6050 标准公式)
//     // 公式: Temperature in degrees C = (temp_out / 340) + 36.53
//     temperature = (float)temp_raw / 340.0f + 36.53f;

//     // 陀螺仪换算 (±2000°/s 量程: 16.4 LSB/°/s)
//     gx = (gx_raw-gx_offset) * 6.1035e-2f;
//     gy = (gy_raw-gy_offset) * 6.1035e-2f;
//     gz = (gz_raw-gz_offset) * 6.1035e-2f;
// }
// //
// // @简介：获取x轴向加速度，单位g
// //
// float MPU6050_GetAx(void){
//     return ax;
// }
// //
// // @简介：获取y轴向加速度，单位g
// //
// float MPU6050_GetAy(void){
//     return ay;
// }
// //
// // @简介：获取z轴向加速度，单位g
// //
// float MPU6050_GetAz(void){
//     return az;
// }
// //
// // @简介：获取温度计的值，单位摄氏度
// // 
// float MPU6050_GetTemperature(void){
//     return temperature;
// }
// //
// // @简介：获取绕x轴的角速度，单位°/s
// //
// float MPU6050_GetGx(void){
//     return gx;
// }

// float MPU6050_GetGy(void){
//     return gy;
// }

// float MPU6050_GetGz(void){
//     return gz;
// }
// //
// // @简介：MPU6050的进程函数
// //
// void APP_MPU6050_Proc(void)
// {
//     PERIODIC(5); 
//     MPU6050_Update(); 

//     // --- 1. 陀螺仪积分 (基于你验证正确的符号) ---
//     float yaw_g   = yaw   + MPU6050_GetGz() * 0.005f;
//     float pitch_g = pitch - MPU6050_GetGx() * 0.005f; // 抬头减小
//     float roll_g  = roll  + MPU6050_GetGy() * 0.005f; // 右倾增加

//     // --- 2. 加速度计解算 (校准方向必须与上面一致) ---
    
//     // 俯仰角：抬头时 ay 变负，atan2(ay, az) 变负，匹配 -Gx。
//     float pitch_a = qatan2(-MPU6050_GetAy(), MPU6050_GetAz()) * 180.0f / 3.1415927f;
    
//     // 横滚角：右倾时 ax 变负，atan2(-ax, az) 变正，匹配 +Gy。
//     float roll_a  = qatan2(-MPU6050_GetAx(), MPU6050_GetAz()) * 180.0f / 3.1415927f;

//     // --- 3. 互补滤波器融合 ---
//     yaw = yaw_g;
//     pitch = 0.95238f * pitch_g + 0.04762f * pitch_a;
//     roll  = 0.95238f * roll_g  + 0.04762f * roll_a;
// }
// float MPU6050_GetYaw(void){
//     return yaw;
// }
// float MPU6050_GetPitch(void){
//     return pitch;
// }
// float MPU6050_GetRoll(void){
//     return roll;
// }
