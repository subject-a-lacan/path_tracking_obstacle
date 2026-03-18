#include "mpu6050.h"
#include "i2c.h"
#include "main.h"
#include "task.h"
#include "math.h"
#include "qmath.h"
#define MPU6050_ADDR 0xD0   
static float ax, ay, az; // 加速度计的结果，单位g
static float temperature; // 温度计的结果，单位摄氏度
static float gx, gy, gz; // 单位°/s   
static float yaw, pitch, roll; // 欧拉角，单位°
static float gx_offset, gy_offset, gz_offset;

// @简介：向寄存器写值
// @参数 reg - 要写入的寄存器的地址
// @参数 value - 要写入的值
static void MPU6050_WriteReg(uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}


// @简介：读取寄存器的值
// @参数 reg - 要读取的寄存器的地址
// @返回值：表示读取到的值
static uint8_t MPU6050_ReadReg(uint8_t reg) {
    uint8_t res;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 100);
    return res;
}
void MPU6050_Init(void) {
    uint8_t data;

    // 1. 复位芯片
    data = 0x80;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    HAL_Delay(100);

    // 2. 唤醒芯片
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // 3. 配置陀螺仪量程 (±2000 deg/s)
    data = 0x18;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // 4. 配置加速度计量程 (±2g)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

//
//@简介：计算零偏
//10000次采样等待时间大概30-40s，效果稳定两分钟0.5度，1000次采样等待时间大概5-8s，效果不太稳定，有效果好的也有差的，5000次采样等待时间大概10-15s，效果比较好，有效果好的也有差的，10000次采样等待时间大概30-40s，效果稳定两分钟0.5度，
//1000次采样等待时间大概5-8s，效果不太稳定，有效果好的也有差的，
//5000次采样等待时间大概10-15s
void MPU6050_Calibrate(int sample_count) {
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (int i = 0; i < sample_count; i++) {
        // 读取陀螺仪原始数据
        int16_t raw_x = (int16_t)((MPU6050_ReadReg(0x43) << 8) | MPU6050_ReadReg(0x44));
        int16_t raw_y = (int16_t)((MPU6050_ReadReg(0x45) << 8) | MPU6050_ReadReg(0x46));
        int16_t raw_z = (int16_t)((MPU6050_ReadReg(0x47) << 8) | MPU6050_ReadReg(0x48));

        sum_gx += raw_x;
        sum_gy += raw_y;
        sum_gz += raw_z;

        HAL_Delay(1); // 延时等待新数据就绪
    }

    // 计算平均值作为零偏
    gx_offset = sum_gx / sample_count;
    gy_offset = sum_gy / sample_count;
    gz_offset = sum_gz / sample_count;
}

//
// @简介：更新mpu6050的值
//
void MPU6050_Update(void)
{
    // --- 1. 读取并拼接原始数据 ---
    // 使用你封装好的 MPU6050_ReadReg 函数
    int16_t ax_raw = (int16_t)((MPU6050_ReadReg(0x3B) << 8) | MPU6050_ReadReg(0x3C));
    int16_t ay_raw = (int16_t)((MPU6050_ReadReg(0x3D) << 8) | MPU6050_ReadReg(0x3E));
    int16_t az_raw = (int16_t)((MPU6050_ReadReg(0x3F) << 8) | MPU6050_ReadReg(0x40));
    
    int16_t temp_raw = (int16_t)((MPU6050_ReadReg(0x41) << 8) | MPU6050_ReadReg(0x42));
    
    int16_t gx_raw = (int16_t)((MPU6050_ReadReg(0x43) << 8) | MPU6050_ReadReg(0x44));
    int16_t gy_raw = (int16_t)((MPU6050_ReadReg(0x45) << 8) | MPU6050_ReadReg(0x46));
    int16_t gz_raw = (int16_t)((MPU6050_ReadReg(0x47) << 8) | MPU6050_ReadReg(0x48));

    // --- 2. 物理量换算 ---
    // 加速度换算 (±2g 量程: 16384 LSB/g)
    ax = ax_raw * 6.1035e-5f; 
    ay = ay_raw * 6.1035e-5f;
    az = az_raw * 6.1035e-5f;

    // 温度换算 (改为 MPU6050 标准公式)
    // 公式: Temperature in degrees C = (temp_out / 340) + 36.53
    temperature = (float)temp_raw / 340.0f + 36.53f;

    // 陀螺仪换算 (±2000°/s 量程: 16.4 LSB/°/s)
    gx = (gx_raw-gx_offset) * 6.1035e-2f;
    gy = (gy_raw-gy_offset) * 6.1035e-2f;
    gz = (gz_raw-gz_offset) * 6.1035e-2f;
}
//
// @简介：获取x轴向加速度，单位g
//
float MPU6050_GetAx(void){
    return ax;
}
//
// @简介：获取y轴向加速度，单位g
//
float MPU6050_GetAy(void){
    return ay;
}
//
// @简介：获取z轴向加速度，单位g
//
float MPU6050_GetAz(void){
    return az;
}
//
// @简介：获取温度计的值，单位摄氏度
// 
float MPU6050_GetTemperature(void){
    return temperature;
}
//
// @简介：获取绕x轴的角速度，单位°/s
//
float MPU6050_GetGx(void){
    return gx;
}

float MPU6050_GetGy(void){
    return gy;
}

float MPU6050_GetGz(void){
    return gz;
}
//
// @简介：MPU6050的进程函数
//
void APP_MPU6050_Proc(void)
{
    PERIODIC(5); 
    MPU6050_Update(); 

    // --- 1. 陀螺仪积分 (基于你验证正确的符号) ---
    float yaw_g   = yaw   + MPU6050_GetGz() * 0.005f;
    float pitch_g = pitch - MPU6050_GetGx() * 0.005f; // 抬头减小
    float roll_g  = roll  + MPU6050_GetGy() * 0.005f; // 右倾增加

    // --- 2. 加速度计解算 (校准方向必须与上面一致) ---
    
    // 俯仰角：抬头时 ay 变负，atan2(ay, az) 变负，匹配 -Gx。
    float pitch_a = qatan2(-MPU6050_GetAy(), MPU6050_GetAz()) * 180.0f / 3.1415927f;
    
    // 横滚角：右倾时 ax 变负，atan2(-ax, az) 变正，匹配 +Gy。
    float roll_a  = qatan2(-MPU6050_GetAx(), MPU6050_GetAz()) * 180.0f / 3.1415927f;

    // --- 3. 互补滤波器融合 ---
    yaw = yaw_g;
    pitch = 0.95238f * pitch_g + 0.04762f * pitch_a;
    roll  = 0.95238f * roll_g  + 0.04762f * roll_a;
}
float MPU6050_GetYaw(void){
    return yaw;
}
float MPU6050_GetPitch(void){
    return pitch;
}
float MPU6050_GetRoll(void){
    return roll;
}