/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "mpu6050_test.h"
#include "stdio.h"
#include "lora.h"
#include "buzzler.h"
#include "ultrasonic.h"
#include "at8236.h"
#include "encoder.h"
#include "gray.h"
#include "pid.h"
#include "task.h"
#include "steer.h"
#include "IMU.h"
//hywhywhyw
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 定义PID结构体变量:循迹
   target:0
   actual:track_error 由CalculateNormalizedValue得出
   out:左右轮差速补偿
*/

PID_t yaw={
    .Target = 0,
    .Actual = 0,
    .Out = 0,
    .Kp = 1.0f,
    .Ki = 0.1f,
    .Kd = 0.1f,
    .Error_now = 0,
    .Error_last = 0,
    .ErrorInt = 0,
    .OutMax = 1000,
    .OutMin = -1000,
    .KdOut = 0,
};    
/*
  定义PID结构体变量：左轮速度
  target:期望速度
  actual:left_speed 由Left_Speed_Proc给出
  Out:PWM
*/
PID_t speed_L={
    .Target = 0,
    .Actual = 0,
    .Out = 0,
    .Kp = 1.0f,
    .Ki = 0.1f,
    .Kd = 0.1f,
    .Error_now = 0,
    .Error_last = 0,
    .ErrorInt = 0,
    .OutMax = 1000,
    .OutMin = -1000,
    .KdOut = 0,
};  

PID_t speed_R={
    .Target = 0,
    .Actual = 0,
    .Out = 0,
    .Kp = 1.0f,
    .Ki = 0.1f,
    .Kd = 0.1f,
    .Error_now = 0,
    .Error_last = 0,
    .ErrorInt = 0,
    .OutMax = 1000,
    .OutMin = -1000,
    .KdOut = 0,
};
PID_t error={
    .Target = 0,
    .Actual = 0,
    .Out = 0,
    .Kp = 1.0f,
    .Ki = 0.1f,
    .Kd = 0.1f,
    .Error_now = 0,
    .Error_last = 0,
    .ErrorInt = 0,
    .OutMax = 1000,
    .OutMin = -1000,
    .KdOut = 0,
};   // 定义PID结构体变量：速度差

// 定义小车运行状态的枚举类型
typedef enum {
    CAR_STATE_TRACKING = 0,    // 循迹状态（默认/核心状态）
    CAR_STATE_LOST_LINE_GO = 1,// 丢线直行状态
    CAR_STATE_OBSTACLE_AVOID = 2// 避障状态
} CarState;
// volatile CarState car_state = CAR_STATE_TRACKING; // 初始化为循迹状态
volatile CarState car_state=CAR_STATE_LOST_LINE_GO;//PID调参测试专用

//超声波+速度传感
volatile int16_t  track_error = 0;    // 灰度传感器计算出的偏航偏差
volatile uint16_t front_distance = 999; // 超声波前方距离（cm）
volatile int16_t  left_speed = 0;     // 左轮当前速度（编码器反馈）
volatile int16_t  right_speed = 0;    // 右轮当前速度（编码器反馈）

//灰度传感器
unsigned short Anolog[8] = {0};      // 存储当前模拟量值的数组
unsigned short white[8] = {1155,2009,1528,2344,2320,1757,1721,1420}; // 存储白色校准值的数组 
unsigned short black[8] = {67,72,72,72,73,74,73,64};     // 存储黑色校准值的数组
unsigned short Normal[8];          // 归一化值数组
No_MCU_Sensor sensor;              // 传感器数据结构体
unsigned char Digtal;							 // 数字量

volatile uint8_t flag_avoid_done = 0;  // 接收中断避障完成的信息
volatile uint8_t flag_avoid_reset = 0; // 发送给中断的命令：复位避障步骤

uint8_t rx_byte = 0;    // 缓存命令
uint8_t rx_state = 0;   // 接收状态机：0代表等帧头，1代表等指令
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1,HAL_MAX_DELAY);
    return ch;
}
void StateMachine_Update(void) 
{   
    PERIODIC(15); // 15ms周期调用一次状态机更新函数
    // 定义静态防抖计数器，函数退出后值不会消失
    static uint8_t lost_line_cnt = 0;
    static uint8_t find_line_cnt = 0;
    static uint8_t obstacle_cnt = 0;
    

    switch (car_state) 
    {
        // ==========================================
        // 状态 1：正常循迹
        // ==========================================
        case CAR_STATE_TRACKING:
            // 优先级最高：判断是否需要避障 (小于20cm)
            // if (front_distance < 20) {
            //     if (++obstacle_cnt > 3) {  // 连续3次确认，防抖
            //         car_state = CAR_STATE_OBSTACLE_AVOID; 
                    
            //         // 通知中断层的避障函数复位内部静态变量，准备执行新的避障！
            //         flag_avoid_reset = 1; 
                    
            //         obstacle_cnt = 0;      
            //     }
            // } 
            // else {
            //     obstacle_cnt = 0; 
                
            //     // 次优先级：判断是否丢线 (0xFF代表8个灯全白)
            //     if (sensor.Digtal == 0xFF) { 
            //         if (++lost_line_cnt > 5) { // 连续5次全白，确认丢线
            //             car_state = CAR_STATE_LOST_LINE_GO; 
                        
            //             // 清零直行同步环的历史误差，防止切入直行瞬间抽搐
            //             error.ErrorInt = 0;
            //             error.Error_last = 0;
            //             error.KdOut = 0;
                        
            //             lost_line_cnt = 0;
            //         }
            //     } else {
            //         lost_line_cnt = 0; 
            //     }
            // }
            break;

        // ==========================================
        // 状态 2：丢线直行 (通过双轮编码器同步 PID 保持直走)
        // ==========================================
        case CAR_STATE_LOST_LINE_GO:
            // // 防撞！
            // if (front_distance < 20) {
            //     if (++obstacle_cnt > 4) {
            //         car_state = CAR_STATE_OBSTACLE_AVOID;
                    
            //         // 同样需要通知中断层复位避障动作
            //         flag_avoid_reset = 1; 
                    
            //         obstacle_cnt = 0;
            //     }
            // } 
            // else {
            //     obstacle_cnt = 0;
                
            //     // 判断是否重新踩到了黑线
            //     if (sensor.Digtal != 0xFF) { // 只要不是全白
            //         if (++find_line_cnt > 4) { // 连续确认4次，防抖
            //             car_state = CAR_STATE_TRACKING; // 成功找回黑线，切回循迹
                        
            //             // 为接下来的循迹转向环清除数据
            //             yaw.ErrorInt = 0;
            //             yaw.Error_last = 0;
            //             yaw.KdOut = 0;
            //             speed_L.ErrorInt = 0;
            //             speed_L.Error_last = 0; 
            //             speed_L.KdOut = 0;
            //             speed_R.ErrorInt = 0; 
            //             speed_R.Error_last = 0;
            //             speed_R.KdOut = 0;
            //             find_line_cnt = 0;
            //         }
            //     } else {
            //         find_line_cnt = 0;
            //     }
            // }
            break;

        // ==========================================
        // 状态 3：避障模式
        // ==========================================
        case CAR_STATE_OBSTACLE_AVOID:
            // // 此时底层定时器中断正在高频调用 Avoidance_Run 控制电机
            
            // if (flag_avoid_done == 1) { 
                
            //     // 避障彻底完成，严格按照逻辑直接切回循迹状态！
            //     car_state = CAR_STATE_TRACKING; 
                
            //     // 避障刚结束车身大概率有偏角，必须清零转向 PID 重新平滑切入赛道
            //     yaw.ErrorInt = 0;
            //     yaw.Error_last = 0;
            //     yaw.KdOut = 0;
                
            //     // 收起捷报，清空信箱，为下一次避障做准备
            //     flag_avoid_done = 0; 
            // }
            break;
    }
}
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
// {
//     if (htim->Instance == TIM1) 
//     {
//         static uint8_t pid_cnt = 0;
//         pid_cnt++;
        
//         // 软件分频：满 20 次即为 20ms 的绝对稳定控制周期
//         if (pid_cnt >= 20) 
//         {
//             pid_cnt = 0;
            
//             // 局部变量，存放算出来的“目标期望速度”
//             int16_t target_L = 0; 
//             int16_t target_R = 0;
//             int16_t base_speed = 300; // 基础直行期望速度，后续可调
            
//             // ==========================================================
//             // 第一步：外环决策  - 根据状态计算目标速度
//             // ==========================================================
//             switch (car_state) 
//             {
//                 case 0: // CAR_STATE_TRACKING (循迹模式)
//                     // 1. 动态降速：误差越大，基础速度越慢 (安全过弯)
//                     // base_speed = 300 - (33 * abs(track_error) / 1024);
//                     if (base_speed < 100) base_speed = 100; // 兜底最低速度
                    
//                     // 2. 转向环 PID 计算
//                     yaw.Target = 0;
//                     yaw.Actual = track_error;
//                     PID_Update(&yaw);
                    
//                     // 3. 差速分配给左右轮目标速度
//                     target_L = base_speed + (int16_t)yaw.Out;
//                     target_R = base_speed - (int16_t)yaw.Out;
//                     break;

//                 case 1: // CAR_STATE_LOST_LINE_GO (丢线直行)
//                     // 1. 同步环 PID 计算 (让左右轮速度差为 0)
//                     error.Target = 0;
//                     error.Actual = left_speed - right_speed; // 实际差速
//                     PID_Update(&error);
                    
//                     // 2. 补偿分配给左右轮
//                     target_L = base_speed + (int16_t)error.Out;
//                     target_R = base_speed - (int16_t)error.Out;
//                     break;
                    

//                 case 2: // CAR_STATE_OBSTACLE_AVOID (避障机动)
//                     // 1. 拦截主函数发来的复位
//                     if (flag_avoid_reset == 1) {
//                         Avoidance_Run(&target_L, &target_R, sensor.Digtal, 1);
//                         flag_avoid_reset = 0; // 执行完复位，重置标志位
//                     }
//                     // 2. 正常执行避障非阻塞状态机
//                     else {
//                         // 注意：这里传出的 target_L 和 target_R 直接作为目标速度送给内环
//                         if (Avoidance_Run(&target_L, &target_R, sensor.Digtal, 0) == 1) {
//                             flag_avoid_done = 1; // 升起捷报，通知主循环切回循迹
//                         }
//                     }
//                     break;
//             }

//             // ==========================================================
//             // 第二步：内环执行 
//             // ==========================================================
            
//             // 1. 计算左轮速度环 PID
//             speed_L.Target = target_L;
//             speed_L.Actual = Read_Encoder_Left(); // 编码器读回来的真实速度
//             PID_Update(&speed_L);
            
//             // 2. 计算右轮速度环 PID
//             speed_R.Target = target_R;
//             speed_R.Actual = Read_Encoder_Right(); 
//             PID_Update(&speed_R);
            
//             // ==========================================================
//             // 第三步：硬件输出 - 经过你封装接口发给 AT8236
//             // ==========================================================
//             // 因为写的是位置式 PID，算出来的 Out 直接就是 PWM 占空比
//             Motor_SetPWM((int16_t)speed_L.Out, (int16_t)speed_R.Out);
//         }
//     }
// }
// 串口调参执行函数
// 传入参数 cmd: 串口接收到的 1~24 的数字 (以十六进制/HEX格式发送)
void UART_PID_Tune(uint8_t cmd) 
{
    float step = 0.1f; // 每次增减的步长
    switch(cmd) 
    {
        // ================= yaw (循迹转向) =================
        case 'a':  yaw.Kp += step; printf("yaw Kp = %.2f\r\n", yaw.Kp); break;
        case 'b':  yaw.Kp -= step; printf("yaw Kp = %.2f\r\n", yaw.Kp); break;
        case 'c':  yaw.Ki += step; printf("yaw Ki = %.2f\r\n", yaw.Ki); break;
        case 'd':  yaw.Ki -= step; printf("yaw Ki = %.2f\r\n", yaw.Ki); break;
        case 'e':  yaw.Kd += step; printf("yaw Kd = %.2f\r\n", yaw.Kd); break;
        case 'f':  yaw.Kd -= step; printf("yaw Kd = %.2f\r\n", yaw.Kd); break;

        // ================= speed_L (左轮速度) =================
        case 'g':  speed_L.Kp += step; printf("speed_L Kp = %.2f\r\n", speed_L.Kp); break;
        case 'h':  speed_L.Kp -= step; printf("speed_L Kp = %.2f\r\n", speed_L.Kp); break;
        case 'i':  speed_L.Ki += step; printf("speed_L Ki = %.2f\r\n", speed_L.Ki); break;
        case 'j':  speed_L.Ki -= step; printf("speed_L Ki = %.2f\r\n", speed_L.Ki); break;
        case 'k':  speed_L.Kd += step; printf("speed_L Kd = %.2f\r\n", speed_L.Kd); break;
        case 'l':  speed_L.Kd -= step; printf("speed_L Kd = %.2f\r\n", speed_L.Kd); break;

        // ================= speed_R (右轮速度) =================
        case 'm': speed_R.Kp += step; printf("speed_R Kp = %.2f\r\n", speed_R.Kp); break;
        case 'n': speed_R.Kp -= step; printf("speed_R Kp = %.2f\r\n", speed_R.Kp); break;
        case 'o': speed_R.Ki += step; printf("speed_R Ki = %.2f\r\n", speed_R.Ki); break;
        case 'p': speed_R.Ki -= step; printf("speed_R Ki = %.2f\r\n", speed_R.Ki); break;
        case 'q': speed_R.Kd += step; printf("speed_R Kd = %.2f\r\n", speed_R.Kd); break;
        case 'r': speed_R.Kd -= step; printf("speed_R Kd = %.2f\r\n", speed_R.Kd); break;

        // ================= error (丢线直行同步) =================
        case 's': error.Kp += step; printf("error Kp = %.2f\r\n", error.Kp); break;
        case 't': error.Kp -= step; printf("error Kp = %.2f\r\n", error.Kp); break;
        case 'u': error.Ki += step; printf("error Ki = %.2f\r\n", error.Ki); break;
        case 'v': error.Ki -= step; printf("error Ki = %.2f\r\n", error.Ki); break;
        case 'w': error.Kd += step; printf("error Kd = %.2f\r\n", error.Kd); break;
        case 'x': error.Kd -= step; printf("error Kd = %.2f\r\n", error.Kd); break;
        
        default: break; // 其他不理会
    }
}
//回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) // 确认是串口2发来的
    {
        switch (rx_state) 
        {
            case 0: // 【状态0：等帧头】
                if (rx_byte == 'C') { // 假设帧头是字符大写的 'C'
                    rx_state = 1;     // 收到帧头了！状态机切到状态1
                }
                // 如果收到的不是 'C'，状态机还是 0，这个错误字节直接被无视
                break;
                
            case 1: // 【状态1：接收指令并执行】
                UART_PID_Tune(rx_byte); // 把收到的 1~24 的数字丢进去执行
                rx_state = 0;           // 极其关键：执行完立刻复位，重新等下一个 'C'
                break;
        }
        
        // 重新开启中断接收下一个字节
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

void IMU_Test(void)
{
    float angles[3];
    PERIODIC(100); // 100ms周期打印一次
    IMU_getYawPitchRoll(angles);
    printf("%.2f,%.2f,%.2f\r\n", angles[0], angles[1], angles[2]);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //调用init函数
  Motor_Init();
  // MPU6050_Init(); // IMU_init() internally calls MPU6050_Init()
  SR04_Init();
  Encoder_Init();
  No_MCU_Ganv_Sensor_Init(&sensor,white,black); 
  HAL_TIM_Base_Start_IT(&htim1);  // 开启 TIM1 的定时器中断
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);// 开启 USART2 的接收中断，准备接收调参命令
  // ESP8266_Init("F521F520","f521f520","192.168.100.15","8080");   //这是Gong的
  ESP8266_Init("F521F520","f521f520","192.168.100.14","8080");   //这是Xu的
  Steer_SetAngle(90);
  IMU_init();
  // HAL_Delay(10);
  // Steer_Stop();
  // gray_test();+
  // Motor_Test(500, 500);
  // Motor_Test_IO();
  // MPU6050_Test();
  // MPU6050_EularAngleTest();
  // Lora_Test();
  // Encoder_Test();
  // SR04_Test();
  // Buzzler_beep_Test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    IMU_Test();
     printf("he yi wei!?\r\n");
    HAL_Delay(1000);

    //更新传感器数据 (每个Proc函数都调用了PERIODIC宏，用于实现伪并行)
    // Left_Speed_Proc(&left_speed);    这两行代码是大错特错 读取速度必须在中断函数里 否则数据不是实时的！
    // Right_Speed_Proc(&right_speed); 
    // SR04_Proc(&front_distance);
    // Gray_Proc(&sensor, Normal, &track_error);
    // Digtal=Get_Digtal_For_User(&sensor); 
    // StateMachine_Update(); // 根据当前传感器数据和状态机逻辑更新小车状态
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //下面是利用vofa调试时需要的代码
        // PERIODIC(1000);
        // printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
        //     error.Actual, error.Target, error.Out,
        //     yaw.Actual, yaw.Target, yaw.Out,
        //     speed_L.Actual, speed_L.Target, speed_L.Out,
        //     speed_R.Actual, speed_R.Target, speed_R.Out,
        //     error.Kp, error.Ki, error.Kd,
        //     yaw.Kp, yaw.Ki, yaw.Kd,
        //     speed_L.Kp, speed_L.Ki, speed_L.Kd,
        //     speed_R.Kp, speed_R.Ki, speed_R.Kd);
    // 串口发送数据的 1 2 3是error的actual target out 4,5,6是yaw的actual target out 7,8,9是speed_L的actual target out
    // 10 11 12 是speed_R的actual target out 13 14 15是error的KP KI KD 16 17 18是yaw的KP KI KD 19 20 21是speed_L的KP KI KD 22 23 24是speed_R的KP KI KD
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
