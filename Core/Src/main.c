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
volatile CarState car_state = CAR_STATE_TRACKING; // 初始化为循迹状态

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
/* USER CODE END PV */
void StateMachine_Update(void) 
{
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
            if (front_distance < 20) {
                if (++obstacle_cnt > 3) {  // 连续3次确认，防抖
                    car_state = CAR_STATE_OBSTACLE_AVOID; 
                    
                    // 通知中断层的避障函数复位内部静态变量，准备执行新的避障！
                    flag_avoid_reset = 1; 
                    
                    obstacle_cnt = 0;      
                }
            } 
            else {
                obstacle_cnt = 0; 
                
                // 次优先级：判断是否丢线 (0xFF代表8个灯全白)
                if (sensor.Digtal == 0xFF) { 
                    if (++lost_line_cnt > 5) { // 连续5次全白，确认丢线
                        car_state = CAR_STATE_LOST_LINE_GO; 
                        
                        // 清零直行同步环的历史误差，防止切入直行瞬间抽搐
                        error.ErrorInt = 0;
                        error.Error_last = 0;
                        error.KdOut = 0;
                        
                        lost_line_cnt = 0;
                    }
                } else {
                    lost_line_cnt = 0; 
                }
            }
            break;

        // ==========================================
        // 状态 2：丢线直行 (通过双轮编码器同步 PID 保持直走)
        // ==========================================
        case CAR_STATE_LOST_LINE_GO:
            // 防撞！
            if (front_distance < 20) {
                if (++obstacle_cnt > 4) {
                    car_state = CAR_STATE_OBSTACLE_AVOID;
                    
                    // 同样需要通知中断层复位避障动作
                    flag_avoid_reset = 1; 
                    
                    obstacle_cnt = 0;
                }
            } 
            else {
                obstacle_cnt = 0;
                
                // 判断是否重新踩到了黑线
                if (sensor.Digtal != 0xFF) { // 只要不是全白
                    if (++find_line_cnt > 4) { // 连续确认4次，防抖
                        car_state = CAR_STATE_TRACKING; // 成功找回黑线，切回循迹
                        
                        // 为接下来的循迹转向环清除数据
                        yaw.ErrorInt = 0;
                        yaw.Error_last = 0;
                        yaw.KdOut = 0;
                        speed_L.ErrorInt = 0;
                        speed_L.Error_last = 0; 
                        speed_L.KdOut = 0;
                        speed_R.ErrorInt = 0; 
                        speed_R.Error_last = 0;
                        speed_R.KdOut = 0;
                        find_line_cnt = 0;
                    }
                } else {
                    find_line_cnt = 0;
                }
            }
            break;

        // ==========================================
        // 状态 3：避障模式
        // ==========================================
        case CAR_STATE_OBSTACLE_AVOID:
            // 此时底层定时器中断正在高频调用 Avoidance_Run 控制电机
            
            if (flag_avoid_done == 1) { 
                
                // 避障彻底完成，严格按照逻辑直接切回循迹状态！
                car_state = CAR_STATE_TRACKING; 
                
                // 避障刚结束车身大概率有偏角，必须清零转向 PID 重新平滑切入赛道
                yaw.ErrorInt = 0;
                yaw.Error_last = 0;
                yaw.KdOut = 0;
                
                // 收起捷报，清空信箱，为下一次避障做准备
                flag_avoid_done = 0; 
            }
            break;
    }
}
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
  MPU6050_Init();
  SR04_Init();
  Encoder_Init();
  No_MCU_Ganv_Sensor_Init(&sensor,white,black); 
  // gray_test();
  // Motor_Test(500, 500);
  // Motor_Test_IO();
  // MPU6050_Test();
  MPU6050_EularAngleTest();
  // Lora_Test();
  // Encoder_Test();
  // SR04_Test();
  // Buzzler_beep_Test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // printf("What does heyiwei mean?\r\n");
    // HAL_Delay(1000);
    //更新传感器数据 (每个Proc函数都调用了PERIODIC宏，用于实现伪并行)
    Left_Speed_Proc(&left_speed);
    Right_Speed_Proc(&right_speed); 
    SR04_Proc(&front_distance);
    Gray_Proc(&sensor, Normal, &track_error);
    Digtal=Get_Digtal_For_User(&sensor); 
    StateMachine_Update(); // 根据当前传感器数据和状态机逻辑更新小车状态
    /* USER CODE END WHILE */
  
    /* USER CODE BEGIN 3 */
    //下面是利用vofa调试时需要的代码
    //printf("%.2f, %.2f, %.2f\r\n", yaw.Actual, speed.Actual, error.Actual);
    //printf("%.2f, %.2f, %.2f\r\n", yaw.Out, speed.Out, error.Out);
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
