#include "lora.h"
#include "main.h"
#include <stdio.h>
#include <usart.h>

/** 
 * @brief  Initialize ESP8266
 * @param  ssid:WiFi的ssid
 * @param  pwd:WiFi的密码
 * @param  ip:你本机的ip
 * @param  port：端口号
 * @retval None
 */
void ESP8266_Init(char *ssid, char *pwd, char *ip, char *port) {
    char buf[128];

    // 1. 设置模式为 Station 并重启
    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CWMODE=1\r\n", 13, 100);
    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+RST\r\n", 8, 100);
    HAL_Delay(1000); // 重启等待

    // 2. 连接 WiFi 
    sprintf(buf, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pwd);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
    HAL_Delay(1000);

    // 3. 设置单连接模式
    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CIPMUX=0\r\n", 13, 100);
    HAL_Delay(100);

    // 4. 建立 TCP 连接 
    sprintf(buf, "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", ip, port);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
    HAL_Delay(500);

    // 5. 开启透传模式并开始发送
    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CIPMODE=1\r\n", 14, 100);
    HAL_Delay(100);
    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CIPSEND\r\n", 12, 100);
    HAL_Delay(100); 
}