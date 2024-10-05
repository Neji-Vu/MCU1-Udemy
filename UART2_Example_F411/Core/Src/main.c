/*
 * main.c
 *
 *  Created on: Sep 26, 2024
 *      Author: manhcuong
 */

#include "stm32f4xx_hal.h"
#include "string.h"

void SystemClockConfig(void);
void UART2_Init(void);
void Error_Handler(void);

UART_HandleTypeDef huart2;


char *data = "Application is actually running\r\n";

int main(void)
{
	HAL_Init();
	SystemClockConfig();
	UART2_Init();

	uint16_t len_of_data = strlen(data);
	HAL_UART_Transmit(&huart2, (uint8_t*)data, len_of_data, HAL_MAX_DELAY);

	while(1){

	}

	return 0;
}

void SystemClockConfig(void)
{

}

void UART2_Init(void)
{
	  huart2.Instance = USART2;
	  huart2.Init.BaudRate = 115200;
	  huart2.Init.WordLength = UART_WORDLENGTH_8B;
	  huart2.Init.StopBits = UART_STOPBITS_1;
	  huart2.Init.Parity = UART_PARITY_NONE;
	  huart2.Init.Mode = UART_MODE_TX_RX;
	  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	  if (HAL_UART_Init(&huart2) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void Error_Handler(void)
{
	while(1);
}
