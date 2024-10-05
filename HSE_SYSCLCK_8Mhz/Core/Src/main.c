/*
 * main.c
 *
 *  Created on: Sep 26, 2024
 *      Author: manhcuong
 */

#include "stm32f4xx_hal.h"
#include "string.h"
#include <stdio.h>

void UART2_Init(void);
void Error_Handler(void);

UART_HandleTypeDef huart2;


char *data = "Application is actually running\r\n";

int main(void)
{
	HAL_Init();
	UART2_Init(); // This UART2 Init will be confiugured in HSI clock (16MHz)

	uint16_t len_of_data = strlen(data);
	HAL_UART_Transmit(&huart2, (uint8_t*)data, len_of_data, HAL_MAX_DELAY);

	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	char msg[100];

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2; // HCLK = 4Mhz
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; // PCLK1 = 2Mhz
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; // PCLK2 = 2Mhz

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) // Change the clock to HSE
	{
		Error_Handler();
	}

	// Disable the HSI clock to save power
	__HAL_RCC_HSI_DISABLE();

	// System tick interrupt after 1ms (HCLK = 4 Mhz)(need 4000 ticks to generate 1ms)
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000U);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK); // Configure the prescaler for Systick is 1

	UART2_Init(); // This UART2 Init will be confiugured in HSE clock (2MHz (hanging in APB bus))

	memset(msg,0,sizeof(msg));
	sprintf(msg,"SYSCLK : %ldHz\r\n",HAL_RCC_GetSysClockFreq());
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

	memset(msg,0,sizeof(msg));
	sprintf(msg,"HCLK   : %ldHz\r\n",HAL_RCC_GetHCLKFreq());
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

	memset(msg,0,sizeof(msg));
	sprintf(msg,"PCLK1  : %ldHz\r\n",HAL_RCC_GetPCLK1Freq());
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

	memset(msg,0,sizeof(msg));
	sprintf(msg,"PCLK2  : %ldHz\r\n",HAL_RCC_GetPCLK2Freq());
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);

	while(1){

	}

	return 0;
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
