/*
 * msp.c
 *
 *  Created on: Sep 26, 2024
 *      Author: manhcuong
 */
#include "stm32f4xx_hal.h"

void HAL_MspInit(void)
{
	// 1. Set up the priority grouping of the arm cortex mx processor
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	// 2. Enable the required system exceptions of the arm cortex mx processor
	// Enable usage fault, memory fault, bus fault system exceptions
	SCB->SHCSR |= (0x7U << 16); // refer generic device

	// 3. Configure the priority for the system exceptions
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0U, 0U);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0U, 0U);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0U, 0U);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	// Doing the low level init of UART2 peripheral
	GPIO_InitTypeDef gpio_uart;

	// 1. Enable the clock for UART2 peripheral
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	// 2. Do the pin muxing configurations
	gpio_uart.Pin = GPIO_PIN_2;
	gpio_uart.Mode =GPIO_MODE_AF_PP;
	gpio_uart.Pull = GPIO_PULLUP;
	gpio_uart.Speed = GPIO_SPEED_FREQ_LOW;
	gpio_uart.Alternate =  GPIO_AF7_USART2; //UART2_TX
	HAL_GPIO_Init(GPIOA,&gpio_uart);

	gpio_uart.Pin = GPIO_PIN_3; //UART2_RX
	HAL_GPIO_Init(GPIOA,&gpio_uart);

	// 3. Enable the IRQ and set up the priority (NVIC settings)
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_NVIC_SetPriority(USART2_IRQn,15,0);
}
