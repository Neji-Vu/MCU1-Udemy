/*
 * it.c
 *
 *  Created on: Sep 26, 2024
 *      Author: manhcuong
 */

#include "stm32f4xx_hal.h"

void SysTick_Handler(void)
{
	HAL_IncTick();

	HAL_SYSTICK_IRQHandler();
}
