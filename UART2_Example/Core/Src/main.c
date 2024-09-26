/*
 * main.c
 *
 *  Created on: Sep 26, 2024
 *      Author: manhcuong
 */

#include "stm32f1xx_hal.h"

void SystemClockConfig(void);

int main(void)
{
	HAL_Init();

	SystemClockConfig();

	return 0;
}

void SystemClockConfig(void)
{

}
