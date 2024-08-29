/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>

#include "stm32f411xx.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void)
{
	// Setting pin for green LED (PD12)
	GPIO_PinConfig_t pPD12_IN_LED = {
			GPIO_PIN_NO_12,
			GPIO_MODE_OUT,
			GPIO_SPEED_DEFAULT,
			GPIO_NO_PUPD,
			GPIO_OP_TYPE_PP,
			GPIO_ALT_FN_DEFAULT
	};
	GPIO_Handle_t GPIO_D_Handle_IN_LED = {GPIOD, pPD12_IN_LED};

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIO_D_Handle_IN_LED);

	/**************** SET UP INTERRUPT AT PIN PD5 ****************/


	GPIO_PinConfig_t pPD5_Interrupt = {
			GPIO_PIN_NO_5,
			GPIO_MODE_IT_FT, //falling edge trigger
			GPIO_SPEED_DEFAULT,
			GPIO_PIN_PU, //pull up
			GPIO_OP_TYPE_DEFAULT,
			GPIO_ALT_FN_DEFAULT
	};
	GPIO_Handle_t GPIO_D_Handle_PD15_Interrupt = {GPIOD, pPD5_Interrupt};

	// Init Gpio pin in interrupt falling mode
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIO_D_Handle_PD15_Interrupt);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI5_9, 0xFF);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI5_9, ENABLE);


    /* Loop forever */
	for(;;){

	}
}

void EXTI9_5_IRQHandler(void){
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);

	GPIO_IRQHandling(GPIO_PIN_NO_5);
}
