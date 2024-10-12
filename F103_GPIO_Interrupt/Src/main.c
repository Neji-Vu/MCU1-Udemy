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

#define ZeroPointFiveSecond 250000
#define OneSeconde 			500000
#define TowSeconde 			1000000

#define IOPCEN_BIT			4U

#define RCC_BASE_ADDR		0x40021000
#define RCC_APB2ENR			(RCC_BASE_ADDR + 0x18)

#define GPIOC_BASE_ADDR		0x40011000
#define GPIOC_CRL			(GPIOC_BASE_ADDR + 0x00)
#define GPIOC_CRH			(GPIOC_BASE_ADDR + 0x04)
#define GPIOC_BSRR			(GPIOC_BASE_ADDR + 0x10)
#define GPIOC_BRR			(GPIOC_BASE_ADDR + 0x14)
#define GPIOC_ODR			(GPIOC_BASE_ADDR + 0x0C)
#define GPIOC_IDR			(GPIOC_BASE_ADDR + 0x08)

#define AFIO_BASE_ADDR		0x40010000
#define AFIO_MAPR			(AFIO_BASE_ADDR + 0x04)

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void delay(uint32_t tick){
	for(uint32_t i = 0U; i < tick; ++i){

	}
}

void EnableSWDDebug(void){
	// Enable RCC for GPIOA to use PA13 PA14
	*(uint32_t*)RCC_APB2ENR |= (1U << 2U);
	// Enable RCC for AFIO
	*(uint32_t*)RCC_APB2ENR |= (1U << 0U);

	// Change the PA13 PA14 to SWD configuration
	*(uint32_t*)AFIO_MAPR |= (1U << 25U);
}

void TurnLedOn(uint32_t PinNo){
	*(uint32_t*)GPIOC_BSRR |= (0x1U << (PinNo + 16));
}

void TurnLedOff(uint32_t PinNo){
	*(uint32_t*)GPIOC_BSRR |= (0x1U << PinNo);
}

void ToggleLed(uint32_t PinNo){
	*(uint16_t*)GPIOC_ODR ^= (0x1U << PinNo);
}

uint8_t ReadBit(uint32_t PinNo){
	return (*(uint16_t*)GPIOC_IDR >> PinNo) & 1U;
}

void SetPC14AsInputPullDown(void){
	// Set mode for PC14 as Button input pin
	*(uint32_t*)GPIOC_CRH &= ~(0xFU << 24U);
	*(uint32_t*)GPIOC_CRH |= (0x2U << 26U);

	// Set PC14 as input pull down
	*(uint32_t*)GPIOC_BSRR |= (0x1U << (14U+16U));
}

void PressToTurnLedOff(void){
	if(ReadBit(14U)){
		// Turn On LED
		TurnLedOff(13);
	}
	else{
		// Turn On LED
		TurnLedOn(13);
	}
}

void ConfigPC14AsEXTIPin(void){
	// Set as raising edge
}

int main(void)
{
	// Enable RCC to generate the clock
	*(uint32_t*)RCC_APB2ENR |= (1U << IOPCEN_BIT);

	/* For SWD Debug */
	EnableSWDDebug();

	// Set Mode for PC13 as LED output pin
	*(uint32_t*)GPIOC_CRH &= ~(0xFU << 20U);
	*(uint32_t*)GPIOC_CRH |= (0x2U << 20U);

	// Set mode for PC14 as Button input pull downpin
	SetPC14AsInputPullDown();

    /* Loop forever */
	while(1)
	{
		PressToTurnLedOff();
	}
}
