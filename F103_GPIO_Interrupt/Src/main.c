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
#define OneSecond 			500000
#define TowSecond 			1000000

#define IOPCEN_BIT			4U

#define RCC_BASE_ADDR		0x40021000
#define RCC_CR				(RCC_BASE_ADDR + 0x00)
#define RCC_CFGR			(RCC_BASE_ADDR + 0x04)
#define RCC_APB2ENR			(RCC_BASE_ADDR + 0x18)

#define GPIOC_BASE_ADDR		0x40011000
#define GPIOC_CRL			(GPIOC_BASE_ADDR + 0x00)
#define GPIOC_CRH			(GPIOC_BASE_ADDR + 0x04)
#define GPIOC_BSRR			(GPIOC_BASE_ADDR + 0x10)
#define GPIOC_BRR			(GPIOC_BASE_ADDR + 0x14)
#define GPIOC_ODR			(GPIOC_BASE_ADDR + 0x0C)
#define GPIOC_IDR			(GPIOC_BASE_ADDR + 0x08)

#define GPIOA_BASE_ADDR		0x40010800
#define GPIOA_CRL			(GPIOA_BASE_ADDR + 0x00)
#define GPIOA_CRH			(GPIOA_BASE_ADDR + 0x04)
#define GPIOA_BSRR			(GPIOA_BASE_ADDR + 0x10)
#define GPIOA_BRR			(GPIOA_BASE_ADDR + 0x14)
#define GPIOA_ODR			(GPIOA_BASE_ADDR + 0x0C)
#define GPIOA_IDR			(GPIOA_BASE_ADDR + 0x08)

#define AFIO_BASE_ADDR		0x40010000
#define AFIO_MAPR			(AFIO_BASE_ADDR + 0x04)
#define AFIO_EXTICR4		(AFIO_BASE_ADDR + 0x14)

#define EXTI_BASE_ADDR		0x40010400
#define	EXTI_IMR			(EXTI_BASE_ADDR + 0x00)
#define	EXTI_RTSR			(EXTI_BASE_ADDR + 0x08)
#define	EXTI_PR				(EXTI_BASE_ADDR + 0x14)

// Processor side
#define NVIC_ISER0			0xE000E100

#define SYSCLK				4U
#define HSI					5U
#define HSE					6U
#define PLLDividedBy2		7U

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void Delay(uint32_t tick){
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

void ConfigInterruptForPC14Pin(void){
	// Set as raising edge
	*(uint32_t*)EXTI_RTSR |= (0x1U << 14U);

	// Enable the interrupt mask for EXTI14 line
	*(uint32_t*)EXTI_IMR |= (0x1U << 14U);

	// Enable the interrupt in the processor side
	// EXTI14 => IRQ number is 40
	uint32_t *NVIC_ISER = (uint32_t*)NVIC_ISER0 + (40/32);
	*NVIC_ISER |= (1 << (40%32));
}

void ConfigPC14AsEXTIPin(void){
	// Set the EXTI14 line for PC14
	*(uint32_t*)AFIO_EXTICR4 |= (0x2U << 8U);

	ConfigInterruptForPC14Pin();
}

void ConfigHSEClock(void){
	// Turn HSE On
	*(uint32_t*)RCC_CR |= (1U << 16U);

	// While until HSE is ready
	while(!((*(uint32_t*)RCC_CR >> 17U) & 1U));

	// Change the system clock into HSE
	*(uint32_t*)RCC_CFGR |= (1U << 0U);
}

void ConfigPLLClockUsingHSE(void){
	// Turn HSE On
	*(uint32_t*)RCC_CR |= (1U << 16U);
	// While until HSE is ready
	while(!((*(uint32_t*)RCC_CR >> 17U) & 1U));

	// HSE divider for PLL entry (divided by 2)
	*(uint32_t*)RCC_CFGR |= (1U << 17U);
	// PLL entry clock source (HSE oscillator clock selected)
	*(uint32_t*)RCC_CFGR |= (1U << 16U);
	// PLL multiplication factor (x4)
	*(uint32_t*)RCC_CFGR |= (2U << 18U);

	// Turn PLL On
	*(uint32_t*)RCC_CR |= (1U << 24U);
	// While until PLL is ready
	while(!((*(uint32_t*)RCC_CR >> 25U) & 1U));

	// Change the system clock into PLL
	*(uint32_t*)RCC_CFGR |= (2U << 0U);
}

void ConfigPA8AsMCOPin(uint8_t source){
	// Set PA8 as MCO pin output
	*(uint32_t*)GPIOA_CRH &= ~(0xFU << 0U);
	*(uint32_t*)GPIOA_CRH |= (0x1U << 0U);
	*(uint32_t*)GPIOA_CRH |= (0x2U << 2U);

	// Select HSE clock to the MCO pin
	*(uint32_t*)RCC_CFGR |= (source << 24U);
}

int main(void)
{
	// Enable RCC to generate the clock for the GPIOC
	*(uint32_t*)RCC_APB2ENR |= (1U << IOPCEN_BIT);

	/* For SWD Debug */
	EnableSWDDebug();

	// Set Mode for PC13 as LED output pin
	*(uint32_t*)GPIOC_CRH &= ~(0xFU << 20U);
	*(uint32_t*)GPIOC_CRH |= (0x2U << 20U);

	// Set mode for PC14 as Button input pull downpin
	// SetPC14AsInputPullDown();

	// Set interrupt in PC14 line
	ConfigPC14AsEXTIPin();

	// Config system clock as PLL
	ConfigPLLClockUsingHSE();

	// Set HSE clock to MCO Pin
	if(((*(uint32_t*)RCC_CFGR >> 2U) & 0x3U) == 0x2){
		ConfigPA8AsMCOPin(PLLDividedBy2);
	}

    /* Loop forever */
	while(1)
	{
//		PressToTurnLedOff();
		TurnLedOn(13);
	}
}

void EXTI15_10_IRQHandler(void){
	// Turn On LED
	TurnLedOff(13);
	// Delay 1s
	Delay(OneSecond);

	// Disable PR Interrupt register to out the Interrupt
	*(uint32_t*)EXTI_PR |= (1U << 14U);
}
