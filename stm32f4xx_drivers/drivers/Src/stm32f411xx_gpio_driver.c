/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: Aug 26, 2024
 *      Author: manhcuong
 */

#include "stm32f411xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi){
		switch ((uint32_t)pGPIOx) {
			case GPIOA_BASEADDR:
				GPIOA_PCLK_EN();
				break;
			case GPIOB_BASEADDR:
				GPIOB_PCLK_EN();
				break;
			case GPIOC_BASEADDR:
				GPIOC_PCLK_EN();
				break;
			case GPIOD_BASEADDR:
				GPIOD_PCLK_EN();
				break;
			case GPIOE_BASEADDR:
				GPIOE_PCLK_EN();
				break;
			case GPIOH_BASEADDR:
				GPIOH_PCLK_EN();
				break;
			default:
				break;
		}
	}
	else{
		switch ((uint32_t)pGPIOx) {
			case GPIOA_BASEADDR:
				GPIOA_PCLK_DI();
				break;
			case GPIOB_BASEADDR:
				GPIOB_PCLK_DI();
				break;
			case GPIOC_BASEADDR:
				GPIOC_PCLK_DI();
				break;
			case GPIOD_BASEADDR:
				GPIOD_PCLK_DI();
				break;
			case GPIOE_BASEADDR:
				GPIOE_PCLK_DI();
				break;
			case GPIOH_BASEADDR:
				GPIOH_PCLK_DI();
				break;
			default:
				break;
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	// 1. Configure the mode of gpio pin
	// First check it is interrupt mode or not
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		// Not interrupt mode
		// Reset bit at the pin position
		pGPIOHandle->pGPIO->MODER &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
		// Set the mode for the pin
		pGPIOHandle->pGPIO->MODER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
	}
	else{
		// Interrupt mode

	}

	// 2. Configure the speed
	// Reset bit at the pin position
	pGPIOHandle->pGPIO->OSPEEDR &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
	// Set the mode for the pin
	pGPIOHandle->pGPIO->OSPEEDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));

	// 3. Configure the pupd settings
	// Reset bit at the pin position
	pGPIOHandle->pGPIO->PUPDR &= ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
	// Set the mode for the pin
	pGPIOHandle->pGPIO->PUPDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));

	// 4. Configure the optype
	// Reset bit at the pin position
	pGPIOHandle->pGPIO->OTYPER &= ~(1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	// Set the mode for the pin
	pGPIOHandle->pGPIO->OTYPER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	// 5. Configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		// Change the functionality of the pin
		uint8_t reg_pos = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		uint8_t bit_pos = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIO->AFR[reg_pos] &= ~(0xF << (bit_pos * 4));
		pGPIOHandle->pGPIO->AFR[reg_pos] = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (bit_pos * 4));
	}

}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	switch ((uint32_t)pGPIOx){
		case GPIOA_BASEADDR:
			GPIOA_REG_RESET();
			break;
		case GPIOB_BASEADDR:
			GPIOB_REG_RESET();
			break;
		case GPIOC_BASEADDR:
			GPIOC_REG_RESET();
			break;
		case GPIOD_BASEADDR:
			GPIOD_REG_RESET();
			break;
		case GPIOE_BASEADDR:
			GPIOE_REG_RESET();
			break;
		case GPIOH_BASEADDR:
			GPIOH_REG_RESET();
			break;
		default:
			break;
	}
}
