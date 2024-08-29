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
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			// 1. Configure FTSR
			// Set FTSR of EXTI
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Reset RTSR of EXTI
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			// 1. Configure RTSR
			// Reset FTSR of EXTI
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Set RTSR of EXTI
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			// 1. Configure both FTSR and RTSR
			// Set FTSR of EXTI
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Set RTSR of EXTI
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else{
			// Do nothing
		}
		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t reg_pos = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t reg_section = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIO);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[reg_pos] |= (portcode << (reg_section * 4));

		// 3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
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
		uint8_t reg_section = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIO->AFR[reg_pos] &= ~(0xF << (reg_section * 4));
		pGPIOHandle->pGPIO->AFR[reg_pos] = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (reg_section * 4));
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

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -   0 or 1
 *
 * @Note              -

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	return (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
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
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)pGPIOx->IDR;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
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
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
		pGPIOx->ODR |= (1 << PinNumber);
	else
		pGPIOx->ODR &= ~(1 << PinNumber);
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
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
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR  = Value;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
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
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR  ^= ( 1 << PinNumber);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
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
void GPIO_IRQInterruptConfig(enum IRQ_NO IRQNumber, uint8_t EnorDi)
{
	if(EnorDi){
		if(IRQNumber >= 0U && IRQNumber < 32U){
			// 0-31 IRQs are set in ISR0 registers
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber >= 32U && IRQNumber < 64U){
			// 32-63 IRQs are set in ISR1 registers
			*NVIC_ISER1 |= (1 << (IRQNumber % 32U));
		}
		else if(IRQNumber >= 64U && IRQNumber < 96U){
			// 64-95 IRQs are set in ISR2 registers
			*NVIC_ISER2 |= (1 << (IRQNumber % 64U));
		}
		else{
			// No applicable
		}
	}
	else{
		if(IRQNumber >= 0U && IRQNumber < 32U){
			// 0-31 IRQs are cleared in ISR0 registers
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber >= 32U && IRQNumber < 64U){
			// 32-63 IRQs are cleared in ISR1 registers
			*NVIC_ICER1 |= (1 << (IRQNumber % 32U));
		}
		else if(IRQNumber >= 64U && IRQNumber < 96U){
			// 64-95 IRQs are cleared in ISR2 registers
			*NVIC_ICER2 |= (1 << (IRQNumber % 64U));
		}
		else{
			// No applicable
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
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
void GPIO_IRQPriorityConfig(enum IRQ_NO IRQNumber, uint8_t IRQPriority)
{
	// Identify the register position needed to configure
	uint8_t reg_pos = IRQNumber / 4;
	// Identify the register section of register needed to configure
	uint8_t reg_section = IRQNumber % 4;

	// Shift No implemented bits in Interrupt Priortity
	uint8_t shift_amount = (reg_section * 8) + (8 - NO_PR_BITS_IMPLEMENTED);

	// Configure the priority to IPR
	*(NVIC_IPR_BASEADDR + reg_pos) |= (IRQPriority << shift_amount);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
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
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		// Clear bit by setting it to 1
		EXTI->PR |= ( 1 << PinNumber);
	}

}
