/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: Aug 26, 2024
 *      Author: manhcuong
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"

/*
 * GPIO posible pins
 */
enum GPIO_PIN_NO
{
	GPIO_PIN_NO_0	 	= 0U,
	GPIO_PIN_NO_1 		= 1U,
	GPIO_PIN_NO_2 		= 2U,
	GPIO_PIN_NO_3 		= 3U,
	GPIO_PIN_NO_4 		= 4U,
	GPIO_PIN_NO_5 		= 5U,
	GPIO_PIN_NO_6 		= 6U,
	GPIO_PIN_NO_7 		= 7U,
	GPIO_PIN_NO_8 		= 8U,
	GPIO_PIN_NO_9 		= 9U,
	GPIO_PIN_NO_10 		= 10U,
	GPIO_PIN_NO_11 		= 11U,
	GPIO_PIN_NO_12 		= 12U,
	GPIO_PIN_NO_13 		= 13U,
	GPIO_PIN_NO_14 		= 14U,
	GPIO_PIN_NO_15		= 15U
};

/*
 * GPIO pin posible modes
 */
enum GPIO_MODE
{
	GPIO_MODE_IN 		= 0U,	/* Set GPIO pin as Input mode */
	GPIO_MODE_OUT 		= 1U,	/* Set GPIO pin as Output mode */
	GPIO_MODE_ALTFN 	= 2U,	/* Set GPIO pin as Alternate function mode */
	GPIO_MODE_ANALOG 	= 3U,	/* Set GPIO pin as Analog mode */
	GPIO_MODE_IT_FT 	= 4U,	/* Set GPIO pin as Interrupt with falling edge mode */
	GPIO_MODE_IT_RT 	= 5U,	/* Set GPIO pin as Interrupt with raising edge mode */
	GPIO_MODE_IT_RFT 	= 6U	/* Set GPIO pin as Interrupt with raising and falling edge mode */
};

/*
 * GPIO pin posible output speeds
 */
enum GPIO_SPEED
{
	GPIO_SPEED_LOW 		= 0U,	/* Set output speed as Low */
	GPIO_SPEED_MEDIUM 	= 1U,	/* Set output speed as Medium */
	GPIO_SPEED_FAST 	= 2U,	/* Set output speed as Fast */
	GPIO_SPEED_HIGH 	= 3U,	/* Set output speed as High */
};

/*
 * GPIO pin Pull up and down configurations
 */
enum GPIO_PUPD
{
	GPIO_NO_PUPD 		= 0U,	/* Set pin no pull-up, pull-down */
	GPIO_PIN_PU 		= 1U,	/* Set pin pull-up */
	GPIO_PIN_PD 		= 2U,	/* Set pin pull-down */
};

/*
 * GPIO pin posible output types
 */
enum GPIO_OP_TYPE
{
	GPIO_OP_TYPE_PP 	= 0U,	/* Set output type as Push-pull */
	GPIO_OP_TYPE_OD 	= 1U,	/* Set output type as Open drain */
};

/*
 * GPIO pin posible output types
 */
enum GPIO_ALT_FN
{
	GPIO_ALT_FN_AF0	 	= 0U,
	GPIO_ALT_FN_AF1 		= 1U,
	GPIO_ALT_FN_AF2 		= 2U,
	GPIO_ALT_FN_AF3 		= 3U,
	GPIO_ALT_FN_AF4 		= 4U,
	GPIO_ALT_FN_AF5 		= 5U,
	GPIO_ALT_FN_AF6 		= 6U,
	GPIO_ALT_FN_AF7 		= 7U,
	GPIO_ALT_FN_AF8 		= 8U,
	GPIO_ALT_FN_AF9 		= 9U,
	GPIO_ALT_FN_AF10 		= 10U,
	GPIO_ALT_FN_AF11 		= 11U,
	GPIO_ALT_FN_AF12 		= 12U,
	GPIO_ALT_FN_AF13 		= 13U,
	GPIO_ALT_FN_AF14 		= 14U,
	GPIO_ALT_FN_AF15		= 15U
};

/*
 * Configuration structure for a GPIO pin
 */
typedef struct
{
	enum GPIO_PIN_NO 	GPIO_PinNumber;
	enum GPIO_MODE 		GPIO_PinMode;
	enum GPIO_SPEED 	GPIO_PinSpeed;
	enum GPIO_PUPD 		GPIO_PinPuPdControl;
	enum GPIO_OP_TYPE 	GPIO_PinOPType;
	enum GPIO_ALT_FN 	GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * Handle structure for a GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t 		*pGPIO; /* Holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t 	GPIO_PinConfig; /* Holds the GPIO pin configuration settings */
}GPIO_Handle_t;

/***************************************************************************************
 * 							APIs supported by GPIO driver
 * 		For more information about the APIs, check the function definitions
 ***************************************************************************************/

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
