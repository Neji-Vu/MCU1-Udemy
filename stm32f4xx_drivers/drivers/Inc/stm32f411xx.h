/*
 * stm32f411xx.h
 *
 *  Created on: Aug 25, 2024
 *      Author: manhcuong
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>

#define __vo volatile

/*
 * Base addresses of FLASH and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U
#define SRAM_BASEADDR			0x20000000U
#define ROM_BASEADDR			0x1FFF0000U
#define SRAM					SRAM_BASEADDR

/*
 * Base addresses of AHBx and APBx Bus peripheral
 */

#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		0x40000000U
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1000U)
#define GPIOH_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1C00U)
#define RCC_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x3800U)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define SPI2_BASEADDR 			(APB1PERIPH_BASEADDR + 0x3800U)
#define SPI3_BASEADDR 			(APB1PERIPH_BASEADDR + 0x3C00U)
#define USART2_BASEADDR 		(APB1PERIPH_BASEADDR + 0x4400U)
#define I2C1_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5400U)
#define I2C2_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5800U)
#define I2C3_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5C00U)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define USART1_BASEADDR 		(APB2PERIPH_BASEADDR + 0x1000U)
#define USART6_BASEADDR 		(APB2PERIPH_BASEADDR + 0x1400U)
#define SPI1_BASEADDR 			(APB2PERIPH_BASEADDR + 0x3000U)
#define SPI4_BASEADDR 			(APB2PERIPH_BASEADDR + 0x3400U)
#define SPI5_BASEADDR 			(APB2PERIPH_BASEADDR + 0x5000U)
#define SYSCFG_BASEADDR 		(APB2PERIPH_BASEADDR + 0x3800U)
#define EXTI_BASEADDR 			(APB2PERIPH_BASEADDR + 0x3C00U)

/*
 * Base addresses of Interrupt Set-enable Registers of Arm Cortex M4 internal registers
 */

#define NVIC_ISER0				((__vo uint32_t*)0xE000E100U)
#define NVIC_ISER1				((__vo uint32_t*)0xE000E104U)
#define NVIC_ISER2				((__vo uint32_t*)0xE000E108U)
#define NVIC_ISER3				((__vo uint32_t*)0xE000E10CU)
#define NVIC_ISER4				((__vo uint32_t*)0xE000E110U)
#define NVIC_ISER5				((__vo uint32_t*)0xE000E114U)
#define NVIC_ISER6				((__vo uint32_t*)0xE000E118U)
#define NVIC_ISER7				((__vo uint32_t*)0xE000E11CU)

/*
 * Base addresses of Interrupt Clear-enable Registers of Arm Cortex M4 internal registers
 */

#define NVIC_ICER0				((__vo uint32_t*)0xE000E180U)
#define NVIC_ICER1				((__vo uint32_t*)0xE000E184U)
#define NVIC_ICER2				((__vo uint32_t*)0xE000E188U)
#define NVIC_ICER3				((__vo uint32_t*)0xE000E18CU)
#define NVIC_ICER4				((__vo uint32_t*)0xE000E190U)
#define NVIC_ICER5				((__vo uint32_t*)0xE000E194U)
#define NVIC_ICER6				((__vo uint32_t*)0xE000E198U)
#define NVIC_ICER7				((__vo uint32_t*)0xE000E19CU)

/*
 * Base addresses of Interrupt Set-pending Registers of Arm Cortex M4 internal registers
 */

#define NVIC_ISPR0				((__vo uint32_t*)0xE000E200U)
#define NVIC_ISPR1				((__vo uint32_t*)0xE000E204U)
#define NVIC_ISPR2				((__vo uint32_t*)0xE000E208U)
#define NVIC_ISPR3				((__vo uint32_t*)0xE000E20CU)
#define NVIC_ISPR4				((__vo uint32_t*)0xE000E210U)
#define NVIC_ISPR5				((__vo uint32_t*)0xE000E214U)
#define NVIC_ISPR6				((__vo uint32_t*)0xE000E218U)
#define NVIC_ISPR7				((__vo uint32_t*)0xE000E21CU)

/*
 * Base addresses of Interrupt Priority Registers of Arm Cortex M4 internal registers
 */

#define NVIC_IPR_BASEADDR		((__vo uint32_t*)0xE000E400U)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/*
 * Peripheral register definition structures
 */

typedef struct
{
	__vo uint32_t MODER;		/* Offset: 0x00, GPIO port mode register */
	__vo uint32_t OTYPER;		/* Offset: 0x04, GPIO port output type register */
	__vo uint32_t OSPEEDR;		/* Offset: 0x08, GPIO port output speed register */
	__vo uint32_t PUPDR;		/* Offset: 0x0C, GPIO port pull-up/pull-down register */
	__vo uint32_t IDR;			/* Offset: 0x10, GPIO port input data register */
	__vo uint32_t ODR;			/* Offset: 0x14, GPIO port output data register */
	__vo uint32_t BSRR;			/* Offset: 0x18, GPIO port bit set/reset register */
	__vo uint32_t LCKR;			/* Offset: 0x1C, GPIO port configuration lock register */
	__vo uint32_t AFR[2];		/* Offset: 0x20, GPIO alternate function low register */
								/* Offset: 0x24, GPIO alternate function high register */

}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;			/* Offset: 0x00, RCC clock control register */
	__vo uint32_t PLLCFGR;		/* Offset: 0x04, RCC PLL configuration register */
	__vo uint32_t CFGR;			/* Offset: 0x08, RCC clock configuration register */
	__vo uint32_t CIR;			/* Offset: 0x0C, RCC clock interrupt register */
	__vo uint32_t AHB1RSTR;		/* Offset: 0x10, RCC AHB1 peripheral reset register */
	__vo uint32_t AHB2RSTR;		/* Offset: 0x14, RCC AHB2 peripheral reset register */
	uint32_t 	  RESERVED1;	/* Offset: 0x18 */
	uint32_t 	  RESERVED2;	/* Offset: 0x1C */
	__vo uint32_t APB1RSTR;		/* Offset: 0x20, RCC APB1 peripheral reset register */
	__vo uint32_t APB2RSTR;		/* Offset: 0x24, RCC APB2 peripheral reset register */
	uint32_t 	  RESERVED3;	/* Offset: 0x28 */
	uint32_t 	  RESERVED4;	/* Offset: 0x2C */
	__vo uint32_t AHB1ENR;		/* Offset: 0x30, RCC AHB1 peripheral clock enable register */
	__vo uint32_t AHB2ENR;		/* Offset: 0x34, RCC AHB2 peripheral clock enable register */
	uint32_t 	  RESERVED5;	/* Offset: 0x38 */
	uint32_t 	  RESERVED6;	/* Offset: 0x3C */
	__vo uint32_t APB1ENR;		/* Offset: 0x40, RCC APB1 peripheral clock enable register */
	__vo uint32_t APB2ENR;		/* Offset: 0x44, RCC APB2 peripheral clock enable register */
	uint32_t 	  RESERVED7;	/* Offset: 0x48 */
	uint32_t 	  RESERVED8;	/* Offset: 0x4C */
	__vo uint32_t AHB1LPENR;	/* Offset: 0x50, RCC AHB1 peripheral clock enable in low power mode register */
	__vo uint32_t AHB2LPENR;	/* Offset: 0x54, RCC AHB2 peripheral clock enable in low power mode register */
	uint32_t 	  RESERVED9;	/* Offset: 0x58 */
	uint32_t 	  RESERVED10;	/* Offset: 0x5C */
	__vo uint32_t APB1LPENR;	/* Offset: 0x60, RCC APB1 peripheral clock enabled in low power mode register */
	__vo uint32_t APB2LPENR;	/* Offset: 0x64, RCC APB2 peripheral clock enabled in low power mode register */
	uint32_t 	  RESERVED11;	/* Offset: 0x68 */
	uint32_t 	  RESERVED12;	/* Offset: 0x6C */
	__vo uint32_t BDCR;			/* Offset: 0x70, RCC Backup domain control register */
	__vo uint32_t CSR;			/* Offset: 0x74, RCC clock control & status register */
	uint32_t 	  RESERVED13;	/* Offset: 0x78 */
	uint32_t 	  RESERVED14;	/* Offset: 0x7C */
	__vo uint32_t SSCGR;		/* Offset: 0x80, RCC spread spectrum clock generation register */
	__vo uint32_t PLLI2SCFGR;	/* Offset: 0x84, RCC PLLI2S configuration register */
	uint32_t 	  RESERVED15;	/* Offset: 0x88 */
	__vo uint32_t DCKCFGR;		/* Offset: 0x8C, RCC Dedicated Clocks Configuration Register*/
}RCC_RegDef_t;

typedef struct
{
	__vo uint32_t IMR;			/* Offset: 0x00, Interrupt mask register */
	__vo uint32_t EMR;			/* Offset: 0x04, Event mask register */
	__vo uint32_t RTSR;			/* Offset: 0x08, Rising trigger selection register */
	__vo uint32_t FTSR;     	/* Offset: 0x0C, Falling trigger selection register */
	__vo uint32_t SWIER;		/* Offset: 0x10, Software interrupt event register */
	__vo uint32_t PR;			/* Offset: 0x14, Pending register */
}EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t MEMRMP;		/* Offset: 0x00, SYSCFG memory remap register */
	__vo uint32_t PMC;			/* Offset: 0x04, SYSCFG peripheral mode configuration register */
	__vo uint32_t EXTICR[4];	/* Offset: 0x08, SYSCFG external interrupt configuration register 1 */
								/* Offset: 0x0C, SYSCFG external interrupt configuration register 2 */
								/* Offset: 0x10, SYSCFG external interrupt configuration register 3 */
								/* Offset: 0x18, SYSCFG external interrupt configuration register 4 */
	__vo uint32_t CMPCR;		/* Offset: 0x1C, Compensation cell control register */
}SYSCFG_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;			/* Offset: 0x00, SPI control register 1 */
	__vo uint32_t CR2;			/* Offset: 0x04, SPI control register 2 */
	__vo uint32_t SR;			/* Offset: 0x08, SPI status register */
	__vo uint32_t DR;			/* Offset: 0x0C, SPI data register */
	__vo uint32_t CRCPR;		/* Offset: 0x10, SPI CRC polynomial register */
	__vo uint32_t RXCRCR;		/* Offset: 0x14, SPI RX CRC register */
	__vo uint32_t TXCRCR;		/* Offset: 0x18, SPI TX CRC register */
	__vo uint32_t I2SCFGR;		/* Offset: 0x1C, SPI_I2S configuration register */
	__vo uint32_t I2SPR;		/* Offset: 0x20, SPI_I2S prescaler register */
}SPI_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t
 */

#define GPIOA 					((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB 					((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC 					((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD 					((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE 					((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOH 					((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC						((RCC_RegDef_t*) RCC_BASEADDR)
#define EXTI					((EXTI_RegDef_t*) EXTI_BASEADDR)
#define SYSCFG					((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)
#define SPI1					((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4					((SPI_RegDef_t*) SPI4_BASEADDR)
#define SPI5					((SPI_RegDef_t*) SPI5_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 7))

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN() 			(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() 			(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() 			(RCC->APB1ENR |= (1 << 23))

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI2_PCLK_EN() 			(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() 			(RCC->APB1ENR |= (1 << 15))
#define SPI1_PCLK_EN() 			(RCC->APB2ENR |= (1 << 12))
#define SPI4_PCLK_EN() 			(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN() 			(RCC->APB2ENR |= (1 << 20))

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART2_PCLK_EN() 		(RCC->APB1ENR |= (1 << 17))
#define USART1_PCLK_EN() 		(RCC->APB2ENR |= (1 << 4))
#define USART6_PCLK_EN() 		(RCC->APB2ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN() 		(RCC->APB2ENR |= (1 << 14))

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI() 		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() 		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() 		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() 		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() 		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI() 		(RCC->AHB1ENR &= ~(1 << 7))

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Diable Macros for SPIx peripherals
 */

#define SPI2_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 15))
#define SPI1_PCLK_DI() 			(RCC->APB2ENR &= ~(1 << 12))
#define SPI4_PCLK_DI() 			(RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI() 			(RCC->APB2ENR &= ~(1 << 20))

/*
 * Clock Diable Macros for USARTx peripherals
 */

#define USART2_PCLK_DI() 		(RCC->APB1ENR &= ~(1 << 17))
#define USART1_PCLK_DI() 		(RCC->APB2ENR &= ~(1 << 4))
#define USART6_PCLK_DI() 		(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock Disable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI() 		(RCC->APB2ENR &= ~(1 << 14))

/*
 * Macros to reset GPIOx registers
 */
#define GPIOA_REG_RESET()		(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0))
#define GPIOB_REG_RESET()		(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1))
#define GPIOC_REG_RESET()		(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2))
#define GPIOD_REG_RESET()		(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3))
#define GPIOE_REG_RESET()		(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4))
#define GPIOH_REG_RESET()		(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7))

/*
 * Macros to identify the EXTIx external interrupt value
 */
#define GPIO_BASEADDR_TO_CODE(PGPIOx)	(PGPIOx == GPIOA) ? 0x0 : \
										(PGPIOx == GPIOB) ? 0x1 : \
										(PGPIOx == GPIOC) ? 0x2 : \
										(PGPIOx == GPIOD) ? 0x3 : \
										(PGPIOx == GPIOE) ? 0x4 : \
										(PGPIOx == GPIOH) ? 0x7 : 0x0

/*
 * Some generic macros
 */
#define ENABLE 					1U
#define DISABLE 				0U
#define SET 					ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_SET				SET
#define FLAG_RESET				RESET

enum IRQ_NO{
	IRQ_NO_EXTI0				= 6U,
	IRQ_NO_EXTI1				= 7U,
	IRQ_NO_EXTI2				= 8U,
	IRQ_NO_EXTI3				= 9U,
	IRQ_NO_EXTI4				= 10U,
	IRQ_NO_EXTI5_9				= 23U,
	IRQ_NO_SPI1					= 35U,
	IRQ_NO_SPI2					= 36U,
	IRQ_NO_EXTI10_15			= 40U,
	IRQ_NO_SPI3					= 51U,
	IRQ_NO_SPI4					= 84U,
	IRQ_NO_SPI5					= 85U
};

/*
 * Bit fields of SPI CR1 register
 */
#define SPI_CR1_BIT_CPHA		0U
#define SPI_CR1_BIT_CPOL		1U
#define SPI_CR1_BIT_MSTR		2U
#define SPI_CR1_BIT_BR			3U
#define SPI_CR1_BIT_SPE			6U
#define SPI_CR1_BIT_SSI			8U
#define SPI_CR1_BIT_SSM			9U
#define SPI_CR1_BIT_RXONLY		10U
#define SPI_CR1_BIT_DFF			11U
#define SPI_CR1_BIT_BIDIMODE	15U

/*
 * Bit fields of SPI CR2 register
 */
#define SPI_CR2_BIT_SSOE		2U
#define SPI_CR2_BIT_RXNEIE		6U
#define SPI_CR2_BIT_TXEIE		7U

/*
 * Bit fields of SPI SR register
 */
#define SPI_SR_BIT_RXNE			0U
#define SPI_SR_BIT_TXE			1U
#define SPI_SR_BIT_BSY			7U

#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_spi_driver.h"

#endif /* INC_STM32F411XX_H_ */
