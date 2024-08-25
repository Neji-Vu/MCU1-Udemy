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
#define SYSCFG_BASEADDR 		(APB2PERIPH_BASEADDR + 0x3800U)
#define EXTI_BASEADDR 			(APB2PERIPH_BASEADDR + 0x3C00U)

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
	__vo uint32_t Reserved1;	/* Offset: 0x18 */
	__vo uint32_t Reserved2;	/* Offset: 0x1C */
	__vo uint32_t APB1RSTR;		/* Offset: 0x20, RCC APB1 peripheral reset register */
	__vo uint32_t APB2RSTR;		/* Offset: 0x24, RCC APB2 peripheral reset register */
	__vo uint32_t Reserved3;	/* Offset: 0x28 */
	__vo uint32_t Reserved4;	/* Offset: 0x2C */
	__vo uint32_t AHB1ENR;		/* Offset: 0x30, RCC AHB1 peripheral clock enable register */
	__vo uint32_t AHB2ENR;		/* Offset: 0x34, RCC AHB2 peripheral clock enable register */
	__vo uint32_t Reserved5;	/* Offset: 0x38 */
	__vo uint32_t Reserved6;	/* Offset: 0x3C */
	__vo uint32_t APB1ENR;		/* Offset: 0x40, RCC APB1 peripheral clock enable register */
	__vo uint32_t APB2ENR;		/* Offset: 0x44, RCC APB2 peripheral clock enable register */
	__vo uint32_t Reserved7;	/* Offset: 0x48 */
	__vo uint32_t Reserved8;	/* Offset: 0x4C */
	__vo uint32_t AHB1LPENR;	/* Offset: 0x50, RCC AHB1 peripheral clock enable in low power mode register */
	__vo uint32_t AHB2LPENR;	/* Offset: 0x54, RCC AHB2 peripheral clock enable in low power mode register */
	__vo uint32_t Reserved9;	/* Offset: 0x58 */
	__vo uint32_t Reserved10;	/* Offset: 0x5C */
	__vo uint32_t APB1LPENR;	/* Offset: 0x60, RCC APB1 peripheral clock enabled in low power mode register */
	__vo uint32_t APB2LPENR;	/* Offset: 0x64, RCC APB2 peripheral clock enabled in low power mode register */
	__vo uint32_t Reserved11;	/* Offset: 0x68 */
	__vo uint32_t Reserved12;	/* Offset: 0x6C */
	__vo uint32_t BDCR;			/* Offset: 0x70, RCC Backup domain control register */
	__vo uint32_t CSR;			/* Offset: 0x74, RCC clock control & status register */
	__vo uint32_t Reserved13;	/* Offset: 0x78 */
	__vo uint32_t Reserved14;	/* Offset: 0x7C */
	__vo uint32_t SSCGR;		/* Offset: 0x80, RCC spread spectrum clock generation register */
	__vo uint32_t PLLI2SCFGR;	/* Offset: 0x84, RCC PLLI2S configuration register */
	__vo uint32_t Reserved15;	/* Offset: 0x88 */
	__vo uint32_t DCKCFGR;		/* Offset: 0x8C, RCC Dedicated Clocks Configuration Register*/
}RCC_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t
 */
#define GPIOA 					( (GPIO_RegDef_t*) GPIOA_BASEADDR )
#define RCC						( (RCC_RegDef_t*) RCC_BASEADDR )

#endif /* INC_STM32F411XX_H_ */
