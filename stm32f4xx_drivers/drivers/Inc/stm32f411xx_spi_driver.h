/*
 * stm32f411xx_spi_driver.h
 *
 *  Created on: Aug 29, 2024
 *      Author: ManhCuong
 */

#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_

#include "stm32f411xx.h"

/*
 * Device mode selection
 */
enum SPI_DEVICE_MODE
{
	SPI_DEVICE_MODE_MASTER		= 0U,	/* MCU is the master */
	SPI_DEVICE_MODE_SLAVE		= 1U	/* MCU is the slave */
};

/*
 * Select full duplex, half duplex or simple communications
 */
enum SPI_BUS_CONFIG
{
	SPI_BUS_CONFIG_FD				= 1U,	/* Full-duplex communication */
	SPI_BUS_CONFIG_HD				= 2U,	/* Half-duplex communication */
	SPI_BUS_CONFIG_SIMPLE_RXONLY	= 3U	/* Simple communication */
};

/*
 * Select Baud rate control (Configure the serial clock in SCLK pin)
 * The serial clock is equal to (SPI_clock / Prescaler)
 * SPI_clock depends on the clock in bus interface hung on
 * Configure the Prescaler number
 */
enum SPI_SCLK_SPEED
{
	SPI_SCLK_SPEED_DIV2				= 0U,
	SPI_SCLK_SPEED_DIV4				= 1U,
	SPI_SCLK_SPEED_DIV8				= 2U,
	SPI_SCLK_SPEED_DIV16			= 3U,
	SPI_SCLK_SPEED_DIV32			= 4U,
	SPI_SCLK_SPEED_DIV64			= 5U,
	SPI_SCLK_SPEED_DIV128			= 6U,
	SPI_SCLK_SPEED_DIV256			= 7U
};

/*
 * Configure the data frame format
 */
enum SPI_DFF
{
	SPI_DFF_8BITS		= 0U,	/* 8-bit data frame format is selected for transmission/reception */
	SPI_DFF_16BITS		= 1U	/* 16-bit data frame format is selected for transmission/reception */
};

/*
 * Configure the clock polarity
 */
enum SPI_CPOL
{
	SPI_CPOL_LOW		= 0U,
	SPI_CPOL_HIGH		= 1U
};

/*
 * Configure the clock phase
 */
enum SPI_CPHA
{
	SPI_CPHA_LOW		= 0U,	/* The first clock transition is the first data capture edge */
	SPI_CPHA_HIGH		= 1U	/* The second clock transition is the first data capture edge */
};

/*
 * Software slave management
 */
enum SPI_SSM
{
	SPI_SSM_DI			= 0U,	/* Software slave management disabled */
	SPI_SSM_EN			= 1U	/* Software slave management enabled */
};
/*
 *  Configuration structure for SPIx peripheral
 */
typedef struct
{
	enum SPI_DEVICE_MODE 	SPI_DeviceMode;
	enum SPI_BUS_CONFIG 	SPI_BusConfig;
	enum SPI_SCLK_SPEED 	SPI_SclkSpeed;
	enum SPI_DFF 			SPI_DFF;
	enum SPI_CPOL 			SPI_CPOL;
	enum SPI_CPHA 			SPI_CPHA;
	enum SPI_SSM 			SPI_SSM;
}SPI_Config_t;

/*
 * Handle structure for a SPIx peripherals
 */
typedef struct
{
	SPI_RegDef_t 		*pSPI; /* Holds the base address of the SPI communication to which the pin belongs */
	SPI_Config_t 	SPI_PinConfig; /* Holds the SPI communication's configuration settings */
}SPI_Handle_t;

/*
 * Generic macros
 */
#define SPI_TX_BUFFER_NOT_EMPTY		0U
#define SPI_TX_BUFFER_EMPTY			1U

/*
 * SPI related status flag definitions
 */
#define SPI_TXE_FLAG				(1 << SPI_SR_BIT_TXE)

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
