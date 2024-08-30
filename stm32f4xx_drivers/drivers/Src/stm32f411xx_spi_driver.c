/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: Aug 29, 2024
 *      Author: ManhCuong
 */

#include "stm32f411xx_spi_driver.h"

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
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
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi){
		switch ((uint32_t)pSPIx) {
			case SPI1_BASEADDR:
				SPI1_PCLK_EN();
				break;
			case SPI2_BASEADDR:
				SPI2_PCLK_EN();
				break;
			case SPI3_BASEADDR:
				SPI3_PCLK_EN();
				break;
			case SPI4_BASEADDR:
				SPI4_PCLK_EN();
				break;
			case SPI5_BASEADDR:
				SPI5_PCLK_EN();
				break;
			default:
				break;
		}
	}
	else{
		switch ((uint32_t)pSPIx) {
			case SPI1_BASEADDR:
				SPI1_PCLK_DI();
				break;
			case SPI2_BASEADDR:
				SPI2_PCLK_DI();
				break;
			case SPI3_BASEADDR:
				SPI3_PCLK_DI();
				break;
			case SPI4_BASEADDR:
				SPI4_PCLK_DI();
				break;
			case SPI5_BASEADDR:
				SPI5_PCLK_DI();
				break;
			default:
				break;
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - Configure the SPI_CR1 register
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Temporary register variable
	uint32_t tempReg = 0x00000000U;

	// 1. Configure the device mode
	tempReg |= pSPIHandle->SPI_PinConfig.SPI_DeviceMode << SPI_CR1_BIT_MSTR;

	// 2. Configure the bus configuration
	if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// Configure bus in full duplex communication
		// BIDIMODE bit needs to be set to 0
		tempReg &= ~(1 << SPI_CR1_BIT_BIDIMODE);
		// RXONLY bit needs to be set to 0
		tempReg &= ~(1 << SPI_CR1_BIT_RXONLY);

	}
	else if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// Configure bus in half duplex communication
		// BIDIMODE bit needs to be set to 1
		tempReg |= 1 << SPI_CR1_BIT_BIDIMODE;
	}
	else if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLE_RXONLY){
		// Configure bus in simple communication
		// BIDIMODE bit needs to be set to 0
		tempReg &= ~(1 << SPI_CR1_BIT_BIDIMODE);
		// RXONLY bit needs to be set to 1
		tempReg |= 1 << SPI_CR1_BIT_RXONLY;
	}
	else{
		// Do nothing
	}

	// 3. Configure the clock speed (Prescaler number)
	tempReg |= (pSPIHandle->SPI_PinConfig.SPI_SclkSpeed << SPI_CR1_BIT_BR);

	// 4. Configure the data frame format
	tempReg |= (pSPIHandle->SPI_PinConfig.SPI_DFF << SPI_CR1_BIT_DFF);

	// 5. Configure the clock polarity
	tempReg |= (pSPIHandle->SPI_PinConfig.SPI_CPOL << SPI_CR1_BIT_CPOL);

	// 6. Configure the clock phase
	tempReg |= (pSPIHandle->SPI_PinConfig.SPI_CPHA << SPI_CR1_BIT_CPHA);

	// 7. Configure the software slave management
	tempReg |= (pSPIHandle->SPI_PinConfig.SPI_SSM << SPI_CR1_BIT_SSM);


	// Set temporary value to SPI_CR1 register
	pSPIHandle->pSPI->CR1 |= tempReg;

}

/*********************************************************************
 * @fn      		  - SPI_DeInit
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
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
 //todo
}

/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
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
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - This is blocking call since the application is actually blocked
 * 						by this function (2 while loops) until it is finished.
 * 						And the second while loop can have the potential to block the application
 * 						permanently sometimes.

 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	// Check the length of the data is 0U or not
	while(Len > 0U){
		// Wait until Tx buffer is empty
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == SPI_TX_BUFFER_NOT_EMPTY);

		// Check the data frame format of SPI is 8-bit or 16-bit
		// 0: data frame format is 8 bits
		// 1: data frame format is 16 bits
		if((pSPIx->CR1 & (1 << SPI_CR1_BIT_DFF)) == 0U){
			// Load 8-bit data to the data register and increment the buffer address
			pSPIx->DR = *pTxBuffer;
			// Increment the buffer address
			pTxBuffer++;

			// Decrement the length
			Len--;
		}
		else{
			// Load 16-bit data to the data register
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			// Increment the buffer address
			(uint16_t*)pTxBuffer++;

			// Decrement the length two times (2 bytes are sent)
			Len -= 2U;
		}
	}
}
