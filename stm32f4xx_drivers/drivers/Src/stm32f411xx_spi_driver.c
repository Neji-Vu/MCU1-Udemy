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
	// Peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPI, ENABLE);

	// Temporary register variable
	uint32_t tempReg = 0x00000000U;

	// 1. Configure the device mode
	tempReg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_BIT_MSTR;

	// 2. Configure the bus configuration
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// Configure bus in full duplex communication
		// BIDIMODE bit needs to be set to 0
		tempReg &= ~(1 << SPI_CR1_BIT_BIDIMODE);
		// RXONLY bit needs to be set to 0
		tempReg &= ~(1 << SPI_CR1_BIT_RXONLY);

	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// Configure bus in half duplex communication
		// BIDIMODE bit needs to be set to 1
		tempReg |= 1 << SPI_CR1_BIT_BIDIMODE;
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLE_RXONLY){
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
	tempReg |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BIT_BR);

	// 4. Configure the data frame format
	tempReg |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_BIT_DFF);

	// 5. Configure the clock polarity
	tempReg |= (pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_BIT_CPOL);

	// 6. Configure the clock phase
	tempReg |= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_BIT_CPHA);

	// 7. Configure the software slave management
	tempReg |= (pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_BIT_SSM);


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
 * @fn      		  - SPI_PeripheralControl
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
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi){
		// Enable SPI
		pSPIx->CR1 |= (1 << SPI_CR1_BIT_SPE);
	}
	else{
		// Disable SPI
		pSPIx->CR1 &= ~(1 << SPI_CR1_BIT_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
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
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi){
		// Enable SSI
		pSPIx->CR1 |= (1 << SPI_CR1_BIT_SSI);
	}
	else{
		// Disable SSI
		pSPIx->CR1 &= ~(1 << SPI_CR1_BIT_SSI);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
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
void  SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_BIT_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_BIT_SSOE);
	}
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
		if((pSPIx->CR1 & (1 << SPI_CR1_BIT_DFF)) == SPI_DFF_8BITS){
			// Load 8-bit data to the data register and increment the buffer address
			pSPIx->DR = *pTxBuffer;
			// Increment the buffer address
			pTxBuffer++;

			// Decrement the length
			Len--;
		}
		else{
			// todo: need to check for odd bytes => fault
			// Load 16-bit data to the data register
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			// Increment the buffer address
			pTxBuffer += 2U;

			// Decrement the length two times (2 bytes are sent)
			Len -= 2U;
		}
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		// 1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		// 2. Mark the SPI state as busy in transmission so that
		//    No other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPI->CR2 |= (1 << SPI_CR2_BIT_TXEIE);
	}

	return state;
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	// Check the length of data is greater than 0U or not
	while(Len > 0U){
		// Wait until the RX buffer is not empty
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == SPI_RX_BUFFER_EMPTY);

		// Check the data frame format is 8-bit of 16-bit
		if((pSPIx->CR1 & (1 << SPI_CR1_BIT_DFF)) == SPI_DFF_8BITS){
			// Read 1 byte data from DR to RxBuffer
			*pRxBuffer = pSPIx->DR;
			// Increment Rx buffer
			pRxBuffer++;
			// Decrease the length
			Len--;
		}
		else{
			// Read 2 bytes data from DR to RxBuffer
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			// Increment Rx buffer
			pRxBuffer += 2U;
			// Decrease the length
			Len -= 2U;
		}
	}
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		// 1. Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		// 2. Mark the SPI state as busy in reception so that
		//    No other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPI->CR2 |= (1 << SPI_CR2_BIT_RXNEIE);
	}

	return state;

}

/*********************************************************************
 * @fn      		  - SPI_IRQConfig
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
void SPI_IRQInterruptConfig(enum IRQ_NO IRQNumber, uint8_t EnorDi)
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
			// Not applicable
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
			// Not applicable
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
void SPI_IRQPriorityConfig(enum IRQ_NO IRQNumber, uint8_t IRQPriority)
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
