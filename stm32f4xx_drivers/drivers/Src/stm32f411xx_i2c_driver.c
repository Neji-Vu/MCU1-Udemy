/*
 * stm32f411xx_i2c_driver.c
 *
 *  Created on: Sep 4, 2024
 *      Author: manhcuong
 */

#include "stm32f411xx_i2c_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = { 2, 4 , 8, 16};

uint32_t  RCC_GetPLLOutputClock()
{

	return 0;
}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t SystemClk = 0U;

	// Calc the System clock of the MCU STM32F4
	uint8_t ClkSrc = 0U;
	ClkSrc = ((RCC->CFGR >> 2) & 0x3U);

	// HSI oscillator used as the system clock
	if(ClkSrc == 0U){
		SystemClk = 160000000U; // 16Mhz
	}
	// HSE oscillator used as the system clock
	else if (ClkSrc == 1U){
		SystemClk = 80000000U; // 8Mhz
	}
	// PLL used as the system clock
	else if (ClkSrc == 2U){
		SystemClk = RCC_GetPLLOutputClock(); // Depends on PLL settings
	}
	else{
		// Not applicable
	}

	// Calc the AHB Prescaler
	uint8_t HPRE_val = 0U, AHB_pre = 0U;
	HPRE_val = (RCC->CFGR >> 4) & 0xFU;

	// system clock not divided
	if(HPRE_val < 8U){
		AHB_pre = 0U;
	}
	else{
		AHB_pre = AHB_PreScaler[HPRE_val - 8U];
	}

	// Calc the APB1 Prescaler
	uint8_t PPRE1_val = 0U, APB1_pre = 0U;
	PPRE1_val = (RCC->CFGR >> 10) & 0x7U;

	// AHB clock not divided
	if(PPRE1_val < 4U){
		APB1_pre = 0U;
	}
	else{
		APB1_pre = APB1_PreScaler[PPRE1_val - 4U];
	}

	return (SystemClk / AHB_pre) / APB1_pre;
}

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
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
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}

}

/*********************************************************************
 * @fn      		  - I2C_Init
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
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	// Temporary variable to calculate the value of control register
	uint32_t tempReg = 0x00000000U;

	// Enable acknowledge bit
	tempReg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
	pI2CHandle->pI2Cx->CR1 |= tempReg;

	// Configure the PREQ field of CR2
	tempReg = 0x00000000U;
	tempReg |= (RCC_GetPCLK1Value() / 1000000U) & 0x3FU; // Convert Hz to Mhz, FREQ bits are 6 last bits
	pI2CHandle->pI2Cx->CR2 |= tempReg;

	// Configure the own address of the slave in OAR1 register
	tempReg = 0x00000000U;
	tempReg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1) & 0x7FU; // 7-bit slave address
	tempReg |= (1 << 14); // Bit 14 must always be kept at 1 (as RM)
	pI2CHandle->pI2Cx->OAR1 |= tempReg;

	// Own address register (OAR2)
	// If setting address in this register it means the slave device has 2 valid addresses

	// Configure the SCL speed via CCR register
	tempReg = 0x00000000U;
	uint16_t CCR_val = 0U;
	// Check the mode of I2C
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM){
		// Standard mode, SCL speed is 100khz
		// Thigh + Tlow = 2*CCR*Tpclk1
		// Tscl = 2*CCR*Tpclk1
		// CCR = Tscl / (2*Tpclk1)
		// CCR = 1/Fscl / (2*(1/Fpclk1)) = 1/Fscl * (Fpclk1 / 2)
		// CCR = Fpclk1 / (2 * Fscl)
		CCR_val = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempReg |= (CCR_val & 0xFFFU);
	}
	else{
		// Configure I2C in Fast mode
		tempReg |= (1 << I2C_CCR_BIT_FS);
		// Configure the DUTY cycle
		tempReg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_BIT_DUTY);
		// CCR is related to DUTY cycle
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			// Tlow/Thigh = 2
			// Tlow + Thigh = 2*CCR*Tpclk1 + CCR*Tpclk1 = 3*CCR*Tpclk1
			// Tscl = 3*CCR*Tpclk1
			// CCR = Tscl / (3*Tpclk1) = Fpclk1 / (3 * Fscl)
			CCR_val = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else{
			// Tlow/Thigh = 16/9
			// Tlow + Thigh = 16*CCR*Tpclk1 + 9*CCR*Tpclk1 = 25*CCR*Tpclk1
			// Tscl = 25*CCR*Tpclk1
			// CCR = Tscl / (25*Tpclk1) = Fpclk1 / (25 * Fscl)
			CCR_val = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempReg |= (CCR_val & 0xFFFU);
	}
	pI2CHandle->pI2Cx->CCR |= tempReg;
}

/*********************************************************************
 * @fn      		  - I2C_DeInit
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
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

}

/*********************************************************************
 * @fn      		  - I2C_PeripheralControl
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
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_BIT_PE);
		//pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
	}else
	{
		pI2Cx->CR1 &= ~(1 << 0);
	}

}

/*********************************************************************
 * @fn      		  - I2C_IRQInterruptConfig
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
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}


/*********************************************************************
 * @fn      		  - I2C_IRQPriorityConfig
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_IPR_BASEADDR + iprx ) |=  ( IRQPriority << shift_amount );

}
