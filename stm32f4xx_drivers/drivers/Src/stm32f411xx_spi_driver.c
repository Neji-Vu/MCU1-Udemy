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
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

}
