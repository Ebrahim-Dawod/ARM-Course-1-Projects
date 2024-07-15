/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jul 3, 2024
 *      Author:
 */

#include "stm32f407xx_spi_driver.h"






/* Peripheral Clock Setup*/

/******************************************************************************
 * @fn				- SPI_PeriClockControl
 *
 * @brief			- this function enables or disables peripheral clock for an SPI
 *
 * @param[in]		- base address of one of the SPI peripherals
 * @param[in]		- Peripheral Clock "ENABLE" or "DISABLE" macros
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx , uint8_t ClockEnOrDi) /* to Enable/Disable the Peripheral Clock for a given SPI Base Address  */
{

	if (ClockEnOrDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
		else if (pSPIx == SPI5)
		{
			SPI5_PCLK_EN();
		}
		else if (pSPIx == SPI6)
		{
			SPI6_PCLK_EN();
		}

	}//End of if

	else//if (ClockEnOrDi == DISABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
		else if (pSPIx == SPI5)
		{
			SPI5_PCLK_DI();
		}
		else if (pSPIx == SPI6)
		{
			SPI6_PCLK_DI();
		}
	}//End of else


}//End of function



/* Initialization and De- Initialization */

/******************************************************************************
 * @fn				- SPI_Init
 *
 * @brief			- Initialize the Registers of a given SPI Peripheral
 *
 * @param[in]		- a pointer to a given SPI base address and to the SPI Configurable items , configured by the user
 *
 * @return			- none
 *
 * @note			-
 *
 * */
void SPI_Init(SPI_Handle_t *pSPIHandle) /* Initialize the Registers of an SPI Peripheral */
{

	// First lets Configure the SPI_CR1 Register

	uint32_t temp_register = 0;		// lets do all the configuration bit fields here in this temp. register variable then we copy it to SPI_CR1 register at the end of the function

	// 1. configure the device mode

	temp_register |= (pSPIHandle->SPIConfig.SPI_DeviceMode << 2);


	// 2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULL_DUPLEX)
	{
		//BiDirectional mode should be cleared
		temp_register &= ~(1 << 15); //BIDI MODE bit 15
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUPLEX)
	{
		//BiDirectional mode should be set
		temp_register |= (1 << 15); //BIDI MODE bit 15
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY)
	{
		//BIDI Mode should be cleared
		temp_register &= ~(1 << 15); 		//BIDI MODE bit 15
		//RXONLY bit must be set
		temp_register |= (1 << 10);			// RX ONLY bit 10

	}

	// 3. Configure the Serial Clock Speed

	temp_register |= (pSPIHandle->SPIConfig.SPI_SclkSpeed /*the macro from @SPI_SclkSpeed*/ << 3/*3 is the first Bit number*/); //Baud rate control 3 bits  (3,4,5)


	// 4. Configure the Data Frame Format (DFF)

	temp_register |= (pSPIHandle->SPIConfig.SPI_DFF << 11);		//Data Frame Format bit 11


	// 5. Configure CPOL (Clock Polarization)

	temp_register |= (pSPIHandle->SPIConfig.SPI_CPOL << 1);		//CPOL bit 1


	// 6. Configure CPHA (Clock Phase)

	temp_register |= (pSPIHandle->SPIConfig.SPI_CPHA << 0);		//CPHA bit 0



	// 7. Configure (SSM) Software slave management

	temp_register |= (pSPIHandle->SPIConfig.SPI_SSM << 9);		//SSM bit 9


	pSPIHandle->pSPIx->CR1 = temp_register ;


}//End of function



/******************************************************************************
 * @fn				- SPI_DeInit
 *
 * @brief			- DeInitialize the Registers of a given SPI Peripheral
 * 					  --> to send its Registers to its Reset State/Value (using RCC Peripheral Reset Register)
 *
 * @param[in]		- a pointer to a given SPI base address
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void SPI_DeInit(SPI_RegDef_t *pSPIx)			/* De-Initialize the Registers of an SPI Peripheral --> to reset its Registers to its Reset State/Value (using RCC Peripheral Reset Register) */
{
	if (pSPIx == SPI1)
		{
			SPI1_REGISTERS_RESET();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_REGISTERS_RESET();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_REGISTERS_RESET();
		}


}//End of function





/* in Communication Protocols there are 3 types of data transmitting or receiving Methodologies :
 *
 * 	1. Polling ( Blocking call ) (non interrupt-Based )
 * 	2. Interrupt-Based (Non Blocking call)
 * 	3. DMA Based (Direct Memory Access)
 */



/*
 * Data Send & Receive
 */

/******************************************************************************
 * @fn				- SPI_SendData
 *
 * @brief			- Send data to SPI Tx Buffer Register
 *
 *
 * @param[in]		- a pointer to the base address of a given SPI
 * @param[in]		- a pointer to the TX buffer to access the data to be sent
 * @param[in]		- the size/Length of data transfer : how many bytes this API (Function) should Send
 *
 *
 *
 * @return			- none
 *
 * @note			-
 *
 * */
void SPI_SendData (SPI_RegDef_t *pSPIx ,uint8_t *pTxBuffer /*pointer to the TX buffer to access the data to be sent*/ , uint32_t Len /*the size of data transfer : how many bytes this API (Function) should Send ...the Standard Practice for length size is to be uint32_t or more*/)
{

}

/******************************************************************************
 * @fn				- SPI_ReceiveData
 *
 * @brief			- Receive data from SPI Rx Buffer Register
 *
 *
 * @param[in]		- a pointer to the base address of a given SPI
 * @param[in]		- a pointer to the RX buffer to access the data to be Received
 * @param[in]		- the size/Length of data transfer : how many bytes this API (Function) should Receive
 *
 *
 *
 * @return			- none
 *
 * @note			-
 *
 * */
void SPI_ReceiveData (SPI_RegDef_t *pSPIx , uint8_t *pRxBuffer /*pointer to the RX buffer to access the data to be sent*/ , uint32_t Len /*the size of data transfer : how many bytes this API (Function) should Receive ...the Standard Practice for length size is to be uint32_t or more*/)
{

}



/* IRQ Configuration & ISR Handling */

/******************************************************************************
 * @fn				- SPI_IRQInterruptConfig
 *
 * @brief			- to Enable or Disable the IRQNumber interrupt of the NVIC Registers of ARM Cortex Mx Processor
 *
 * @param[in]		- IRQ Number (refer to my developed device header file at section @IRQ_NUMBER_SPI )
 * @param[in]		- IRQ "Enable" or "Disable" macros
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void SPI_IRQInterruptConfig(uint8_t IRQNumber , uint8_t IRQEnOrDi /* Enable or Disable */) // for Disabling & Enabling IRQNumber Interrupt
{

}


/******************************************************************************
 * @fn				- SPI_IRQPriorityConfig
 *
 * @brief			- to set the priority of a given IRQNumber interrupt of the NVIC Registers of ARM Cortex Mx Processor
 *
 * @param[in]		- IRQ Number (refer to my developed device header file at section @IRQ_NUMBER_SPI )
 * @param[in]		- IRQ Priority (refer to my developed device header file at section @NVIC_PRIORITY_NUMBER)
 *
 * @return			- none
 *
 * @note			- none
 *
 * */

void SPI_IRQPriorityConfig(uint8_t IRQNumber , uint8_t IRQPriority )
{
	/*IRQ number = IRQ Position in the NVIC
	 *which is different from IRQ Priority */


}


/******************************************************************************
 * @fn				- SPI_IRQHandling
 *
 * @brief			- we need to End the interrupt Status so that after executing the interrupt ; the processor return back to its main routine
 * 						if we didn't clear the interrupt status ; then the processor still be Triggered by an interrupt and infinitely repeats the execution of the ISR( Interrupt service routine)
 *
 *
 * @param[in]		- a pointer to a given SPI base address and to the SPI Configurable items , configured by the user
 *
 *
 * @return			- none
 *
 * @note			- none
 *
 **/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

}






