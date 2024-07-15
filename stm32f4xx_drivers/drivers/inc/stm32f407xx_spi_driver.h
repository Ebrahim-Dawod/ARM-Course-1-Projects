/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jul 3, 2024
 *      Author:
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_



#include "stm32f407xx.h"



/*
 *  Configuration structure for SPIx peripheral
 */


typedef struct

{
	uint8_t SPI_DeviceMode ;	/* possible values from @SPI_DeviceMode (Search about this reference) Change Mode: whether Master or slave*/
	uint8_t SPI_BusConfig ;		/* possible values from @SPI_BusConfig (Search about this reference) Change bus : Full/Half duplex or simplex */
	uint8_t SPI_SclkSpeed ;		/* possible values from @SPI_SclkSpeed (Search about this reference) Change SPI serial clock speed*/
	uint8_t SPI_DFF ;			/* possible values from @SPI_DFF (Search about this reference) Data Frame Format _ Change data size: whether 8 bit or 16 bit*/
	uint8_t SPI_CPOL ;			/* possible values from @SPI_CPOL (Search about this reference) clock state : clock is low/High when inactive (no communication)*/
	uint8_t SPI_CPHA ;			/* possible values from @SPI_CPHA (Search about this reference) Clock Phase : Data is valid on the 1st / 2nd edge of the clock (Leading/Trailing edge)*/
	uint8_t SPI_SSM ;			/* possible values from @SPI_SSM (Search about this reference) Software SS (slave select) Management : to select the Slave internally by Software*/


}SPI_Config_t;


/*
 * 	Handle structure for SPIx peripheral
 */


typedef struct

{

	SPI_RegDef_t pSPIx;			/* this pointer holds the base address of SPIx (x: 0,1,2) Peripheral (to Access the SPI Registers) */
	SPI_Config_t SPIConfig;		/* this structure holds all the Configurable items of SPIx (x: 0,1,2) Peripheral (to Access the SPI Configurable items .. configured by user) */


}SPI_Handle_t;


/* SPI Device Mode: whether Master or slave
 *
 * @SPI_DeviceMode
 */

#define SPI_DEVICE_MODE_SLAVE		0			//(default mode)
#define SPI_DEVICE_MODE_MASTER		1

/* @debugging SPI: if the SPI peripheral is not Producing Clock even when ensuring that SPI Clock is enabled from the RCC Peripheral
 *
 * then : maybe you incorrectly configured the MCU as a slave (MSTR Bit in SPI_CR1 Register is 0 )
 * solution : to make the MCU Generate SPI Clock then you should ensure that MCU is Configured as a master (MSTR Bit in SPI_CR1 Register is 1 )
 * */

/* SPI bus : Full duplex ,Half duplex or simplex
 *
 * @SPI_BusConfig
 */


#define SPI_BUS_CONFIG_FULL_DUPLEX			1
#define SPI_BUS_CONFIG_HALF_DUPLEX			2
#define SPI_BUS_CONFIG_SIMPLEX_RX_ONLY		3
//#define SPI_BUS_CONFIG_SIMPLEX_TX_ONLY		4 		//not used configuration




/* SPI serial clock speed
 *
 *	SPI Serial Clock Speed Prescaler to divide the speed of the clock by (2,4,6,...)
 *
 * @SPI_SclkSpeed
 */

#define SPI_SCLK_SPEED_DIVISION_2		0		//(default mode)
#define SPI_SCLK_SPEED_DIVISION_4		1
#define SPI_SCLK_SPEED_DIVISION_8		2
#define SPI_SCLK_SPEED_DIVISION_16		3
#define SPI_SCLK_SPEED_DIVISION_32		4
#define SPI_SCLK_SPEED_DIVISION_64		5
#define SPI_SCLK_SPEED_DIVISION_128		6
#define SPI_SCLK_SPEED_DIVISION_256		7


/* SPI Data Frame Format : whether the data size is 8 bits or 16 bits
 *
 * @SPI_DFF
 */

#define SPI_DFF_8_BITS			0			//(default mode)
#define SPI_DFF_16_BITS			1


/* SPI Clock Polarization : ( clock state ) choose whether the clock is low or High when the bus is idle (no communication)
 *
 * @SPI_CPOL
 */


#define SPI_CPOL_LOW			0			//(default mode)the clock is low when the bus is idle (no communication)
#define SPI_CPOL_HIGH			1			//the clock is High when the bus is idle (no communication)



/* SPI Clock Phase : choose whether the Data is valid on the Bus at the 1st or 2nd edge of the clock (Leading/Trailing edge)
 *
 * @SPI_CPHA
 */

#define SPI_CPHA_LOW			0			//(default mode) the 1st edge of the clock (the 1st clock transition) is the 1st data capture edge (and the Data will be valid on the Bus at 1st data capture edge)
#define SPI_CPHA_HIGH			1			//the 2nd edge of the clock (the 2nd clock transition) is the 1st data capture edge (and the Data will be valid on the Bus at 2st data capture edge)




/* SPI Software SS (slave select) Management : to select the Slave internally by Software
 *
 * @SPI_SSM
 */

#define SPI_SSM_DISABLED			0		//(default mode) (Software Slave Select Management is DISABLED) to select the Slave using SS(Active low) Pin by Hardware by pulling the pin to logic Low
#define SPI_SSM_ENABLED				1		//(Software Slave Select Management is ENABLED)to select the Slave internally by Software






/*******************************************************************************************************************
 * 												APIs Supported by this driver
 * 								for more information about the APIs check the function definitions
 *
 *******************************************************************************************************************/


/*
 * Peripheral Clock Setup
 *
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx , uint8_t ClockEnOrDi); /* to Enable/Disable the Peripheral Clock for a given SPI Base Address  */

/*
 * Initialization and De- Initialization
 *
 */
void SPI_Init(SPI_Handle_t *pSPIHandle); /* Initialize the Registers of an SPI Peripheral */

void SPI_DeInit(SPI_RegDef_t *pSPIx);			/* DeInitialize the Registers of an SPI Peripheral --> to send its Registers to its Reset State/Value (RCC Peripheral Reset Register) */






/* in Communication Protocols there are 3 types of data transmitting or receiving Methodologies :
 *
 * 	1. Polling ( Blocking call ) (non interrupt-Based )
 * 	2. Interrupt-Based (Non Blocking call)
 * 	3. DMA Based (Direct Memory Access)
 */



/*
 * Data Send & Receive
 */

void SPI_SendData (SPI_RegDef_t *pSPIx ,uint8_t *pTxBuffer /*pointer to the TX buffer to access the data to be sent*/ , uint32_t Len /*the size of data transfer : how many bytes this API (Function) should Send ...the Standard Practice for length size is to be uint32_t or more*/);
void SPI_ReceiveData (SPI_RegDef_t *pSPIx , uint8_t *pRxBuffer /*pointer to the RX buffer to access the data to be sent*/ , uint32_t Len /*the size of data transfer : how many bytes this API (Function) should Receive ...the Standard Practice for length size is to be uint32_t or more*/);




/* IRQ Configuration & ISR Handling */

void SPI_IRQInterruptConfig(uint8_t IRQNumber , uint8_t IRQEnOrDi /* Enable or Disable */); // for Disabling & Enabling IRQNumber Interrupt
void SPI_IRQPriorityConfig(uint8_t IRQNumber , uint8_t IRQPriority );

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);








#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
