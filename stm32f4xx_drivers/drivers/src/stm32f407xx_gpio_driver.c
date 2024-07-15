/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jun 27, 2024
 *      Author:
 */


#include "stm32f407xx_gpio_driver.h"




/**************************************** Peripheral Clock Setup  ***************************************/

/******************************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- this function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the GPIO peripheral
 * @param[in]		- Peripheral Clock "Enable" or "Disable" macros
 *
 * @return			- none
 *
 * @note			- none
 *
 * */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx , uint8_t ClockEnOrDi)		/* to Enable/Disable the Peripheral Clock for a given GPIO Base Address  */
{

	if (ClockEnOrDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}//End of if
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}


	}//End of else


} // End of Function


/********************************************* Init. & DeInit. ***************************************/



/******************************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- Initialize the Registers of a given GPIO Peripheral
 *
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			-
 *
 * */



void GPIO_Init(GPIO_Handle_t *pGPIOHandle) /* Initialize the Registers of a given GPIO Peripheral */
{
	uint32_t temp=0;  // temp. Register


	//1. Configure the mode of the GPIO

	//select only non interrupt modes
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); /* PinMode Value (Given by the user) is left shifted by (PinNumber *2) */  /* multiply by 2 because in MODER Register, each pin takes 2 bit field */
		pGPIOHandle->pGPIOx->MODER &= ~(RESET_MASK_2_BIT << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));// RESET 2 Bits (to ensure that 2 bits are cleared )
		pGPIOHandle->pGPIOx->MODER |=  temp; //SET

	}//End of if




	/************** STM32f4x GPIO pins interrupt delivery to the processor ****************
	 *
	 *  the interrupt signal will be delivered to the processor through the EXTI peripheral Circuit block passing in EXTI lines to the NVIC Block (inside the Processor) to the processor core itself
	 *  so we have to Configure every point in the way of the interrupt signal until it reaches the processor safely
	 *
	 *
	 *
	 * 1.in the Peripheral side (outside the processor):
	 *	 According to the vector table:
	 *
	 * 	 EXTI peripheral Circuit Block delivers GPIOx_PIN0 interrupt  	  through EXTI0 line to the NVIC IRQ number 6
	 * 	 EXTI peripheral Circuit Block delivers GPIOx_PIN1 interrupt  	  through EXTI1 line to the NVIC IRQ number 7
	 * 	 EXTI peripheral Circuit Block delivers GPIOx_PIN2 interrupt  	  through EXTI2 line to the NVIC IRQ number 8
	 * 	 EXTI peripheral Circuit Block delivers GPIOx_PIN3 interrupt  	  through EXTI3 line to the NVIC IRQ number 9
	 * 	 EXTI peripheral Circuit Block delivers GPIOx_PIN4 interrupt  	  through EXTI4 line to the NVIC IRQ number 10
	 * 	 EXTI peripheral Circuit Block delivers GPIOx_PIN5:9 interrupt    through EXTI5:9 line to the NVIC IRQ number 23
	 * 	 EXTI peripheral Circuit Block delivers GPIOx_PIN10:15 interrupt  	  through EXTI10:15 line (one Line) to the NVIC IRQ number 40
	 *
	 * 2. GPIO port Name (A,B,C,...) and pin number (0-15) that produces the interrupt is decided by SYSCFG_EXTICR Register Configuration
	 *
	 *
	 * 3. EXTI peripheral Circuit Block does Edge detection (FT,RT) , Enable/Disable of Interrupt delivery to the processor side
	 *
	 * 4. in the Processor side: (refer to the Processor (ARM Cortex M4 ) User Guide to configure NVIC registers   ):
	 *
	 * 	  IRQs are disabled by default , you have to configure the NVIC registers in order to Enable or Disable the interrupt reception on those IRQ numbers
	 *
	 * */


	/******* GPIO Pin Interrupt Configuration for any micro-controller *******
	 *
	 * 1. Pin must be in input mode configuration
	 *
	 * 2. Configure the edge trigger for the EXTI Circuit Block (Rising Edge Trigger , Falling Edge Trigger, Rising & Falling Edge Trigger)(Configuring EXTI Registers)
	 *
	 * 3. Enable interrupt delivery from peripheral to the processor ( on peripheral side (EXTI Registers by using EXTI_IMR register interrupt mask register) )
	 *		make Interrupt request from line x is not masked
	 *
	 * 4. Identify the IRQ number on which the processor accepts the interrupt from that pin
	 *
	 * 5. Configure the IRQ Priority for the identified IRQ number (processor side)(through the NVIC registers)
	 *
	 * 6. Enable interrupt reception on that IRQ number ( processor side) (through the NVIC registers)
	 *
	 * 7. Implement IRQ Handler (tell the processor what code to execute when an interrupt is triggered)
	 *
	 ****************/



	else   // select Interrupt modes
	{

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT )
		{
			// 1. Configure (Set the interrupt bit)the FTSR (Falling edge trigger selection register)
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//Clearing the Corresponding bit in RTSR Register (Optional)(just to ensure that it's not Activated)
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );


		}
		else if ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT )
		{
			// 1. Configure (Set the interrupt bit) in the RTSR (Rising Edge trigger selection register)
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clearing the Corresponding bit in FTSR Register (Optional)(just to ensure that it's not Activated)
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FRT)
		{
			// 1. Configure (SET the interrupt bit) in both FTSR & RTSR registers
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		// 2. Configure the GPIO port & pin selection in SYSCFG_EXTICR register

		uint8_t SYSCFG_EXTICR_register_selector		=  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;  // selecting the  SYSCFG_EXTICR register from EXTICR[4] array
		uint8_t SYSCFG_EXTICR_bit_selector			=  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4 ;	 // selecting the start bit to write the port code in the EXTICR register
		uint8_t port_code = GPIO_BASEADDR_TO_PORT_CODE( pGPIOHandle->pGPIOx );  //Calculating the port code (0 for port A , 1 for port B , 2 for port C , ..etc ) to be selected

		// Enabling SYSCFG clock
		SYSCFG_PCLK_EN();

		// Selecting the port & pin that produce the interrupt
		SYSCFG->EXTICR[SYSCFG_EXTICR_register_selector] &= ~( port_code <<  SYSCFG_EXTICR_bit_selector * 4 ); // clear the bits first
		SYSCFG->EXTICR[SYSCFG_EXTICR_register_selector] |=  ( port_code <<  SYSCFG_EXTICR_bit_selector * 4 ); // set the required bits


		// 3. Enable the interrupt delivery on the EXTI line corresponding to the pin number provided using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // make Interrupt request from line x is not masked


	}//End of else






	////////////////////////////////////////////////////////////////////////////////////////

	//2. Configure the speed (Slew Rate)
	temp =0; // resetting temp variable content to be ready for another use
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(RESET_MASK_2_BIT << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));// RESET 2 Bits (to ensure that 2 bits are cleared )
	pGPIOHandle->pGPIOx->OSPEEDR |= temp ; //SET

	/////////////////////////////////////////////////////////////////////////////////////////////////

	//3. Configure the Pull-up & Pull-Down Settings
	temp =0; // resetting temp variable content to be ready for another use
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~(RESET_MASK_2_BIT<<( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // RESET 2 Bits (to ensure that 2 bits are cleared )
	pGPIOHandle->pGPIOx->PUPDR |=  temp; //SET

	////////////////////////////////////////////////////////////////////////////////////////////////

	//4. Configure the Output Type
	temp =0; // resetting temp variable content to be ready for another use
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ; /* PinOPType Value (Given by the user) is left shifted by (PinNumber )  */  /* we didn't multiply because in OTYPER Register, each pin takes 1 bit field */
	pGPIOHandle->pGPIOx->OTYPER &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// RESET 1 Bit (to ensure that this bit is cleared )
	pGPIOHandle->pGPIOx->OTYPER |=  temp ; //SET

	////////////////////////////////////////////////////////////////////////////////////////////////

	//5. Configure the Alternate Functionality
	temp =0; // resetting temp variable content to be ready for another use

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT_FN )
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= GPIO_PIN_NUMBER_7 )
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));  /* PinAltFunMode Value (Given by the user) is left shifted by (PinNumber * 4)  */  /* multiply by 4 because in AFRL Register, each pin takes 4 bit field */
			pGPIOHandle->pGPIOx->AFRL &=  ~(RESET_MASK_4_BIT << ( 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)) ; // RESET 4 Bits (to ensure that 4 bits are cleared )
			pGPIOHandle->pGPIOx->AFRL |=  temp ;//SET
		}//End of if

		else 	// if ( pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber > GPIO_PIN_NUMBER_7 )
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));  /* PinAltFunMode Value (Given by the user) is left shifted by (PinNumber * 4)  */  /* multiply by 4 because in AFRH Register, each pin takes 4 bit field */
			pGPIOHandle->pGPIOx->AFRH &= ~(RESET_MASK_4_BIT << ( 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)) ; // RESET 4 Bits (to ensure that 4 bits are cleared )
			pGPIOHandle->pGPIOx->AFRH |=  temp ; //SET

		}// End of else
	}//End of if



} // End of function GPIO_Init()


/******************************************************************************
 * @fn				- GPIO_DeInit
 *
 * @brief			- DeInitialize the Registers of a given GPIO Peripheral
 * 					  --> to send its Registers to its Reset State/Value (using RCC Peripheral Reset Register)
 *
 * @param[in]		- the GPIO port peripheral base address
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) 			/* DeInitialize the Registers of a given GPIO Peripheral --> to send its Registers to its Reset State/Value (RCC Peripheral Reset Register) */
{

	if (pGPIOx == GPIOA)
	{
		GPIOA_REGISTERS_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REGISTERS_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REGISTERS_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REGISTERS_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REGISTERS_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REGISTERS_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REGISTERS_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REGISTERS_RESET();
	}
	else if (pGPIOx == GPIOI)
	{
		GPIOI_REGISTERS_RESET();
	}

}//End of function

/****************************** data Read & write operations **********************************/



/******************************************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			- to read an input value from a GPIO pin
 *
 * @param[in]		- GPIO port base address
 * @param[in]		- the pin number of that port
 *
 * @return			- the read input value from the pin (0 or 1)
 *
 * @note			- none
 *
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx  , uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) (pGPIOx->IDR&(1<<PinNumber))>>PinNumber; //read from a given pin from IDR Register
	return value;
}

/******************************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			- to read the 16 bit input value from a GPIO port
 *
 * @param[in]		- GPIO port base address
 *
 * @return			- the read input 16 bit value from the port
 *
 * @note			- none
 *
 * */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

	uint16_t value;
	value = (uint16_t) pGPIOx->IDR; //read from the Entire GPIO port from its corresponding IDR Register
	return value;

}

/******************************************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			- write a bit value to the GPIO output pin
 *
 * @param[in]		- the base address of the GPIO port to which that pin belongs
 * @param[in]		- the pin number
 * @param[in]		- the value to be written to that pin
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber , uint8_t value)
{
	if (value == SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin
		pGPIOx->ODR |= (1<<PinNumber);//SET
	}
	else	// value == RESET
	{
		// write 0 to the pin
		pGPIOx->ODR &= ~(1<<PinNumber);//CLEAR
	}

}


/******************************************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			- write 16 bit value to the GPIO output port
 *
 * @param[in]		- the base address of that GPIO port
 * @param[in]		- the 16 bit value to be written to that port
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx  , uint16_t value)
{


	pGPIOx->ODR = value;

	/* if (value == SET)
	{
		//write 1 to the output data register of the port
		pGPIOx->ODR = 0x0000ffff;//SET the 16 bit/pin of the port
	}
	else // value == RESET
	{
		// write 0 to the port
		pGPIOx->ODR = 0x0;//CLEAR the port
	}*/

}


/******************************************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 * @brief			- to toggle the state of a GPIO pin from High to Low and Vice Versa
 *
 * @param[in]		- the base address of that GPIO port
 * @param[in]		- the GPIO pin number
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber)
{

	pGPIOx->ODR ^= (1<<PinNumber); // XOR-ing the bit value (flip its value)

}


/************************************ IRQ Configuration & ISR Handling ************************************/





/******************************************************************************
 * @fn				- GPIO_IRQInterruptConfig
 *
 * @brief			- to Enable or Disable the IRQNumber interrupt of the NVIC Registers of ARM Cortex Mx Processor
 *
 * @param[in]		- IRQ number
 * @param[in]		- IRQ "Enable" or "Disable" macros
 *
 * @return			- none
 *
 * @note			- none
 *
 * */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber , uint8_t IRQEnOrDi /* Enable or Disable */)
{
	/*	what we Configure in this function are processor Specific registers not the micro-controller registers ....... (refer to processor Ref.Man. / User.Guide/...etc.)
	 *
	 * interrupt Set-Enable Registers NVIC_ISER0 : NVIC_ISER3 (NVIC_ISER4 : 7 are not commonly used) (only to Enable interrupt not for disable)
	 * interrupt Clear-Enable Registers NVIC_ICER0 : NVIC_ICER3 (NVIC_ICER4 : 7 are not commonly used) (only to Disable interrupt not for Enable)
	 * interrupt priority registers		IPR
	 *
	 * NVIC_ISER0 bit 0 corresponds to IRQNumber 0 , NVIC_ISER0 bit 1 corresponds to IRQNumber 1 , NVIC_ISER0 bit 31 corresponds to IRQNumber 31
	 * NVIC_ISER1 bit 0 corresponds to IRQNumber 32 , NVIC_ISER1 bit 1 corresponds to IRQNumber 33 , NVIC_ISER1 bit 31 corresponds to IRQNumber 63
	 * NVIC_ISER2 bit 0 corresponds to IRQNumber 64 , NVIC_ISER2 bit 1 corresponds to IRQNumber 65 , NVIC_ISER2 bit 31 corresponds to IRQNumber 95
	 *
	 * NVIC_ICER0 bit 0 corresponds to IRQNumber 0 , NVIC_ICER0 bit 1 corresponds to IRQNumber 1 , NVIC_ICER0 bit 31 corresponds to IRQNumber 31
	 * NVIC_ICER1 bit 0 corresponds to IRQNumber 32 , NVIC_ICER1 bit 1 corresponds to IRQNumber 33 , NVIC_ICER1 bit 31 corresponds to IRQNumber 63
	 * NVIC_ICER2 bit 0 corresponds to IRQNumber 64 , NVIC_ICER2 bit 1 corresponds to IRQNumber 65 , NVIC_ICER2 bit 31 corresponds to IRQNumber 95
	 *
	 *
	*/

	/*IRQ number = IRQ Position in the NVIC
	 *which is different from IRQ Priority */

	if (IRQEnOrDi == ENABLE ) //if(IRQ is Enabled)
	{
		if(IRQNumber < 32) //IRQNumber values between 0 : 31
		{
			// you have to program NVIC_ISER0 register
			*NVIC_ISER0_BASEADDR |= ( 1 << IRQNumber );

		}
		else if (IRQNumber > 31 && IRQNumber < 64 ) //IRQNumber values between 32 : 63
		{
			// you have to program NVIC_ISER1 register
			*NVIC_ISER1_BASEADDR |= ( 1 << ( IRQNumber % 32 /*Subtract 32 from IRQNumber 32 and above to Start from bit 0 of ISER1 register */ ) );
		}
		else if ( IRQNumber > 63 && IRQNumber < 96 ) //IRQNumber values between 64 : 96  // STM32F407 MCU Vector Table have only 81 IRQ numbers is implemented , so we only implemented IRQ numbers till 96 (ISER2 & ICER2 registers) not more that
		{
			// you have to program NVIC_ISER2 register
			*NVIC_ISER2_BASEADDR |= ( 1 << ( IRQNumber % 64 /*Subtract 64 from IRQNumber 64 and above to Start from bit 0 of ISER2 register */ ) );

		}


	}
	else	// if(IRQEnOrDi == DISABLE)   // if(IRQ is Disabled) // you have to disable IRQ number
	{
		if(IRQNumber < 32) //IRQNumber values between 0 : 31
		{
			// you have to program NVIC_ICER0 register
			*NVIC_ICER0_BASEADDR |= ( 1 << IRQNumber ); //disable IRQ number

		}
		else if (IRQNumber > 31 && IRQNumber < 64 ) //IRQNumber values between 32 : 63
		{
			// you have to program NVIC_ICER1 register 	// disable IRQ number
			*NVIC_ICER1_BASEADDR |= ( 1 << ( IRQNumber % 32 /*Subtract 32 from IRQNumber 32 and above to Start from bit 0 of ICER1 register */) );

		}
		else if ( IRQNumber > 63 && IRQNumber < 96 ) //IRQNumber values between 64 : 96   // STM32F407 MCU Vector Table have only 81 IRQ numbers is implemented , so we only implemented IRQ numbers till 96 (ISER2 & ICER2 registers) not more that
		{
			// you have to program NVIC_ICER2 register	// disable IRQ number
			*NVIC_ICER2_BASEADDR |= ( 1 << ( IRQNumber % 64 /*Subtract 64 from IRQNumber 64 and above to Start from bit 0 of ICER2 register */ ) );

		}

	}//End of else


}//End of function


/******************************************************************************
 * @fn				- GPIO_IRQPriorityConfig
 *
 * @brief			- to set the priority of a given IRQNumber interrupt of the NVIC Registers of ARM Cortex Mx Processor
 *
 * @param[in]		- IRQ Number (refer to my developed device header file at section @IRQ_NUMBER_EXTI )
 * @param[in]		- IRQ Priority
 *
 * @return			- none
 *
 * @note			- none
 *
 * */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber , uint32_t IRQPriority )
{

/*******************************************************************
 * the  NVIC of Cortex M4 Processor have 60 (IPR) Interrupt Priority Registers from IPR0 : IPR59
 * each IPR Register (4Bytes) is divided into 4 sections , each section is 1 Byte (8 bits)
 * each section (1 Byte) of the IPR register stores 1 IRQNumber_PriorityValue
 * IRQNumber_PriorityValue is stored in the 4 most significant bits of each section {1,1,1,1,x,x,x,x} (x:not implemented bit)
 *	(each section (1 Byte) has only its 4 most significant bits available for storing the IRQNumber_PriorityValue )
 * so:
 * IPR0 register stores  {IRQNumber3_PriorityValue(1Byte) , IRQNumber2_PriorityValue(1Byte) , IRQNumber1_PriorityValue(1Byte) , IRQNumber0_PriorityValue(1Byte)}
 * IPR1 register stores  {IRQNumber7_PriorityValue , IRQNumber6_PriorityValue , IRQNumber5_PriorityValue , IRQNumber4_PriorityValue}
 * .
 * .
 * IPRn register stores  {IRQNumber(4n+3)_PriorityValue , IRQNumber(4n+2)_PriorityValue , IRQNumber(4n+1)_PriorityValue , IRQNumber(4n)_PriorityValue}
 * .
 * .
 * IPR59 register stores  {IRQNumber239_PriorityValue , IRQNumber238_PriorityValue , IRQNumber237_PriorityValue , IRQNumber236_PriorityValue }
 *
 *
 */

	// 1. first lets find out the IPR register
	uint8_t IPRx_number = IRQNumber / 4 /*divided by 4 because each IPR register has 4 IRQNumber sections */ ; 	// IPRx_number is from 0 : 59	(60 IPR registers)

	// to find out the section that corresponds to the IRQNumber
	uint8_t IPRx_section_number = IRQNumber % 4 ;	// IPRx_section is from 0 : 3 	(4 sections each section is 1 Byte)(IPR take 4Bytes = 4 sections)

	//(each section (1 Byte) has only its 4 most significant bits available for storing the IRQNumber_PriorityValue )
	uint8_t shift_value = (8/*Multiply 8 bit * section number .. to jump to the required bit in the register  */ * IPRx_section_number) + ( 8/*section size is 8 bits reserved for each IRQNumber*/ - NUMBER_OF_PRIORITY_BITS_IMPLEMENTED);

	/* de-reference the address to get its value*/
	 *( NVIC_IPR_BASEADDR + ( /*Jump to this offset*/IPRx_number ) ) = (IRQPriority <<  shift_value);
	 // As you increment a uint32_t pointer it will be incremented by 32bits (4Bytes) Not by 1 Byte , so we shouldn't multiply IPRx_number * 4 in the previous line
}//End of function








/******************************************************************************
 * @fn				- GPIO_IRQHandling
 *
 * @brief			- we need to End the interrupt Status so that after executing the interrupt ; the processor return back to its main routine
 * 						if we didn't clear the interrupt status ; then the processor still be Triggered by an interrupt and infinitely repeats the execution of the ISR( Interrupt service routine)
 * 						we clear the interrupt status by Clearing the bit of EXTI Pending Register (PR) corresponds to the EXTI (External Interrupt) number
 *
 * @param[in]		- GPIO pin number  from which i want to accept interrupt (to choose a macro value refer to @GPIO_PIN_NUMBER )
 *
 *
 * @return			- none
 *
 * @note			- none
 *
 **/
void GPIO_IRQHandling(uint8_t PinNumber)
{

	/*
	 *
	 *
	 *
	 *
	 * */




	// 1. Implement the ISR (Interrupt Service Routine) Function (i.e. Implement IRQ Handler ) (User Application C Code usually written in main.c file or application.c) (tell the processor what code to execute when an interrupt is triggered)


	/* 2. store the Address of your ISR at the Vector Address location corresponding to the IRQ Number for which you have written the ISR
	 * How to do so :
	 * search for the name of the [your Peripheral]_IRQHandler in the startup file (startup_stm32f407vgtx.s) provided(generated) by STM32Cube IDE or by Keil IDE
	 * for example : EXTI0 ISR name is :  EXTI0_IRQHandler (in the startup file)
	 *
	 * then in your application.c file or main.c ... implement a function called: void EXTI0_IRQHandler(void) ->( ISR has neither input parameters nor return )
	 * the same name as you found in startup file structure object
	 *
	 * why we do so, because the startup file has aligned this Handler name to its Handler Address (vector table address) (using a struct object in assembly)(the same way of assigning group of registers memory Addresses (using typedef struct), you do in device header file (stm32f407xx.h)when you develop the Peripheral Register Definition Structures)
	 ***/


	/*
	 * TODO : don't forget to write the ISR Function in your main/application.c file
	 * the implementation is something like this:
	 *
	 *
		void EXTI0_IRQHandler(void)
		{
			//handle the interrupt
			GPIO_IRQHandling(EXTI_INTERRUPT_PIN_NUMBER); // to clear the interrupt corresponding bit in the pending register	// TODO : Don't forget to Edit the input parameter macro

			//write your ISR (User Application) Code Here

		}

	 *
	 *
	 *
	 *
	 *
	 * */










	// clear the EXTI_PR (EXTI Pending Register) corresponding to the pin number
	if ( EXTI->PR & (1 << PinNumber) ) // if  the EXTI_PR bit position corresponding to that interrupt pin number is Set (the triggered interrupt is actually pended in EXTI_PR register )
	{

		// Clear the Pending Register Bit (By writing 1 : according to STM32F407 Reference Manual )
		EXTI->PR |= (1 << PinNumber);

	}// End of if


}// End of function


















































