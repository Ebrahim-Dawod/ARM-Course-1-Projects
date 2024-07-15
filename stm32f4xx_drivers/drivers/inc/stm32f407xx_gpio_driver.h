/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jun 27, 2024
 *      Author:
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_


#include "stm32f407xx.h"





/*
 * This is a Configuration Structure for a GPIO pin
 */

typedef struct
{
	/* we select uint8_t data type because we will hold numbers from 0 : 15 max */
	uint8_t GPIO_PinNumber;			/* possible values from @GPIO_PIN_NUMBER (Search about this reference) choose pin number from 0 to 15 */
	uint8_t GPIO_PinMode;			/* possible values from @GPIO_PIN_MODES (Search about this reference)  */
	uint8_t GPIO_PinSpeed;			/* possible values from @GPIO_PIN_SPEED (Search about this reference) Change Slew Rate of the Output Signal*/
	uint8_t GPIO_PinPuPdControl;	/* possible values from @GPIO_PIN_PUPD (Search about this reference) control Pull-up or Pull-down  */
	uint8_t GPIO_PinOPType;			/* possible values from @GPIO_PIN_OUT_TYPES (Search about this reference) Type of output pin : Push-Pull or Open Drain  */
	uint8_t GPIO_PinAltFunMode;		/* choose one of the Alternate Functions of the Pin */

}GPIO_PinConfig_t;


/*
 * This is a Handle Structure for a GPIO pin
 */

typedef struct
{

	GPIO_RegDef_t *pGPIOx ;		/* pointer to hold the base address of the GPIO Port like (Base Address of Port A , Base Address of GPIO B ,... etc. ) */
	GPIO_PinConfig_t  GPIO_PinConfig; /* this holds GPIO pin Configuration Settings */

}GPIO_Handle_t;




/* @GPIO_PIN_NUMBER
 * GPIO Pin Number
 * */

#define GPIO_PIN_NUMBER_0		0
#define GPIO_PIN_NUMBER_1		1
#define GPIO_PIN_NUMBER_2		2
#define GPIO_PIN_NUMBER_3		3
#define GPIO_PIN_NUMBER_4		4
#define GPIO_PIN_NUMBER_5		5
#define GPIO_PIN_NUMBER_6		6
#define GPIO_PIN_NUMBER_7		7
#define GPIO_PIN_NUMBER_8		8
#define GPIO_PIN_NUMBER_9		9
#define GPIO_PIN_NUMBER_10		10
#define GPIO_PIN_NUMBER_11		11
#define GPIO_PIN_NUMBER_12		12
#define GPIO_PIN_NUMBER_13		13
#define GPIO_PIN_NUMBER_14		14
#define GPIO_PIN_NUMBER_15		15




/* @GPIO_PIN_MODES
 * GPIO pin Possible modes
 *   */


#define GPIO_MODE_INPUT 		0  		// (RESET STATE) Input pin mode
#define GPIO_MODE_OUTPUT		1		// output pin mode
#define GPIO_MODE_ALT_FN		2		// Alternate Function pin mode
#define GPIO_MODE_ANALOG		3		// Analog pin mode
#define GPIO_MODE_IT_FT			4		// Interrupt mode when Falling edge detection is Triggered on an input pin
#define GPIO_MODE_IT_RT			5		// Interrupt mode when Rising edge detection is Triggered on an input pin
#define GPIO_MODE_IT_FRT		6		// Interrupt mode when Falling or Rising edge detection is Triggered on an input pin

/*	@GPIO_PIN_OUT_TYPES
 *  GPIO pin Possible output types  */

#define GPIO_OUT_TYPE_PUSH_PULL		0  //(RESET STATE)
#define GPIO_OUT_TYPE_OPEN_DRAIN	1


/*	@GPIO_PIN_SPEED
 *  GPIO pin Possible output signal Transition Speed (Slew Rate)  */

#define GPIO_SPEED_ LOW			0		// Low speed means low slew rate
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_HIGH			2
#define GPIO_SPEED_VERY_HIGH	3		//High speed means High Slew Rate


/* @GPIO_PIN_PUPD
 * GPIO pin Pull up and Pull down configuration macro  */

#define GPIO_PIN_NO_INTERNAL_PULL_UP_PULL_DOWN			0			// No internal pull-up , pull down (Disabled)
#define GPIO_PIN_INTERNAL_PULL_UP		1
#define GPIO_PIN_INTERNAL_PULL_DOWN		2



/* function-like macro for Calculating the port code (0 for port A , 1 for port B , 2 for port C , ..etc ) to be selected for SYSCFG_EXTICR Register Configuration
 *
 *
 * @param[in]	 - a given GPIO base address
 *
 * @return   	 - numbers from 0 to 8 representing the corresponding Port name (A to I port)
 *  */
/* this technique is called "C Conditional/Ternary Operator" */
#define GPIO_BASEADDR_TO_PORT_CODE( x )		 (  /*if*/( x == GPIOA ) ?/*return/evaluate*/ 0  : /*else*/  /*continue the code in a new line \ */ \
												/*if*/( x == GPIOB ) ?/*return/evaluate*/ 1  : /*else*/ \
												/*if*/( x == GPIOC ) ?/*return/evaluate*/ 2  : /*else*/ \
												/*if*/( x == GPIOD ) ?/*return/evaluate*/ 3  : /*else*/ \
												/*if*/( x == GPIOE ) ?/*return/evaluate*/ 4  : /*else*/ \
												/*if*/( x == GPIOF ) ?/*return/evaluate*/ 5  : /*else*/ \
												/*if*/( x == GPIOG ) ?/*return/evaluate*/ 6  : /*else*/ \
												/*if*/( x == GPIOH ) ?/*return/evaluate*/ 7  : /*else*/ \
												/*if*/( x == GPIOI ) ?/*return/evaluate*/ 8  : /*else*/ 0 /*Keep on the reset state*/ )



/*EXTI corresponding pin number that triggers the interrupt
 * @EXTI_INTERRUPT_PIN_NUMBER
 *
 * */
#define EXTI_INTERRUPT_PIN_NUMBER 		5	//TODO: Change this pin number to your actual EXTI Interrupt Pin number based on your application



/***********************************************************************************************
 * 								APIs Supported by this driver
 * 					for more info. about APIs check the Function definitions in xx_driver.c file
 *
 ***********************************************************************************************/


/* Peripheral Clock Setup  */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx , uint8_t ClockEnOrDi); /* to Enable/Disable the Peripheral Clock for a given GPIO Base Address  */

/* Init. & DeInit.  */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle); /* Initialize the Registers of a given GPIO Peripheral */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);			/* DeInitialize the Registers of a given GPIO Peripheral --> to send its Registers to its Reset State/Value (RCC Peripheral Reset Register) */


/* data Read & write operations */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx  , uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx  , uint8_t PinNumber , uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx  , uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber);


/* IRQ Configuration & ISR Handling */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber , uint8_t IRQEnOrDi /* Enable or Disable */); // for Disabling & Enabling IRQNumber Interrupt
void GPIO_IRQPriorityConfig(uint8_t IRQNumber , uint32_t IRQPriority );

void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
