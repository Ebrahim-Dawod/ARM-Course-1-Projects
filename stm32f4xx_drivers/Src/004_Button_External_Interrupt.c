/*
 * 003_LED_Button_External.c
 *
 *	Description : Exercise :
 *	Connect an external Button to PD5 pin and toggle the on-Board LED of Discovery Board (PD12 pin.. Green) whenever interrupt is triggered by the button press.
 *
 *	interrupt should be triggered during a falling edge of the button press (High to Low Transition)  (button needs to be (normally in High state) in pull-up configuration) (internal pull-up)
 *
 *
 *  Created on: Jun 29, 2024
 *      Author:
 */



#include "stm32f407xx_gpio_driver.h"

#include "string.h"


#define HIGH 				1
#define LOW 				0
#define BUTTON_PRESSED 		LOW		// for the button Configuration : External Button with Internal Pull-Up Activated by Software   ,   TODO : it differs from one Application to another (Check your Button Connections & Configurations)




// functions Prototypes

void delay(void); // simple delay function

static GPIO_Handle_t GpioLed , GpioButton ; // defined it as global Variables to be Accessed by the ISR Function ("static" to be accessed by the functions inside this c file only)



int main(void)
{
	// clearing / initializing GpioLed , GpioButton (if not cleared/initialized it will carry Garbage value and send it to the registers then corrupt registers values)

	memset(&GpioLed/*access this memory Address to change its value*/ ,0/*set its value to 0*/,sizeof(GpioLed)/*the size of memory that need to be cleared with reference to the Given address*/);
	memset(&GpioButton/*access this memory Address to change its value*/ ,0/*set its value to 0*/,sizeof(GpioButton)/*the size of memory that need to be cleared with reference to the Given address*/);


	/* Configuration & initialization of the Externally connected LED with the +5v line of the stm board through external current limiting resistor */

	GpioLed.pGPIOx = GPIOD ;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_12 ;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT ;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PUSH_PULL ;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH ;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_INTERNAL_PULL_UP_PULL_DOWN;

	GPIO_PeriClockControl(GpioLed.pGPIOx , ENABLE ); 			// Enabling the Clock of GPIO Port D
	GPIO_Init(&GpioLed);		// Initiate the GPIO Registers



	/* Configuration & initialization of the External Button Connected with Software Activated Internal (inside MCU ) Pull-Up Configuration   */

	GpioButton.pGPIOx = GPIOD ;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_5 ;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_INTERNAL_PULL_UP ;

	GPIO_PeriClockControl(GpioButton.pGPIOx , ENABLE );
	GPIO_Init(&GpioButton);

	// IRQ Configuration
	GPIO_IRQInterruptConfig(IRQ_NUMBER_EXTI5_9, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NUMBER_EXTI5_9, NVIC_IRQ_PRIORITY_15);



	while(1);


	return 0 ;
}



void delay(void)
{
	for(uint32_t counter = 0 ; counter< 500000/2 ; counter++);


}


// ISR Implementation

void EXTI9_5_IRQHandler(void)
{

	//Interrupt Handling
	GPIO_IRQHandling(GpioButton.GPIO_PinConfig.GPIO_PinNumber);

	// write your ISR Code here

	delay(); // to de-bounce the noisy actions of a button Press/release transitions
	// Toggle the on Board Green LED (for each button interrupt)
	GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);


}













































































































































































