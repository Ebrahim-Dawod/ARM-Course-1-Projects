/*
 * 003_LED_Button_External.c
 *
 *  Created on: Jun 29, 2024
 *      Author:
 */





#include "stm32f407xx_gpio_driver.h"


#define HIGH 				1
#define LOW 				0
#define BUTTON_PRESSED 		LOW		// for the button Configuration : External Button Connected with External Pull-Up Configuration through the +5v line from the stm board  , & No Internal Pull up or Pull Down configuration is Activated by software




// functions Prototypes

void delay(void); // simple delay function

int main(void)
{
	/* Configuration & initialization of the Externally connected LED with the +5v line of the stm board through external current limiting resistor */

	GPIO_Handle_t GpioLed , GpioButton ;
	GpioLed.pGPIOx = GPIOA ;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_8 ;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT ;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PUSH_PULL ;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH ;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_INTERNAL_PULL_UP_PULL_DOWN;

	GPIO_PeriClockControl(GpioLed.pGPIOx , ENABLE ); 			// Enabling the Clock of GPIO Port D
	GPIO_Init(&GpioLed);		// Initiate the GPIO Registers



	/* Configuration & initialization of the External Button Connected with External Pull-Up Configuration through the +5v line from the stm board    */

	GpioButton.pGPIOx = GPIOB ;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_12 ;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_INTERNAL_PULL_UP_PULL_DOWN ;

	GPIO_PeriClockControl(GpioButton.pGPIOx , ENABLE );
	GPIO_Init(&GpioButton);


	while(1)
	{
		if ( GPIO_ReadFromInputPin( GpioButton.pGPIOx  , GpioButton.GPIO_PinConfig.GPIO_PinNumber) == BUTTON_PRESSED ) // read the status of the Button
		{
			GPIO_ToggleOutputPin(GpioLed.pGPIOx , GpioLed.GPIO_PinConfig.GPIO_PinNumber );
			delay(); 	 /* Debouncing the button (wait until Button State Become Stable , to ignore the noisy unintended transitions of the button State (several high/low transitions)  due to mechanical bouncing of the button when pressed then released ) */

		}// End of if
	} // End of while

	return 0 ;
}



void delay(void)
{
	for(uint32_t counter = 0 ; counter< 500000/2 ; counter++);


}















































































































































































