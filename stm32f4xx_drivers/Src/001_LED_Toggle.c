/*
 * 001_LED_Toggle.c
 *
 *  Created on: Jun 29, 2024
 *      Author:
 */



#include "stm32f407xx_gpio_driver.h"


// functions Prototypes

void delay(void); // simple delay function

int main(void)
{

	GPIO_Handle_t GpioLed ;
	GpioLed.pGPIOx = GPIOD ;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUMBER_12 ;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT ;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PUSH_PULL ;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH ;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_INTERNAL_PULL_UP_PULL_DOWN;



	GPIO_PeriClockControl(GpioLed.pGPIOx , ENABLE ); 			// Enabling the Clock of GPIO Port D
	GPIO_Init(&GpioLed);		// Initiate the GPIO Registers



	while(1)
	{

		GPIO_ToggleOutputPin(GpioLed.pGPIOx , GpioLed.GPIO_PinConfig.GPIO_PinNumber );
		delay();

	}

	return 0 ;
}



void delay(void)
{
	for(uint32_t counter = 0 ; counter< 500000/2 ; counter++);


}



















































































