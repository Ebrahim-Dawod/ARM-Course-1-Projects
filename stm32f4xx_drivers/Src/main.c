/*
 * main.c
 *
 *  Created on: Jul 1, 2024
 *      Author:
 */



#include "stm32f407xx_gpio_driver.h"

int main(void)
{




	return 0 ;
}



void EXTI0_IRQHandler(void)
{
	//handle the interrupt
	GPIO_IRQHandling(EXTI_INTERRUPT_PIN_NUMBER); // to clear the interrupt corresponding bit in the pending register // TODO : Don't forget to Edit the input parameter macro

	/*write your ISR User Application Code Here*/

}























































































