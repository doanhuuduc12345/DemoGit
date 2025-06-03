/*
 * main.c
 *
 *  Created on: Apr 18, 2025
 *      Author: doanh
 */


#include "stm32f407xx.h"

int main(void)
{

	// code later
	return 0;
}


void EXTI0_IRQHandler(void)
{
	//handle the interrupt
	GPIO_IRQHandling(0);
}
