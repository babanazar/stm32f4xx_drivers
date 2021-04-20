/*
 * main.c
 *
 *  Created on: Apr 20, 2021
 *      Author: Babanazar
 */

#include "stm32f469xx.h"

int main(void)
{
	return 0;
}

void EXTI0_IRQHandler(void)
{
	// handle the interrupt
	GPIO_IRQHandling(0);
}
