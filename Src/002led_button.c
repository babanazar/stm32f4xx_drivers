/*
 * 002led_button.c
 *
 *  Created on: Mar 16, 2021
 *      Author: Babanazar
 */

#include "stm32f469xx.h"


void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t gpioLed, gpioButton;
	gpioLed.pGPIOx = GPIOG;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOG, ENABLE);
	GPIO_Init(&gpioLed);


	gpioButton.pGPIOx = GPIOA;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioButton);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == GPIO_PIN_SET)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOG,  GPIO_PIN_NO_6);
		}
	}

	return 0;
}
