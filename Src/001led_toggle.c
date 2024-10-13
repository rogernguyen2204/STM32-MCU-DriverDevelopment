/*
 * 001led_toggle.c
 *
 *  Created on: Apr 3, 2023
 *      Author: nguye
 */


#include "stm32f4xx.h"
#include "stm32f407xx_gpio_driver.h"
#include <stdint.h>

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}
int main(void){

	GPIO_Handle_t gpioled; // initialize the GPIO port
	gpioled.pGPIOx = GPIOA;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_VERY_HIGH_SPEED;
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // output type = push pull, no need to enable pull up pull down resistor
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&gpioled);

	// toggle the led with software delay

	while(1){
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_1);
		delay();
	}
	return 0;
}
