/*
 * traffic_test.c
 *
 *  Created on: Apr 25, 2023
 *      Author: nguye
 */


#include "stm32f4xx.h"
#include "stm32f407xx_gpio_driver.h"
#include <stdint.h>

void delay(void)
{
	for(uint32_t i = 0 ; i < 5000000 ; i ++);
}

void quick_delay(void){
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}
int main(void){

	GPIO_Handle_t Red_light,Yellow_light, Green_light; // initialize the GPIO port
	Red_light.pGPIOx = GPIOA;
	Red_light.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	Red_light.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Red_light.GPIO_PinConfig.GPIO_PinSpeed = GPIO_VERY_HIGH_SPEED;
	Red_light.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // output type = push pull, no need to enable pull up pull down resistor
	Red_light.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&Red_light);

	Yellow_light.pGPIOx = GPIOE;
	Yellow_light.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Yellow_light.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	Yellow_light.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Yellow_light.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	Yellow_light.GPIO_PinConfig.GPIO_PinSpeed = GPIO_VERY_HIGH_SPEED;

	// Turn on the clock
	GPIO_PeriClockControl(GPIOE,ENABLE);

	//Initialize Yellow light
	GPIO_Init(&Yellow_light);

	// Configure Green Light
	Green_light.pGPIOx = GPIOB;
	Green_light.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Green_light.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	Green_light.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Green_light.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	Green_light.GPIO_PinConfig.GPIO_PinSpeed = GPIO_VERY_HIGH_SPEED;

	// Turn on the clock
	GPIO_PeriClockControl(GPIOB,ENABLE);

	//Initialize Green light
	GPIO_Init(&Green_light);

	// toggle the led with software delay

	while(1){
		// Turn on Red light
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_1);
		delay();
		// Turn red light off
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_1);
		quick_delay();
		//Turn the yellow light on
		GPIO_ToggleOutputPin(GPIOE,GPIO_PIN_NO_9);
		quick_delay();
		//Turn the yellow light off
		GPIO_ToggleOutputPin(GPIOE,GPIO_PIN_NO_9);
		quick_delay();
		// Turn the Green light on
		GPIO_ToggleOutputPin(GPIOB,GPIO_PIN_NO_11);
		delay();
		//Turn the Green light off
		GPIO_ToggleOutputPin(GPIOB,GPIO_PIN_NO_11);
		quick_delay();
	}
	return 0;
}
