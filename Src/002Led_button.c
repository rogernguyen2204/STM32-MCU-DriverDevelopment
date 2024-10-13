/*
 * 002Led_button.c
 *
 *  Created on: Apr 5, 2023
 *      Author: nguye
 */


#include<stm32f4xx.h>
#include<stm32f407xx_gpio_driver.h>
#include<stdint.h>

#define BTN_PRESSED    ENABLE
void delay(void){
	for (uint32_t i;i<50000/2;i++);
}

int main(void){
	GPIO_Handle_t gpioled, gpio_btn; // initialize the GPIO port

	// configure LED
	gpioled.pGPIOx = GPIOD;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_VERY_HIGH_SPEED;
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // output type = push pull, no need to enable pull up pull down resistor
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&gpioled);

	//configure button
	gpio_btn.pGPIOx = GPIOA;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_VERY_HIGH_SPEED;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&gpio_btn);

	// toggle the led with software delay

	while(1){
		if  (GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) == BTN_PRESSED){
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}

	}
	return 0;
}
