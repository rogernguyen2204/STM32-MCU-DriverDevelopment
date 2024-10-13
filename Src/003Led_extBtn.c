/*
 * 003Led_extBtn.c
 *
 *  Created on: Apr 5, 2023
 *      Author: nguye
 */


#include<stm32f4xx.h>
#include<stm32f407xx_gpio_driver.h>
#include<stdint.h>

#define LOW            0
#define BTN_PRESSED   LOW
void delay(void){
	for (uint32_t i;i<500000;i++);
}

int main(void){
	GPIO_Handle_t gpioled, gpio_btn; // initialize the GPIO port

	// configure LED
	gpioled.pGPIOx = GPIOA;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_VERY_HIGH_SPEED;
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // output type = push pull, no need to enable pull up pull down resistor
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&gpioled);

	//configure button
	gpio_btn.pGPIOx = GPIOC;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_VERY_HIGH_SPEED;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC,ENABLE);

	GPIO_Init(&gpio_btn);

	// turn off the LED

	while(1){
		// turn on the LED
			if  (GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_7) == BTN_PRESSED){
				delay();
				GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
			}
		}
	return 0;
	}
