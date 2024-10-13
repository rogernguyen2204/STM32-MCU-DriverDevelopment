#include<stm32f4xx.h>
#include<stm32f407xx_gpio_driver.h>
#include<stdint.h>
#include<string.h>

#define LOW            0
#define BTN_PRESSED   LOW
void delay(void){
	for (uint32_t i;i<500000;i++);
}

int main(void){
	GPIO_Handle_t gpioled, gpio_btn; // initialize the GPIO port
	// local variable usually contains garbage data so should clear all the garbage data before initializing the structure
	memset(&gpioled,0,sizeof(gpioled));  // set each and every member elements of the structure to 0
	memset(&gpio_btn,0,sizeof(gpioled));  // set each and every member elements of the structure to 0

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
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_VERY_HIGH_SPEED;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC,ENABLE);

	GPIO_Init(&gpio_btn);

	GPIO_WriteToOutputPin(GPIOA,GPIO_PIN_NO_8,GPIO_PIN_RESET);
	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15); // priority only matter when more than 1 interrupt involve
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	return 0;

	while(1);

	}

void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_7);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);

}







