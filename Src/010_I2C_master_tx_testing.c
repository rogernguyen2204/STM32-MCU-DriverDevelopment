/*
 * 010_I2C_master_tx_testing.c
 *
 *  Created on: Jun 13, 2023
 *      Author: nguye
 */


#include "stm32f4xx.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include<string.h>
#include<stdio.h>


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

/*
 * PB6 SCL
 * PB9 SDA
 */

I2C_Handle_t I2C1Handle;
//some data
uint8_t some_data[] = "We are testing I2C master Tx\n";

void I2C1_GPIOInits(void){
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;

	// SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	// SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);

}


void I2C1_Inits(void){
      I2C1Handle.pI2Cx = I2C1;
      I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
      I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x61;
      I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
      I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
      I2C_Init(&I2C1Handle);
}
 // need to change the button
void GPIO_ButtonInit(void){
	GPIO_Handle_t gpio_btn;
	gpio_btn.pGPIOx = GPIOA;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_VERY_HIGH_SPEED;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&gpio_btn);
}

int main(void){
	//GPIO button init
	GPIO_ButtonInit();
	//I2C pin inits
	I2C1_GPIOInits();
	//I2C peripheral configuration
	I2C1_Inits();
	// enable the clock for the i2cx peripheral
	I2C_PeripheralControl(I2C1, ENABLE);
	//wait for button press
	while(1){
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();
		// send some data
		I2C_MasterSendData(&I2C1Handle, some_data, strlen((char*)some_data), 0x68);
	}
}

