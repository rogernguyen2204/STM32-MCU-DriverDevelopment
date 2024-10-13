/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: May 20, 2023
 *      Author: nguye
 */


#include "stm32f4xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include<string.h>

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode: 5
 */
void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

void SPI2_GPIOInits(void){
	GPIO_Handle_t SPIpins;
	SPIpins.pGPIOx = GPIOB; // Base address of GPIO handle
	SPIpins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIpins.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	SPIpins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIpins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIpins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;

	//SCLK
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIpins);

	//MOSI
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIpins);

	//MISO
	//SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIpins);

	//NSS
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIpins);
}


void SPI2_Inits(void){
	SPI_Handle_t SPIHandles;

	SPIHandles.pSPIx = SPI2;
	SPIHandles.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULL_DUPLEX;
	SPIHandles.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPIHandles.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32; // generate maximum sclk, around 8Mhz
	SPIHandles.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPIHandles.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPIHandles.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPIHandles.SPIConfig.SPI_SSM = SPI_SSM_DI; // hardware slave management enabled for NSS pin

	SPI_Init(&SPIHandles);

}

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

	char user_data[] = "Hello World";
	GPIO_ButtonInit();
	// this function is to initialize GPIO pin to behave as SPI pin
	SPI2_GPIOInits();

	// this function is used to initialized the SPI2 peripheral parameters
	SPI2_Inits();
	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI2,ENABLE);
	while(1){

		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();
		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);
		// send length information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2,&dataLen,1);

		// to send data
		SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

		//first let confirm that SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));
		// Disable the peripheral after communication
		SPI_PeripheralControl(SPI2,DISABLE);
	}
	return 0;

}
