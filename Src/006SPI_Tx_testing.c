/*
 * 006SPI_Tx_testing.c
 *
 *  Created on: Apr 19, 2023
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

void SPI2_GPIOInits(void){
	GPIO_Handle_t SPIpins;
	SPIpins.pGPIOx = GPIOB; // Base address of GPIO handle
	SPIpins.GPIO_PinConfig.GPIO_PinAltFuncMode = GPIO_MODE_ALTFN;
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
	//SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIpins);
}


void SPI2_Inits(void){
	SPI_Handle_t SPIHandles;

	SPIHandles.pSPIx = SPI2;
	SPIHandles.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULL_DUPLEX;
	SPIHandles.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPIHandles.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; // generate maximum sclk, around 8Mhz
	SPIHandles.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPIHandles.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPIHandles.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPIHandles.SPIConfig.SPI_SSM = SPI_SSM_EN; // software slave management enabled for NSS pin

	SPI_Init(&SPIHandles);

}
int main(void){

	char user_data[] = "Hello World";
	// this function is to initialize GPIO pin to behave as SPI pin
	SPI2_GPIOInits();

	// this function is used to initialized the SPI2 peripheral parameters
	SPI2_Inits();

	// this make NSS signal internally high and avoid MODF error
	SPI_SSIConfig(SPI2,ENABLE);

	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,ENABLE);

    // to send data
	SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

	//first let confirm that SPI is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_SR_BSY));
	// Disable the peripheral after communication
	SPI_PeripheralControl(SPI2,DISABLE);

	while(1);



}
