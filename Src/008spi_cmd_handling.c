/*
 * 008spi_cmd_handling.c
 *
 *  Created on: May 29, 2023
 *      Author: nguye
 */

#include "stm32f4xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include<string.h>


// command codes
#define  COMMAND_LED_CTRL          (0X50)
#define  COMMAND_SENSOR_READ       (0X51)
#define  COMMAND_LED_READ          (0X52)
#define  COMMAND_PRINT             (0X53)
#define  COMMAND_ID_READ           (0X54)


#define  LED_ON      (1)
#define  LED_OFF     (0)

//arduino analog pins
#define ANALOG_PIN0    (0)
#define ANALOF_PIN1    (1)
#define ANALOG_PIN2    (2)
#define ANALOG_PIN3    (3)
#define ANALOG_PIN4    (4)

//arduino LED
#define LED_PIN   (9)


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

void SPI2_GPIOInits(void){
	GPIO_Handle_t SPIpins;
	SPIpins.pGPIOx = GPIOB; // Base address of GPIO handle
	SPIpins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;  //
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
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIpins);

	//NSS
	SPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIpins);
}


void SPI2_Inits(void){
	SPI_Handle_t SPIHandles;

	SPIHandles.pSPIx = SPI2;
	SPIHandles.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FULL_DUPLEX;
	SPIHandles.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPIHandles.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // generate maximum sclk, around 8Mhz
	SPIHandles.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPIHandles.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPIHandles.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPIHandles.SPIConfig.SPI_SSM = SPI_SSM_DI; // hardware slave management enabled for NSS pin

	SPI_Init(&SPIHandles);

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

uint8_t SPI_VerifyResponse(uint8_t ackbyte){
	if (ackbyte == 0xF5){
		return 1;
	}
	return 0;
}
int main(void){

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;
	GPIO_ButtonInit();
	// this function is to initialize GPIO pin to behave as SPI pin
	SPI2_GPIOInits();

	// this function is used to initialized the SPI2 peripheral parameters
	SPI2_Inits();

	SPI_SSOEConfig(SPI2,ENABLE);
	while(1){

		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();
		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);
		// Send the first command
		//1. CMD_LED_CTRL     <pin no(1)>    <value(1)>
		uint8_t commndcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];
		//Send the command to slave
		SPI_SendData(SPI2, &commndcode, 1);
		//do dummy read(receive API) to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		// send some dummy bits (1byte) to fetch the response from slave
		// the slave does not initiate data transfer, so it only transfer the data when receiving a data in (how shift register work)
		SPI_SendData(SPI2, &dummy_write, 1);
		//read the  ACK or NACK byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		//check whether the receive byte is ACK or NACK
		if(SPI_VerifyResponse(ackbyte)){

			args[0] = LED_PIN;
			args[1] = LED_ON;
			//send arguments
			SPI_SendData(SPI2, args, 2);
		}

		//2. CMD_SENSOR_READ
		// wait until the button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();

		commndcode = COMMAND_SENSOR_READ;
		//Send the command to slave
		SPI_SendData(SPI2, &commndcode, 1);
		//do dummy read(receive API) to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		// send some dummy bits (1byte) to fetch the response from slave
		// the slave does not initiate data transfer, so it only transfer the data when receiving a data in (how shift register work)
		SPI_SendData(SPI2, &dummy_write, 1);
		//read the  ACK or NACK byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		//check whether the receive byte is ACK or NACK
		if(SPI_VerifyResponse(ackbyte)){

			args[0] = ANALOG_PIN0;

			//send arguments
			SPI_SendData(SPI2, args, 1);
			//do dummy read(receive API) to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);
			//insert some delay so that slave can ready with the data
			delay();
			// the slave does not initiate data transfer, so it only transfer the data when receiving a data in (how shift register work)
			SPI_SendData(SPI2, &dummy_write, 1);
			// receive the data from the sensor that connected to the Slave
			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
		}


		while(SPI_GetFlagStatus(SPI2, SPI_SR_BSY));
		// Disable the peripheral after communication
		SPI_PeripheralControl(SPI2,DISABLE);
	}
	return 0;
}
