/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Apr 1, 2023
 *      Author: nguye
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_


#include "stm32f4xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;         /*!< possible values from @GPIO_PIN_NUMBER>*/
	uint8_t GPIO_PinMode;           /*!< possible values from @GPIO_PIN_MODES>*/
	uint8_t GPIO_PinSpeed;          /*!< possible values from @GPIO_SPEED>*/
	uint8_t GPIO_PinPuPdControl;    /*!< possible values from @GPIO_PUPD>*/
	uint8_t GPIO_PinOPType;         /*!< possible values from @GPIO_OP_TYPES>*/
	uint8_t GPIO_PinAltFuncMode;
}GPIO_PinConfig_t;

typedef struct
{
	// pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx;             // this hold the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;   // this hold the GPIO pin configuration settings
}GPIO_Handle_t;

 // Macros
//1. GPIO pin possible mode (reference manual)
/*@GPIO_PIN_NUMBER*/
#define GPIO_PIN_NO_0           0
#define GPIO_PIN_NO_1           1
#define GPIO_PIN_NO_2           2
#define GPIO_PIN_NO_3           3
#define GPIO_PIN_NO_4           4
#define GPIO_PIN_NO_5           5
#define GPIO_PIN_NO_6           6
#define GPIO_PIN_NO_7           7
#define GPIO_PIN_NO_8           8
#define GPIO_PIN_NO_9           9
#define GPIO_PIN_NO_10          10
#define GPIO_PIN_NO_11          11
#define GPIO_PIN_NO_12          12
#define GPIO_PIN_NO_13          13
#define GPIO_PIN_NO_14          14
#define GPIO_PIN_NO_15          15






//2. GPIO pin possible mode (reference manual)
/*@GPIO_PIN_MODES*/

#define GPIO_MODE_IN            0
#define GPIO_MODE_OUT           1
#define GPIO_MODE_ALTFN         2
#define GPIO_MODE_ANALOG        3
#define GPIO_MODE_IT_FT         4   //GPIO input mode falling edge
#define GPIO_MODE_IT_RT         5   //GPIO input mode rising edge
#define GPIO_MODE_IT_RFT        6   //GPIO input mode rising-falling edge

//3. GPIO pin possible output type (reference manual)
/*@GPIO_OP_TYPES*/
#define GPIO_OP_TYPE_PP         0    //GPIO output push pull
#define GPIO_OP_TYPE_OD         1    //GPIO output open-drain

//4. GPIO pin possible output speed
/*@GPIO_SPEED*/
#define GPIO_SPEED_LOW          0
#define GPIO_SPEED_MEDIUM       1
#define GPIO_HIGH_SPEED         2
#define GPIO_VERY_HIGH_SPEED    3

//5. GPIO pin possible pull up - pull down configuration macros
/*@GPIO_PUPD*/
#define GPIO_NO_PUPD            0
#define GPIO_PIN_PU             1
#define GPIO_PIN_PD             2

/************************************************************************
 *            APIs supported by this driver
 *            For more information about the APIs check the function definitions
 *************************************************************************/
// Peripheral Clock Setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDis);

// Initialization
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// Data Read and Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx); // 1 port has 16 pins
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value); // no return value (write)
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx ,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//IRQ Configuration and ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDis);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
