/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Apr 1, 2023
 *      Author: nguye
 */


#include "stm32f407xx_gpio_driver.h"
#include "stm32f4xx.h"
#include<stdint.h>

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param1[in]         - base address of the gpio peripheral
 * @param2[in]         - ENABLE or DISABLE macros
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

// Peripheral Clock Setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDis)
{
	if (EnorDis == ENABLE){
		if (pGPIOx == GPIOA){
			GPIOA_PCLK_EN;
		}else if (pGPIOx == GPIOB){
			GPIOB_PCLK_EN;
		}else if (pGPIOx == GPIOC){
			GPIOC_PCLK_EN;
		}else if (pGPIOx == GPIOD){
			GPIOD_PCLK_EN;
		}else if (pGPIOx == GPIOE){
			GPIOE_PCLK_EN;
		}else if (pGPIOx == GPIOF){
			GPIOF_PCLK_EN;
		}else if (pGPIOx == GPIOG){
			GPIOG_PCLK_EN;
		}else if (pGPIOx == GPIOH){
			GPIOH_PCLK_EN;
		}else if (pGPIOx == GPIOI){
			GPIOI_PCLK_EN;
		}
	}else{
	if (pGPIOx == GPIOA){
		GPIOA_PCLK_DI;
	}else if (pGPIOx == GPIOB){
		GPIOB_PCLK_DI;
	}else if (pGPIOx == GPIOC){
		GPIOC_PCLK_DI;
	}else if (pGPIOx == GPIOD){
		GPIOD_PCLK_DI;
	}else if (pGPIOx == GPIOE){
		GPIOE_PCLK_DI;
	}else if (pGPIOx == GPIOF){
		GPIOF_PCLK_DI;
	}else if (pGPIOx == GPIOG){
		GPIOG_PCLK_DI;
	}else if (pGPIOx == GPIOH){
		GPIOH_PCLK_DI;
	}else if (pGPIOx == GPIOI){
		GPIOI_PCLK_DI;
	}
   }
}

// Initialization
/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function Initialize and configure the  GPIO Port Mode register depends of different modes
 *
 * @param1[in]         - base address of the GPIO peripheral structure
 * @param2[in]         -
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

// Peripheral Clock Setup
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	//enable the clock;
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);
	//1. Configure the mode of the GPIO pin

	// Driver specific so write macros in the gpio driver header file
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		// Non interrupt mode
		// configure the pin mode to specific bit field in port mode register due to the pin number
		temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}else{
		// Interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//1. Configure the FTSR
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR bit
			EXTI ->EXTI_RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//1. Configure the RTSR
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding FTSR bit
			EXTI ->EXTI_FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//1. Configure both  FTSR and RTSR
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding FTSR bit
			EXTI ->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2.  Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = (pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber)/4;
		uint8_t temp2 = (pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber)%4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN;
		SYSCFG->SYSCFG_EXTICR[temp1] = portcode << (temp2 *4);



		//3. Enable the EXTI interrupt delivery using IMR
		EXTI->EXTI_IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}


	temp = 0;
	//2. Configure the speed
	temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//3. Configure the pull up pull down setting
	temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;
	//4. Configure the output type
	temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType) << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;
	//5. Configure the alt functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// configure the alt function register
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << ( 4 * temp2 ) );

	}
}
// Initialization
/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function reset the GPIO Port using the RCC Reset register
 *
 * @param1[in]         - base address of the GPIO peripheral
 * @param2[in]         -
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA){
		    GPIOA_REG_RESET;
		}else if (pGPIOx == GPIOB){
			GPIOB_REG_RESET;
		}else if (pGPIOx == GPIOC){
			GPIOC_REG_RESET;
		}else if (pGPIOx == GPIOD){
			GPIOD_REG_RESET;;
		}else if (pGPIOx == GPIOE){
			GPIOE_REG_RESET;
		}else if (pGPIOx == GPIOF){
			GPIOF_REG_RESET;
		}else if (pGPIOx == GPIOG){
			GPIOH_REG_RESET;
		}else if (pGPIOx == GPIOH){
			GPIOH_REG_RESET;
		}else if (pGPIOx == GPIOI){
			GPIOI_REG_RESET;
		}
}
// Initialization
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function read and return the value (high or low) of the GPIO pin
 *
 * @param1[in]         - base address of the GPIO peripheral
 * @param2[in]         - Pin number
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */



// Data Read and Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
  uint8_t value;

  value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

  return value;

}
// Initialization
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function read and return the value (high or low;1 or 0) of the GPIO Port
 *
 * @param1[in]         - base address of the GPIO peripheral
 * @param2[in]         -
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)((pGPIOx->IDR));  // read from the whole port

	return value;
}

// Initialization
/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function write 1 or 0 to the GPIO output pin
 *
 * @param1[in]         - base address of the GPIO peripheral
 * @param2[in]         - Pin Number
 * @param3[in]         - Value that want to write (1 or 0)
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{ // no return value (write)
	if (Value == GPIO_PIN_SET){
	// write 1 to the output data register at the bitfield corresponding to the pin
		pGPIOx->ODR |= (1<<PinNumber);
	}else{
	// write 1 to the output data register at the bitfield corresponding to the pin
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}
// Initialization
/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function write 1 or 0 to the GPIO output port
 *
 * @param1[in]         - base address of the GPIO peripheral
 * @param2[in]         - Value that want to write (4 byte hexadecimal format)
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx ,uint16_t Value)
{
	pGPIOx->ODR = Value;
}

// Initialization
/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function toggle the output state of GPIO output pin (1 to 0; 0 to 1)
 *
 * @param1[in]         - base address of the GPIO peripheral
 * @param2[in]         - Pin Number
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);   //bitwiseXOR
}


// Initialization
/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             - This function set or clear the NVIC register depends on users'selections
 *
 * @param1[in]         - IRQNumber
 * @param2[in]         - ENABLE or DISABLE (1 or 0)
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
//IRQ Configuration and ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDis)
{
if(EnorDis == ENABLE){ // use interrupt set enable register ISE (from processor side Cortex M4)
	if(IRQNumber <= 31){
		// program ISER0 register
		*NVIC_ISER0 |= 1 << IRQNumber;
	}else if (IRQNumber > 31 && IRQNumber < 64){
		// program ISER1 register
		*NVIC_ISER1 |= (1 << (IRQNumber %32));
	}else if (IRQNumber >= 64 && IRQNumber < 96){
		// program ISER2 register
		*NVIC_ISER2 |= (1 << (IRQNumber %64));
	}
}else{
	if(IRQNumber <= 31){ // use interrupt clear enable register ICE (from processor side Cortex M4)
			// program ICER0 register
		*NVIC_ICER0 |= 1 << IRQNumber;
	}else if (IRQNumber > 31 && IRQNumber < 64){
		// program ICER1 register
		*NVIC_ICER1 |= (1 << (IRQNumber %32));
	}else if (IRQNumber >= 64 && IRQNumber < 96){
		// program ICER2 register
		*NVIC_ICER2 |= (1 << (IRQNumber %64));
	}
}
}

// Initialization
/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param1[in]         - IRQNumber
 * @param2[in]         - IRQPRIORITY NUMBER
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){
	//1. IPR register on M4 processor
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8*iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

// Initialization
/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - THIS FUNCTION CLEAR THE PENDING REGISTER AFTER THE IRQHANDLE HAS BEEN IMPLEMENT TO AVOID INFINITE INTERRUPT
 *
 * @param1[in]         - PIN NUMBER
 * @param2[in]         -
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_IRQHandling(uint8_t PinNumber){
	// clear the exti pr register corresponding to the pin number
	if(EXTI->EXTI_PR & (1 << PinNumber)){
		//clear
		EXTI->EXTI_PR |= (1<< PinNumber);
	}
}
