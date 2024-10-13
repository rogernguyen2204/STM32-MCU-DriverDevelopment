/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Apr 15, 2023
 *      Author: nguye
 */

#include "stm32f407xx_spi_driver.h"
#include "stm32f4xx.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


// Peripheral Clock Setup

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPIx peripherals
 *
 * @param1[in]         - base address of the SPI peripheral
 * @param2[in]         - ENABLE or DISABLE macros
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDis)
{
	if (EnorDis == ENABLE){
			if (pSPIx == SPI1){
				SPI1_PCLK_EN;
			}else if (pSPIx == SPI2){
				SPI2_PCLK_EN;
			}else if (pSPIx == SPI3){
				SPI3_PCLK_EN;
		}else{
			if (pSPIx == SPI1){
				SPI1_PCLK_DI;
			}else if (pSPIx == SPI2){
				SPI2_PCLK_DI;
			}else if (pSPIx == SPI3){
				SPI3_PCLK_DI;
			}
	   }
}
}

// Initialization
/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This API help user initiallize SPI_CR1 register
 *
 * @param1[in]         - base address of SPI_Handle_t
 * @param2[in]         -
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);

    //Configure the SPI_CR1 register
	uint32_t tempreg = 0;

	//1. Configure the device mode
	tempreg |= (pSPIHandle -> SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);
	//2. Configure the bus config
	if (pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FULL_DUPLEX){
		// BIDIMODE should be clear
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HALF_DUPLEX){
		// BIDIMODE should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY){
		//BIDI mode should be clear
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		// RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3. Configure the SPI serial clock speed
	tempreg |= (pSPIHandle -> SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	//4. Configure the DFF
	tempreg |= (pSPIHandle -> SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//5. Configure the CPOL
	tempreg |= (pSPIHandle -> SPIConfig.SPI_CPOL << SPI_CR1_CPOL);
	//6. Configure the CPHA
	tempreg |= (pSPIHandle -> SPIConfig.SPI_CPHA << SPI_CR1_CPHA);
	//7. Configure SSM
	tempreg |= (pSPIHandle -> SPIConfig.SPI_SSM << SPI_CR1_SSM);


	//save the tempreg to SPI_CR1 register
	pSPIHandle->pSPIx->SPI_CR1 = tempreg;
}
// DEInitialization
/*********************************************************************
 * @fn      		  - SPI_DEInit
 *
 * @brief             - This API help user reset SPI peripheral
 *
 * @param1[in]         - base address of SPI_RegDef_t peripheral
 * @param2[in]         -
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1){
		SPI1_REG_RESET;
	}else if (pSPIx == SPI2){
		SPI2_REG_RESET;
	}else if (pSPIx == SPI3){
		SPI3_REG_RESET;
	}
}
// Peripheral Clock Setup

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 * @param1[in]        -
 * @param2[in]        -
 * @param3[in]        -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SPI_SR & FlagName){
		return FLAG_SET;
	}
	    return FLAG_RESET;
}


/*
 * Data Send and Receive
 */

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param1[in]         -
 * @param2[in]         -
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  This is a Blocking Call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
while(Len >0){
	//1. wait until the TXE is set
	while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)== FLAG_RESET); // if TXE = RESET, infinite loop until TXE is SET

	//2. Check the DFF bit in CR1
	if (pSPIx ->SPI_CR1 & (1 << SPI_CR1_DFF)){
		//16 bit DFF
		//a. Load the data into the DR
		pSPIx->SPI_DR = *((uint16_t*)pTxBuffer); // DeReference the data in the Tx buffer, also typecast to 16 bits to match with the DFF
		Len--;
		Len--;
		(uint16_t*)pTxBuffer++;
	}else{
		//8 bit DFF
		pSPIx->SPI_DR = *(pTxBuffer); // DeReference the data in the Tx buffer,
	    Len--;
	    pTxBuffer++;
	}
}
}
// Peripheral Clock Setup

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPIx peripherals
 *
 * @param1[in]         - base address of the SPI peripheral
 * @param2[in]         - ENABLE or DISABLE macros
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	{
	while(Len >0){
		//1. wait until the TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)== FLAG_RESET); // if TXE = RESET, infinite loop until TXE is SET

		//2. Check the DFF bit in CR1
		if (pSPIx ->SPI_CR1 & (1 << SPI_CR1_DFF)){
			//16 bit DFF
			//a. Load the data from DR to Rxbuffer Address
			*((uint16_t*)pRxBuffer) = pSPIx->SPI_DR; // DeReference the data in the Tx buffer, also typecast to 16 bits to match with the DFF
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else{
			//8 bit DFF
			*(pRxBuffer) = pSPIx->SPI_DR; // DeReference the data in the Tx buffer,
			Len--;
			pRxBuffer++;
		}
	}
}
}

/*
 * IRQ Configuration and ISR handling
 */
//IRQ Configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDis)
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


/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param1[in]         - IRQNumber
 * @param2[in]         - IRQPriority
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8*iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}
/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - This function enables or disables SPI peripheral itself
 *
 * @param1[in]         - base address of the SPI peripheral
 * @param2[in]         - ENABLE or DISABLE macros
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDis)
{
	if (EnorDis == 1){
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
	}else{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             - This function enable or disable the SSI bit in the SPI_CR1 register
 *
 * @param1[in]         - base address of the SPI peripheral
 * @param2[in]         - ENABLE or DISABLE macros
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDis){
	if (EnorDis == 1){
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);
	}else{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             - This function enables or disables SSOE bit in the SPI_CR2 register
 *
 * @param1[in]         - base address of the SPI peripheral
 * @param2[in]         - ENABLE or DISABLE macros
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDis){
	if (EnorDis == 1){
		pSPIx->SPI_CR2 |= (1 << SPI_CR2_SSOE);
	}else{
		pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_SSOE);
}

}
/*********************************************************************
 * @fn      		  - SPI_SendDataWithIT
 *
 * @brief             -
 * @param1[in]         -
 * @param2[in]         -
 * @param3[in]         -
 *
 * @return            - state of SPI peripheral
 *
 * @Note              -  none
 */
uint8_t SPI_SendDataWithIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle ->TxState;
	// only happen when the state of SPI is ready
	if (state!= SPI_BUSY_IN_TX){
		//1. Save the Tx buffer address and Len information in some global variables
		 pSPIHandle->pTxBuffer = pTxBuffer;
		 pSPIHandle->TxLen = Len;
		/*2. Mark the SPI state as busy in transmission so that no other code can take
		 over the same SPI peripheral until transmission is over*/
		 pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		 pSPIHandle->pSPIx->SPI_CR2 |= (1<< SPI_CR2_TXEIE);
	}
	return state;
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveDataWithIT
 *
 * @brief             -
 * @param1[in]        -
 * @param2[in]        -
 * @param3[in]        -
 *
 * @return            -  state of SPI peripheral
 *
 * @Note              -  none
 */
uint8_t SPI_ReceiveDataWithIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
uint8_t state = pSPIHandle ->RxState;
if (state!= SPI_BUSY_IN_RX){
	//1. Save the Tx buffer address and Len information in some global variables
	 pSPIHandle->pRxBuffer = pRxBuffer;
	 pSPIHandle->RxLen = Len;
	/*2. Mark the SPI state as busy in transmission so that no other code can take
	 over the same SPI peripheral until transmission is over*/
	 pSPIHandle->RxState = SPI_BUSY_IN_RX;
	//3. Enable the RXNEIE control bit to get interrupt whenever TXE flag is set in SR
	 pSPIHandle->pSPIx->SPI_CR2 |= (1<< SPI_CR2_RXNEIE);
}
return state;
}

/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             -
 * @param1[in]        -
 * @param2[in]        -
 * @param3[in]        -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void SPI_IRQHandling(SPI_Handle_t *pHandle){
//1. First have to check where does the interrupt happened
	uint8_t temp1,temp2;
	// check for TXE
	temp1 = pHandle->pSPIx->SPI_SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->SPI_CR2 &(1 << SPI_CR2_TXEIE);
	if (temp1 && temp2)
	{
		//hanlde TXE
		spi_txe_interrupt_handle(pHandle);
	}
	//check for RXNE
	temp1 = pHandle->pSPIx->SPI_SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->SPI_CR2 &(1 << SPI_CR2_RXNEIE);
	if (temp1 && temp2)
	{
		//hanlde TXE
		spi_rxne_interrupt_handle(pHandle);
	}
	// check for OVR flag (not checking TI frame format and Mode fault)
	temp1 = pHandle->pSPIx->SPI_SR &(1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->SPI_CR2 &(1 << SPI_CR2_ERRIE);

	if(temp1 && temp2){
		// hanlde ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

// some helper function implementations
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	// check the DFF bit in CR1
	if (pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){
		//16 bit DFF
		//a. Load the data into the DR
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuffer); // DeReference the data in the Tx buffer, also typecast to 16 bits to match with the DFF
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;

	}else{
		//8 bit DFF
		pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pTxBuffer; // DeReference the data in the Tx buffer, also typecast to 16 bits to match with the DFF
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	//Tx Len is zero, so close the SPI transmission and inform the application that TX is over
	if (!pSPIHandle-> TxLen){
		//This prevents interrupts from setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	if (pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){
			//16 bit DFF
			//a. Load the data into the DR
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR; // DeReference the data in the Rx buffer, also typecast to 16 bits to match with the DFF
		    pSPIHandle->RxLen -= 2;
		    pSPIHandle->RxLen--;
			pSPIHandle->RxLen--;


	}else{
		//8 bit DFF
		 *(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->SPI_DR; // DeReference the data in the Rx buffer, also typecast to 16 bits to match with the DFF
			pSPIHandle->RxLen--;
			pSPIHandle->RxLen--;
	}

	if (!pSPIHandle-> RxLen){
		//Rx Len is zero, so close the SPI transmission and inform the application that TX is over
		//This prevents interrupts from setting up of TXE flag
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
    //1. clear the OVR flag
	uint8_t temp;
	 if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		 temp = pSPIHandle -> pSPIx->SPI_DR;
		 temp = pSPIHandle -> pSPIx->SPI_SR;

	 }
	 (void)temp;
	//2. Inform the application
	 SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	 uint8_t temp;
	 temp = pSPIx->SPI_DR;
	 temp = pSPIx->SPI_SR;
	 (void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){

	//This is a weak implementation. The user application may override this function
}
