/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Jun 6, 2023
 *      Author: nguye
 */


#include "stm32f4xx.h"
#include "stm32f407xx_i2C_driver.h"
// helper functions prototype
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

// Helper functions
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){
	// make space for the R/W bit
	SlaveAddr = SlaveAddr << 1;
    SlaveAddr &= ~(1); // SlaveAddr is slave address +r/w bit
    pI2Cx->I2C_DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx){
	// This bit is cleared by software reading SR1 register followed reading SR2, or by hardware when PE=0
	uint32_t dummyRead = pI2Cx ->I2C_SR1;
	dummyRead = pI2Cx ->I2C_SR2;
	(void)dummyRead;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
}


// AHB PreScaler factor
uint16_t AHB_PreScaler[9] = {2,4,8,16,32,64,128,258,512};

// APB1 PreScaler factor

uint16_t APB1_PreScaler[4] = {2,4,8,16};


/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
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
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDis){
	{
		if (EnorDis == ENABLE){
				if (pI2Cx == I2C1){
					I2C1_PCLK_EN;
				}else if (pI2Cx == I2C2){
					I2C2_PCLK_EN;
				}else if (pI2Cx == I2C3){
					I2C3_PCLK_EN;
			}else{
				if (pI2Cx == I2C1){
					I2C1_PCLK_DI;
				}else if (pI2Cx == I2C2){
					I2C2_PCLK_DI;
				}else if (pI2Cx == I2C3){
					I2C3_PCLK_DI;
				}
		   }
	}
	}
}
/*********************************************************************
 * @fn      		   - RCC_GetPCLK1Value
 *
 * @brief              -
 * @param1[in]         -
 * @param2[in]         -
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
uint32_t RCC_GetPLLOutputClock(void){
	return 0;
}
/*********************************************************************
 * @fn      		   - RCC_GetPCLK1Value
 *
 * @brief              - need to get the APB1 Clock value to configure the FREQ bit field in CR2 register
 * @param1[in]         -
 * @param2[in]         -
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
uint32_t RCC_GetPCLK1Value(void){

	uint32_t pclk1, SystemClk;

	uint8_t clksrc,temp,ahbp,apb1p;
	clksrc = ((RCC->RCC_CFGR >> 2) & 0x03);
	if (clksrc == 0){
		SystemClk = 16000000;
	}else if (clksrc == 1){
		SystemClk = 8000000;
	}else if (clksrc == 2){
		SystemClk = RCC_GetPLLOutputClock();
	}


	// find the value of the AHB prescaler (more infor in the data sheet)
	 temp = ((RCC->RCC_CFGR >> 4) &0xF);
	 if  (temp <8){
		 ahbp = 1;
	 }
	 else {
		 ahbp = AHB_PreScaler[temp-9];
	 }

	 // find the value of the APB1 prescaler (more infor in the data sheet)
	 temp = ((RCC->RCC_CFGR >> 10) &0x7);
	 if  (temp <4){
		 apb1p = 1;
	 }
	 else {
		 apb1p = APB1_PreScaler[temp-4];
	 }

	 // calculate PCLK1
	 pclk1 = (SystemClk/ahbp)/apb1p;

	return pclk1;
}

/*********************************************************************
 * @fn      		   - I2C_Init
 *
 * @brief              -
 * @param1[in]         - base address of the I2C peripheral
 * @param2[in]         - ENABLE or DISABLE macros
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t tempreg = 0;
	//enable the clock for I2Cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);
	//1. Control the ACKING bit
	tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->I2C_CR1 = tempreg;

	//2. Configure the CR2_FREQ
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() /1000000U;
	pI2CHandle->pI2Cx->I2C_CR2 = tempreg & 0x3F;

	//3. Configure the OAR1 ( store the slave address - more information check data sheet)
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD71);
	//bit 14 is required to keep HIGH - due to the reference manual
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->I2C_OAR1 = tempreg;

	//4. Configure the CCR bit field of the CCR register
	//a. CCR calculation
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle ->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		// mode is standard mode
		// calculate using the formula present in the reference manual
		ccr_value = (RCC_GetPCLK1Value()/(2*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		//program ccr_value to the CRR bit field in CRR register
		tempreg |= (ccr_value & 0xFFF);
	}else{
		// mode is fast mode
		// Need to configure the fast mode
		tempreg |= (1 << I2C_CCR_FS);
		// Need to configure the duty cycle
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = (RCC_GetPCLK1Value()/(3*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else{
			ccr_value = (RCC_GetPCLK1Value()/(25*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->I2C_CCR = tempreg;

	//5. TRISE configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		// mode is standard mode

		tempreg = (RCC_GetPCLK1Value()/1000000U) + 1;
	}else{
		// mode is fast mode
		tempreg = ((RCC_GetPCLK1Value()*300)/10000000000U) + 1;
	}
	pI2CHandle->pI2Cx->I2C_TRISE = (tempreg & 0x3F);
}


/*********************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             -
 * @param1[in]         -
 * @param2[in]         -
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	{
		if (pI2Cx == I2C1){
			I2C1_REG_RESET;
		}else if (pI2Cx == I2C2){
			I2C2_REG_RESET;
		}else if (pI2Cx == I2C3){
			I2C3_REG_RESET;
		}
	}
}


/*********************************************************************
 * @fn      		  - I2C_MasterSendData
 *
 * @brief             -
 *
 * @param1[in]         -
 * @param2[in]         -
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr){
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm the start generation is completed by checking the SB flag in SR1
    //   Note: Until SB is clear SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	//3. Send the address of the slave with r/w bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx,SlaveAddr);

	//4. Confirm that address phase is complete by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));
	//5. Clear the ADDR flag according to its software sequence
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//6. Send the data until Len becomes 0
	while(Len >0){
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE)){
			pI2CHandle->pI2Cx->I2C_DR = *pTxbuffer;
			pTxbuffer++;
			Len--;
		}
	}

	/*7. When Len becomes zero wait for TXE = 1 and BTF = 1 before generating the STOP condition
	 *   Note: TXE = 1, BTF = 1, means that both SR and DR are empty and next tranmission should begin
	 *   When BTF = 1 SCL will be stretched (pull to LOW)*/
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: Generating STOP condition auto clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


}
/*********************************************************************
 * @fn      		  - I2C_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param1[in]         -
 * @param2[in]         -
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDis){

}

/*********************************************************************
 * @fn      		  - I2C_IRQPriorityConfig
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

}

/*********************************************************************
 * @fn      		  - I2C_PeripheralControl
 *
 * @brief             -
 *
 * @param1[in]         -
 * @param2[in]         -
 * @param3[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDis){
	if (EnorDis == 1){
			pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
		}else{
			pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);
		}
}


/*********************************************************************
 * @fn      		  - I2C_GetFlagStatus
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
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){
	if(pI2Cx->I2C_SR1 & FlagName){
			return FLAG_SET;
		}
		    return FLAG_RESET;
}


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
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){

}
