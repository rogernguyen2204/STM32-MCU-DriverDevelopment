/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Jun 6, 2023
 *      Author: nguye
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f4xx.h"

// Configuration structure for I2Cx peripheral
typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

// Handle structure for I2Cx peripheral
typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM    100000
#define I2C_SCL_SPEED_FM4K  400000
#define I2C_SCL_SPEED_FM2K  400000

/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE      1
#define I2C_ACK_DISABLE     0

/*
 * @I2C_ACKControl
 */
#define I2C_FM_DUTY_2       0
#define I2C_FM_DUTY_16_9    1

/*
 * I2C related status flags definitions
 */
#define I2C_FLAG_SB         (1 << I2C_SR1_SB)
#define I2C_FLAG_TXE        (1 << I2C_SR1_TXE)  // just masking information of the TXE flag (the position in the SR register)
#define I2C_FLAG_RXNE       (1 << I2C_SR1_RXNE)
#define I2C_FLAG_ADDR       (1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF        (1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF      (1 << I2C_SR1_STOPF)
#define I2C_FLAG_BERR       (1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO       (1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF         (1 << I2C_SR1_AF)
#define I2C_FLAG_OVR        (1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR     (1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT    (1 << I2C_SR1_TIMEOUT)

/************************************************************************
 *            APIs supported by this driver
 *            For more information about the APIs check the function definitions
 *************************************************************************/

// Peripheral Clock Setup
void I2C_PeriClockControl(I2C_RegDef_t *pSPIx, uint8_t EnorDis);

// Initialization
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data Send and Receive
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr);

/*
 * IRQ Configuration and ISR handling
 */

//IRQ Configuration and ISR handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDis);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


/*
 * Other control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDis);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPLLOutputClock(void);

/*
 * Application call back
 */
void SPI_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
