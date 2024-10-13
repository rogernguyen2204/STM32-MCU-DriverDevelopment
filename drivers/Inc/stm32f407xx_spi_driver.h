/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Apr 15, 2023
 *      Author: nguye
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f4xx.h"

/*
 *  Configuration structure for SPIx peripheral
 */
typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;    // DUPLEX, HALF DUPLEX, SIMPLEX
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;       // slave management type
}SPI_Config_t;

/*
 *  Handle structure for SPIx peripheral
 */
typedef struct{
	SPI_RegDef_t *pSPIx;             // this holds the base address of SPIx(x;0,1,2) peripheral register
	SPI_Config_t SPIConfig;          // this configure the SPI configuration setting
	uint8_t *pTxBuffer;              // to store the APPLICATION Tx buffer address
	uint8_t *pRxBuffer;              // to store the APPLICATION Rx buffer address
	uint32_t TxLen;                  // to store Tx Len
	uint32_t RxLen;                  // to store Rx Len
	uint8_t  TxState;                // to store Tx state
	uint8_t  RxState;                // to Store Rx state
}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER        (1)
#define SPI_DEVICE_MODE_SLAVE         (0)

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FULL_DUPLEX           (1)
#define SPI_BUS_CONFIG_HALF_DUPLEX           (2)
#define SPI_BUS_CONFIG_SIMPLEX_RX_ONLY       (3)

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2                (0)
#define SPI_SCLK_SPEED_DIV4                (1)
#define SPI_SCLK_SPEED_DIV8                (2)
#define SPI_SCLK_SPEED_DIV16               (3)
#define SPI_SCLK_SPEED_DIV32               (4)
#define SPI_SCLK_SPEED_DIV64               (5)
#define SPI_SCLK_SPEED_DIV128              (6)
#define SPI_SCLK_SPEED_DIV256              (7)

/*
 * @SPI_DFF
 */

#define SPI_DFF_8BITS      (0)
#define SPI_DFF_16BITS     (1)

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW      (0)
#define SPI_CPOL_HIGH     (1)

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW      (0)
#define SPI_CPHA_HIGH     (1)

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI       (0)
#define SPI_SSM_EN       (1)

/*
 * SPI related status flags definitions
 */

#define SPI_TXE_FLAG        (1 << SPI_SR_TXE)  // just masking information of the TXE flag (the position in the SR register)
#define SPI_RXNE_FLAG       (1 << SPI_SR_RXNE)
#define SPI_BSY_FLAG        (1 << SPI_SR_BSY)


/*
 * Possible SPI APPLICATION STATE
 */
#define SPI_READY           0
#define SPI_BUSY_IN_RX      1
#define SPI_BUSY_IN_TX      2

/*
 * Possible SPI APPLICATION EVENT
 */
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4
/************************************************************************
 *            APIs supported by this driver
 *            For more information about the APIs check the function definitions
 *************************************************************************/

// Peripheral Clock Setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDis);

// Initialization
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
// interrupt base
uint8_t SPI_SendDataWithIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataWithIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
//IRQ Configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDis);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDis);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDis);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDis);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application call back
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
