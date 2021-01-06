/*
 * stm32L1xx_spi.h
 *
 *  Created on: Nov. 15, 2020
 *      Author: admin
 */

#ifndef INC_STM32L1XX_SPI_H_
#define INC_STM32L1XX_SPI_H_

#include "stm32L152xx.h"

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode; /*Selecting master mode or slave mode*/
	uint8_t SPI_BusConfig;	/*Selecting bus config as full duplex half duplex or simplex*/
	uint8_t SPI_SclkSpeed;	/*Selecting serial clock speed*/
	uint8_t SPI_DFF;		/*Selecting data frame format 8bit or 16bit*/
	uint8_t SPI_CPOL;		/*Selecting clock polarity*/
	uint8_t SPI_CPHA;		/*Selecting clock phase*/
	uint8_t SPI_SSM;		/*Selecting software slave management*/
}SPI_Config_t;

/*
 *  Handle structure for SPIx peripheral
 */

typedef struct
{
	SPI_RegDef_t *pSPIx; 	/*Holds the base address of SPIx (x = 1, 2, 3) peripheral*/
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer; 	/* To store the app. Tx buffer address */
	uint8_t *pRxBuffer; 	/* To store the app. Rx buffer address */
	uint32_t TxLen; 		/* To store Tx Len */
	uint32_t RxLen; 		/* To store Rx Len */
	uint8_t TxState; 		/* To store Tx state (Ready or Busy)*/
	uint8_t RxState; 		/* To store Rx state (Ready or Busy)*/
}SPI_Handle_t;

/*
 * SPI application states
 */
#define SPI_READY						0
#define SPI_BUSY_IN_RX					1
#define SPI_BUSY_IN_TX					2

/*
 * SPI application events
 */
#define SPI_EVENT_TX_CMPTL				1
#define SPI_EVENT_RX_CMPTL				2
#define SPI_EVENT_OVR_ERR				3

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_SLAVE			0
#define SPI_DEVICE_MODE_MASTER			1

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW					0
#define SPI_CPOL_HIGH					1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW					0
#define SPI_CPHA_HIGH					1

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI						0
#define SPI_SSM_EN						1

/*
 * SPI status flag masking values
 */
#define SPI_FLAG_RXNE					(1 << SPI_SR_RXNE)
#define SPI_FLAG_TXE					(1 << SPI_SR_TXE)
#define SPI_FLAG_CHSIDE					(1 << SPI_SR_CHSIDE)
#define SPI_FLAG_UDR					(1 << SPI_SR_UDR)
#define SPI_FLAG_CRCERR					(1 << SPI_SR_CRCERR)
#define SPI_FLAG_MODF					(1 << SPI_SR_MODF)
#define SPI_FLAG_OVR					(1 << SPI_SR_OVR)
#define SPI_FLAG_BSY					(1 << SPI_SR_BSY)
#define SPI_FLAG_FRE					(1 << SPI_SR_FRE)


/********************************************************************************************
 * 									SPI Driver API's
 * 									Function Prototypes
 ********************************************************************************************/
/*
 * Function	: To initialize SPIx
 * Param	: Pointer to SPIx Handler
 * Return	: None
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);

/*
 * Function	: To de-initialize SPIx
 * Param	: Pointer to SPIx handle structure
 * Return	: None
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Function	: To setup SPIx peripheral clock
 * Param	: Pointer to SPIx base address, Enable or Disable
 * Return	: None
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Function	: To send data through SPI peripheral in blocking mode
 * Param	: Pointer to SPI base address, Pointer to transmit buffer, data length
 * Return	: None
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);

/*
 * Function	: To receive data through SPI peripheral in blocking mode
 * Param	: Pointer to SPI base address, Pointer to receive buffer, data length
 * Return	: None
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * Function	: To send data through SPI peripheral in interrupt mode
 * Param	: Pointer to SPIx handle structure, Pointer to transmit buffer, data length
 * Return	: SPI state
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);

/*
 * Function	: To receive data through SPI peripheral in interrupt mode
 * Param	: Pointer to SPIx handle structure, Pointer to receive buffer, data length
 * Return	: SPI state
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * Function	: To configure interrupt priority
 * Param	: Interrupt number, Interrupt priority
 * Return	: None
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Function	: To configure interrupt
 * Param	: Interrupt number, Enable or Disable
 * Return	: None
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);

/*
 * Function	: Interrupt handler
 * Param	: Pointer to SPI Handler
 * Return	: None
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Function	: This function gets the status of the passed SPI flag
 * Param	: Pointer to SPIx base address, Flag mask value
 * Return	: SET or RESET
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

/*
 * Function	: This function is called to enable or disable the SPI peripheral
 * Param	: Pointer to SPIx base address, Enable or Disable
 * Return	: None
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Function	: This function is called to configure the SSI bit
 * Param	: Pointer to SPIx base address, Enable or Disable
 * Return	: None
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Function	: This function is called to clear the OVR flag
 * Param	: Pointer to SPIx base address
 * Return	: None
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);

/*
 * Function	: This function is called to close the SPI transmission
 * Param	: Pointer to SPIx handle
 * Return	: None
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);

/*
 * Function	: This function is called to close the SPI reception
 * Param	: Pointer to SPIx handle
 * Return	: None
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Function	: This function is to be called by application
 * Param	: Pointer to SPIx handle, AppEvent
 * Return	: None
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);

#endif /* INC_STM32L1XX_SPI_H_ */
