/*
 * stm32L152xx_I2C.h
 *
 *  Created on: Dec. 17, 2020
 *      Author: admin
 */

#ifndef INC_STM32L1XX_I2C_H_
#define INC_STM32L1XX_I2C_H_

#include "stm32L152xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint8_t I2C_SCLSpeed;		/*Selecting serial clock speed*/
	uint8_t I2C_DeviceAddress;	/*Setting device address*/
	uint8_t I2C_ACKControl;		/*Setting Ack to enable or disable*/
	uint8_t I2C_FMDutyCycle;	/*Fast mode duty cycle*/
}I2C_Config_t;

/*
 *  Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t 		*pTxBuffer;	/*To store the app. Tx buffer address*/
	uint8_t 		*pRxBuffer;	/*To store the app. Rx buffer address*/
	uint32_t 		TxLen;		/*To store the Tx Len*/
	uint32_t 		RxLen;		/*To store the Rx Len*/
	uint8_t 		TxRxState;	/*To store the communication state*/
	uint8_t 		DevAddr;	/*To store the slave/device address*/
	uint32_t 		RxSize;		/*To store the Rx size*/ /*FIXME suspect this RxSize might not be needed*/
	uint8_t 		RepeatStart;/*To store the repeated start value*/
}I2C_Handle_t;

/*
 * I2C Clock select
 */
#define I2C_SCL_SPEED_STDM	100000
#define I2C_SCL_SPEED_FM	400000

/*
 * I2C Ack control
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * I2C Fast Mode Ducty Cycle
 */
#define I2C_FM_DUTY_2		0
#define I2CFM_DUTY_16_9		1

/*
 * I2C status flag masking values
 */
#define I2C_FLAG_SB						(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR					(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF					(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10					(1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF					(1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE					(1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE					(1 << I2C_SR1_TXE)
#define I2C_FLAG_FLAG_BERR				(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO					(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF						(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR					(1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR					(1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT				(1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT				(1 << I2C_SR1_SMBALERT)

/*
 * I2C application state
 */
#define I2C_READY						0u
#define I2C_BUSY_IN_RX					1u
#define I2C_BUSY_IN_TX					2u

/*
 * I2C application events
 */
#define I2C_EV_TX_DONE					0u
#define I2C_EV_RX_DONE					1u
#define I2C_EV_STOP						2u

/*
 * I2C application error event
 */
#define I2C_ERROR_BERR  				3u
#define I2C_ERROR_ARLO  				4u
#define I2C_ERROR_AF    				5u
#define I2C_ERROR_OVR   				6u
#define I2C_ERROR_TIMEOUT 				7u

/********************************************************************************************
 * 									I2C Driver API's
 * 									Function Prototypes
 ********************************************************************************************/
/*
 * Function	: To initialize I2Cx
 * Param	: Pointer to I2Cx Handler
 * Return	: None
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);

/*
 * Function	: To de-initialize I2Cx
 * Param	: Pointer to I2Cx handle structure
 * Return	: None
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Function	: To setup I2Cx peripheral clock
 * Param	: Pointer to I2Cx base address, Enable or Disable
 * Return	: None
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/*
 * Function	: To send data through I2C peripheral in master mode
 * Param	: Pointer to I2Cx handle structure, Pointer to data buffer, Data length, slave address, repeated start
 * Return	: None
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t RepeatStart);

/*
 * Function	: To Generate start condition
 * Param	: Pointer to I2Cx base address
 * Return	: None
 */
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);

/*
 * Function	: To Generate stop condition
 * Param	: Pointer to I2Cx base address
 * Return	: None
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

/*
 * Function	: To receive data through I2C peripheral in master mode
 * Param	: Pointer to I2Cx handle structure, Pointer to data buffer, Data length, slave address, repeated start
 * Return	: None
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t RepeatStart);

/*
 * Function	: To send data through I2C peripheral in interrupt mode
 * Param	: Pointer to I2Cx handle structure, Pointer to data buffer, Data length, slave address, repeated start
 * Return	: I2C state (Busy in Tx or Busy in Rx or Ready)
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t RepeatStart);

/*
 * Function	: To receive data through I2C peripheral in interrupt mode
 * Param	: Pointer to I2Cx handle structure, Pointer to data buffer, Data length, slave address, repeated start
 * Return	: I2C state (Busy in Tx or Busy in Rx or Ready)
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t RepeatStart);

/*
 * Function	: To configure interrupt priority
 * Param	: Interrupt number, Interrupt priority
 * Return	: None
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Function	: To configure interrupt
 * Param	: Interrupt number, Enable or Disable
 * Return	: None
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);

/*
 * Function	: Interrupt handler for I2C event
 * Param	: Pointer to I2Cx handle structure
 * Return	: None
 */
void I2C_EV_IRQHandler(I2C_Handle_t *pI2CHandle);

/*
 * Function	: Interrupt handler for I2C error
 * Param	: Pointer to I2Cx handle structure
 * Return	: None
 */
void I2C_ER_IRQHandler(I2C_Handle_t *pI2CHandle);

/*
 * Function	: This function gets the status of the passed I2C flag
 * Param	: Pointer to I2Cx base address, Flag mask value
 * Return	: SET or RESET
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/*
 * Function	: This function disables or enables acking
 * Param	: Pointer to I2Cx base address, Enable or Disable
 * Return	: None
 */
void I2C_AckControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/*
 * Function	: Reset the I2C handle structure and close the transmission
 * Param	: Pointer to I2Cx handle structure
 * Return	: None
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

/*
 * Function	: Reset the I2C handle structure and close the reception
 * Param	: Pointer to I2Cx handle structure
 * Return	: None
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);

/*
 * Function	: This function is called to enable or disable the I2C peripheral
 * Param	: Pointer to I2Cx base address, Enable or Disable
 * Return	: None
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/*
 * Function	: This function is to be called by application
 * Param	: Pointer to I2Cx handle, AppEvent
 * Return	: None
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);


#endif /* INC_STM32L1XX_I2C_H_ */
