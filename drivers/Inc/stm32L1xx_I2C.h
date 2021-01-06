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
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
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
#define I2C_FLAG_FLAG_BERR					(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO					(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF						(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR					(1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR					(1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT				(1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT				(1 << I2C_SR1_SMBALERT)

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
 * Function	: To receive data through I2C peripheral in master mode
 * Param	: Pointer to I2Cx handle structure, Pointer to data buffer, Data length, slave address, repeated start
 * Return	: None
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t RepeatStart);

/*
 * Function	: To send data through I2C peripheral in interrupt mode
 * Param	:
 * Return	:
 */


/*
 * Function	: To receive data through I2C peripheral in interrupt mode
 * Param	:
 * Return	:
 */


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
 * Function	: This function gets the status of the passed I2C flag
 * Param	: Pointer to I2Cx base address, Flag mask value
 * Return	: SET or RESET
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

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
