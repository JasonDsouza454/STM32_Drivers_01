/*
 * stm32L1xx_I2C.c
 *
 *  Created on: Dec. 17, 2020
 *      Author: admin
 */

#include "stm32L1xx_I2C.h"
#include "stm32L1xx_rcc.h"

/********************************************************************************************
 * 									I2C Driver Static API's
 * 									Function Prototypes
 ********************************************************************************************/

/*
 * Function	: To execute the address phase in write mode
 * Param	: Pointer to I2Cx base address, Slave address
 * Return	: None
 */
static void I2C_AddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);

/*
 * Function	: To execute the address phase in read mode
 * Param	: Pointer to I2Cx base address, Slave address
 * Return	: None
 */
static void I2C_AddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);

/*
 * Function	: To clear ADDR flag
 * Param	: Pointer to I2C Handler
 * Return	: None
 */
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle);

/*
 * Function	: Interrupt handler for I2C Tx in master helper function to IRQ handler
 * Param	: Pointer to I2C Handler
 * Return	: None
 */
static void I2C_MasterTxInterruptHandle(I2C_Handle_t *pI2CHandle);

/*
 * Function	: Interrupt handler for I2C Rx in master helper function to IRQ handler
 * Param	: Pointer to I2C Handler
 * Return	: None
 */
static void I2C_MasterRxInterruptHandle(I2C_Handle_t *pI2CHandle);

/********************************************************************************************
 * 									I2C Driver API's
 * 									Function Implementation
 ********************************************************************************************/

/***********************************Function Header***********************************
 * Function	: I2C_GenerateStartCondition
 * Brief	: To Generate start condition
 * Param1	: Pointer to I2Cx base address
 * Return	: None
 * Note		: None
 *************************************************************************************/
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/***********************************Function Header***********************************
 * Function	: I2C_GenerateStopCondition
 * Brief	: To Generate stop condition
 * Param1	: Pointer to I2Cx base address
 * Return	: None
 * Note		: None
 *************************************************************************************/
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/***********************************Function Header***********************************
 * Function	: I2C_AddressPhaseWrite
 * Brief	: To execute the address phase in write mode
 * Param1	: Pointer to I2Cx base address
 * Para3m2	: Slave address
 * Return	: None
 * Note		: None
 *************************************************************************************/
static void I2C_AddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	/*Send the address of the slave with r/nw bit set to w(0) (in total 8 bits)*/
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2Cx->DR = SlaveAddr;
}

/***********************************Function Header***********************************
 * Function	: I2C_AddressPhaseRead
 * Brief	: To execute the address phase in read mode
 * Param1	: Pointer to I2Cx base address
 * Para3m2	: Slave address
 * Return	: None
 * Note		: None
 *************************************************************************************/
static void I2C_AddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	/*Send the address of the slave with r/nw bit set to R(1) (in total 8 bits)*/
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;
	pI2Cx->DR = SlaveAddr;
}

/***********************************Function Header***********************************
 * Function	: I2C_ClearAddrFlag
 * Brief	: To clear ADDR flag
 * Param1	: Pointer to I2C Handler
 * Return	: None
 * Note		: None
 *************************************************************************************/
static void I2C_ClearAddrFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t DummyRead;
	if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		/*Device in master mode*/
		if (I2C_BUSY_IN_RX == pI2CHandle->TxRxState)
		{
			/*In receive mode*/
			if (1u == pI2CHandle->RxSize)
			{
				/*Rx size is only 1*/
				/*Disable Acking and then Clear the Addr flag by dummy reading the SR1 register then reading SR2 register*/
				I2C_AckControl(pI2CHandle->pI2Cx, DISABLE);
				DummyRead = pI2CHandle->pI2Cx->SR1;
				DummyRead = pI2CHandle->pI2Cx->SR2;
				(void)DummyRead;/*To avoid variable unused warning*/
			}
		}
		else if (I2C_BUSY_IN_TX == pI2CHandle->TxRxState)
		{
			/*In transmit mode*/
			/*Clear the Addr flag by dummy reading the SR1 register then reading SR2 register*/
			DummyRead = pI2CHandle->pI2Cx->SR1;
			DummyRead = pI2CHandle->pI2Cx->SR2;
			(void)DummyRead;/*To avoid variable unused warning*/
		}
		else
		{}
	}
}


/***********************************Function Header***********************************
 * Function	: I2C_MasterTxInterruptHandle
 * Brief	: Interrupt handler for I2C Tx in master helper function to IRQ handler
 * Param1	: Pointer to I2Cx handle structure
 * Return	: None
 * Note		: None
 *************************************************************************************/
static void I2C_MasterTxInterruptHandle(I2C_Handle_t *pI2CHandle)
{
	/*TXE flag is set then transfer one byte*/
	if (pI2CHandle->TxLen > 0u)
	{
		/*Load the data into the DR register*/
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		/*Decrement the length and increment the Tx buffer for next byte*/
		pI2CHandle->TxLen--;
		pI2CHandle->pTxBuffer++;
	}
}

/***********************************Function Header***********************************
 * Function	: I2C_MasterRxInterruptHandle
 * Brief	: Interrupt handler for I2C Rx in master helper function to IRQ handler
 * Param1	: Pointer to I2Cx handle structure
 * Return	: None
 * Note		: None
 *************************************************************************************/
static void I2C_MasterRxInterruptHandle(I2C_Handle_t *pI2CHandle)
{
	if (1u == pI2CHandle->RxSize) /*FIXME suspect this should be RxLen*/
	{
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}
	else if (pI2CHandle->RxSize > 1u)
	{
		if (2u == pI2CHandle->RxSize) /*FIXME suspect this should be RxLen*/
		{
			/*Disable Acking*/
			I2C_AckControl(pI2CHandle->pI2Cx, DISABLE);
		}
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
		pI2CHandle->pRxBuffer++;
	}
	else
	{
		/*Rx length is 0*/
		/*Generate Stop condition*/
		if (RESET == pI2CHandle->RepeatStart)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}
		/*Reset all the elements of the handle structure*/
		I2C_CloseReceiveData(pI2CHandle);
		/*Notify application about reception complete*/
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_DONE);
	}
}

/***********************************Function Header***********************************
 * Function	: I2C_Init
 * Brief	: This function initializes the I2Cx
 * Param	: Pointer to I2C Handler
 * Return	: None
 * Note		: None
 *************************************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t Tempreg = 0;
	uint16_t CcrValue = 0;
	uint8_t Trise;
	/*Enable I2C peripheral clock*/
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);
	/*Configure ACK in CR1*/
	Tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = Tempreg;
	/*Configure Freq field in CR2*/
	Tempreg = 0;
	Tempreg |= RCC_GetPClk1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (Tempreg & 0x3F);
	/*Configure own address */
	Tempreg = 0;
	Tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	Tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = Tempreg;
	/*CCR calculation and configuration*/
	Tempreg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_STDM)
	{
		/*standard mode*/
		CcrValue = (RCC_GetPClk1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		Tempreg |= (CcrValue & 0xFFF);
	}
	else
	{
		/*fast mode*/
		Tempreg |= (1 << 15);
		Tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if (I2C_FM_DUTY_2 == pI2CHandle->I2C_Config.I2C_FMDutyCycle)
		{
			CcrValue = (RCC_GetPClk1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			CcrValue = (RCC_GetPClk1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		Tempreg |= (CcrValue & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = Tempreg;
	/*Trise configuration*/
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_STDM)
	{
		/*Standard mode*/
		Trise = (RCC_GetPClk1Value() / 1000000U) + 1;
	}
	else
	{
		/*Fast mode*/
		Trise = ((RCC_GetPClk1Value() * 300) / 1000000U) + 1;
	}
	Trise &= 0x3F;
	pI2CHandle->pI2Cx->TRISE = Trise;
}

/***********************************Function Header***********************************
 * Function	: I2C_DeInit
 * Brief	: This function de-initializes the I2Cx
 * Param	: Pointer to I2Cx base address
 * Return	: None
 * Note		: None
 *************************************************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if (I2C1 == pI2Cx)
	{
		I2C1_REG_RESET();
	}
	else if (I2C2 == pI2Cx)
	{
		I2C2_REG_RESET();
	}
	else
	{}
}

/***********************************Function Header***********************************
 * Function	: I2C_PeriClockControl
 * Brief	: This function enables or disables the peripheral clock for the I2Cx
 * Param1	: Pointer to I2Cx base address
 * Param2	: Enable or Disable
 * Return	: None
 * Note		: None
 *************************************************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (ENABLE == EnOrDi)
	{
		if (I2C1 == pI2Cx)
		{
			I2C1_PCLK_EN();
		}
		else if (I2C2 == pI2Cx)
		{
			I2C1_PCLK_EN();
		}
		else
		{}
	}
	else
	{
		if (I2C1 == pI2Cx)
		{
			I2C1_PCLK_DI();
		}
		else if (I2C2 == pI2Cx)
		{
			I2C2_PCLK_DI();
		}
		else
		{}
	}
}

/***********************************Function Header***********************************
 * Function	: I2C_PeripheralControl
 * Brief	: This function is called to enable or disable the I2C peripheral
 * Param1	: Pointer to I2Cx base address
 * Param2	: Enable or Disable
 * Return	: None
 * Note		: None
 *************************************************************************************/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (ENABLE == EnOrDi)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/***********************************Function Header***********************************
 * Function	: I2C_MasterSendData
 * Brief	: To send data through I2C peripheral in master mode
 * Param1	: Pointer to I2Cx handle structure
 * Param2	: Pointer to data buffer
 * Param3	: Data length
 * Param4	: slave address
 * Param5	: Repeated start - yes set or no reset
 * Return	: None
 * Note		: None
 *************************************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t RepeatStart)
{
	/*Generate start condition*/
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	/*Verify start generation is completed by checking the SB flag in SR1*/
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));
	/*Execute Address phase*/
	I2C_AddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);
	/*Verify the address is sent/ matched to slave address by verifying the ADDR flag*/
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));
	/*Clear ADDR flag*/
	I2C_ClearAddrFlag(pI2CHandle);
	/*Verify data register is zero and send data till length is zero*/
	while (Length > 0)
	{
		while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Length--;
	}
	/*When length is zero wait for TXE = 1 and BTF = 1 before generating stop condition*/
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)));
	/*Generate the stop condition*/
	if (RESET == RepeatStart)
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

/***********************************Function Header***********************************
 * Function	: I2C_MasterReceiveData
 * Brief	: To receive data through I2C peripheral in master mode
 * Param1	: Pointer to I2Cx handle structure
 * Param2	: Pointer to data buffer
 * Param3	: Data length
 * Param4	: slave address
 * Param5	: Repeated start - yes set or no reset
 * Return	: None
 * Note		: None
 *************************************************************************************/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t RepeatStart)
{
	uint32_t LoopCounter;
	/*Generate start condition*/
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	/*Verify start generation is completed by checking the SB flag in SR1*/
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));
	/*Execute Address phase*/
	I2C_AddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);
	/*Verify the address is sent/ matched to slave address by verifying the ADDR flag*/
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));
	/*Reading 1byte*/
	if (1u == Length)
	{
		/*Clear ADDR flag*/
		I2C_ClearAddrFlag(pI2CHandle);
		/*Generate stop condition*/
		if (RESET == RepeatStart)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}
		/*Wait till RxNE flag is set*/
		while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));
		/*Read data into buffer*/
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}
	else if (Length > 1u)
	{
		/*Clear ADDR flag*/
		I2C_ClearAddrFlag(pI2CHandle);
		/*Read data until length is zero*/
		for (LoopCounter = Length; LoopCounter > 0; LoopCounter--)
		{
			/*Wait till RxNE flag is set*/
			while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));
			if (2u == LoopCounter)
			{
				/*Disable Acking*/
				I2C_AckControl(pI2CHandle->pI2Cx, DISABLE);
				/*Generate stop condition*/
				if (RESET == RepeatStart)
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}
			/*Read data into buffer & Increment the data buffer*/
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
		}
	}
	else
	{}
	if (I2C_ACK_ENABLE == pI2CHandle->I2C_Config.I2C_ACKControl)
	{
		I2C_AckControl(pI2CHandle->pI2Cx, ENABLE);
	}
}

/***********************************Function Header***********************************
 * Function	: I2C_MasterSendDataIT
 * Brief	: To send data through I2C peripheral in interrupt mode
 * Param1	: Pointer to I2Cx handle structure
 * Param2	: Pointer to data buffer
 * Param3	: Data length
 * Param4	: slave address
 * Param5	: Repeated start - yes set or no reset
 * Return	: I2C state (Busy in Tx or Busy in Rx or Ready)
 * Note		: None
 *************************************************************************************/
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t RepeatStart)
{
	uint8_t I2CState = pI2CHandle->TxRxState;

	if ((I2CState != I2C_BUSY_IN_TX) && (I2CState != I2C_BUSY_IN_RX))
	{
		/*Set the I2C handle with the passed values and set the I2C state to busy in TX*/
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Length;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->RepeatStart = RepeatStart;
		/*Generate start condition*/
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		/*Enable ITBUFEN Control Bit*/
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		/*Enable ITEVTEN Control Bit*/
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		/*Enable ITERREN Control Bit*/
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return I2CState;
}

/***********************************Function Header***********************************
 * Function	: I2C_MasterReceiveDataIT
 * Brief	: To receive data through I2C peripheral in interrupt mode
 * Param1	: Pointer to I2Cx handle structure
 * Param2	: Pointer to data buffer
 * Param3	: Data length
 * Param4	: slave address
 * Param5	: Repeated start - yes set or no reset
 * Return	: I2C state (Busy in Tx or Busy in Rx or Ready)
 * Note		: None
 *************************************************************************************/
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Length, uint8_t SlaveAddr, uint8_t RepeatStart)
{
	uint8_t I2CState = pI2CHandle->TxRxState;

	if( (I2CState != I2C_BUSY_IN_TX) && (I2CState != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Length;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		/*FIXME suspect this RxSize might not be needed*/
		pI2CHandle->RxSize = Length; /*Rxsize is used in the ISR code to manage the data reception*/
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->RepeatStart = RepeatStart;
		/*Generate start condition*/
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		/*Enable ITBUFEN Control Bit*/
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		/*Enable ITEVTEN Control Bit*/
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		/*Enable ITERREN Control Bit*/
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return I2CState;
}

/***********************************Function Header***********************************
 * Function	: I2C_IRQInterruptConfig
 * Brief	: This function configures the I2C interrupts
 * Param1	: Interrupt number
 * Param2	: Interrupt priority
 * Param3	: Enable or Disable
 * Return	: None
 * Note		: None
 *************************************************************************************/
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if (ENABLE == EnOrDi)
	{
		if (IRQNumber <= 31U)
		{
			/*Program ISER0 register*/
			*NVIC_ISER0 = (1U << IRQNumber);
		}
		else if ((IRQNumber > 32U) && (IRQNumber < 64U))
		{
			/*Program ISER1 register*/
			*NVIC_ISER1 = (1U << (IRQNumber % 32U));
		}
		else if ((IRQNumber >= 64U) && (IRQNumber < 96U))
		{
			/*Program ISER2 register*/
			*NVIC_ISER2 = (1U << (IRQNumber % 64U));
		}
		else
		{}
	}
	else
	{
		if (IRQNumber <= 31U)
		{
			/*Program ISER0 register*/
			*NVIC_ICER0 = (1U << IRQNumber);
		}
		else if ((IRQNumber > 32U) && (IRQNumber < 64U))
		{
			/*Program ISER1 register*/
			*NVIC_ICER1 = (1U << (IRQNumber % 32U));
		}
		else if ((IRQNumber >= 64U) && (IRQNumber < 96U))
		{
			/*Program ISER2 register*/
			*NVIC_ICER2 = (1U << (IRQNumber % 64U));
		}
		else
		{}
	}
}

/***********************************Function Header***********************************
 * Function	: I2C_IRQPriorityConfig
 * Brief	: This function configures the I2C interrupts priority
 * Param1	: Interrupt number
 * Param2	: Interrupt priority
 * Return	: None
 * Note		: None
 *************************************************************************************/
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	/*Find ipr register*/
	uint8_t iprx = IRQNumber / 4U;
	uint8_t iprx_section = IRQNumber % 4U;
	uint8_t shift_amount = (8U * iprx_section) + (8U - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (4U *iprx)) |= (IRQPriority << shift_amount);
}

/***********************************Function Header***********************************
 * Function	: I2C_EV_IRQHandler
 * Brief	: Interrupt handler for I2C event
 * Param1	: Pointer to I2Cx handle structure
 * Return	: None
 * Note		: None
 *************************************************************************************/
void I2C_EV_IRQHandler(I2C_Handle_t *pI2CHandle)
{
	/*Interrupt handling for both master and slave mode of a device*/
	uint32_t Temp1, Temp2, Temp3;
	Temp1 = pI2CHandle->pI2Cx->CR2 & (1u << I2C_CR2_ITEVTEN);
	Temp2 = pI2CHandle->pI2Cx->CR2 & (1u << I2C_CR2_ITBUFEN);
	/*Handle For interrupt generated by SB event
	Note : SB flag is only applicable in Master mode*/
	Temp3 = pI2CHandle->pI2Cx->SR1 & (1u << I2C_SR1_SB);
	if(Temp1 && Temp3)
	{
		/*Interrupt generated due to SB event (This will not be executed in slave mode)*/
		if (I2C_BUSY_IN_RX == pI2CHandle->TxRxState)
		{
			/*Execute Address phase*/
			I2C_AddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if (I2C_BUSY_IN_TX == pI2CHandle->TxRxState)
		{
			/*Execute Address phase*/
			I2C_AddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else
		{}
	}
	/*Handle For interrupt generated by ADDR event
	Note : When master mode : Address is sent
		 When Slave mode   : Address matched with own address*/
	Temp3 = pI2CHandle->pI2Cx->SR1 & (1u << I2C_SR1_ADDR);
	if(Temp1 && Temp3)
	{
		/*Clear ADDR flag*/
		I2C_ClearAddrFlag(pI2CHandle);
	}
	/*Handle For interrupt generated by BTF(Byte Transfer Finished) event*/
	Temp3 = pI2CHandle->pI2Cx->SR1 & (1u << I2C_SR1_BTF);
	if(Temp1 && Temp3)
	{
		if (I2C_BUSY_IN_TX == pI2CHandle->TxRxState)
		{
			if (pI2CHandle->pI2Cx->SR1 & (1u << I2C_SR1_TXE))
			{
				/*Both BTF and TXE flags are set*/
				if (0u == pI2CHandle->TxLen)
				{
					/*Generate Stop condition*/
					if (RESET == pI2CHandle->RepeatStart)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}
					/*Reset all the elements of the handle structure*/
					I2C_CloseSendData(pI2CHandle);
					/*Notify application about transmission complete*/
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_DONE);
				}
			}
		}
		else
		{}
	}
	/*Handle For interrupt generated by STOPF event
		Note : Stop detection flag is applicable only slave mode . For master this flag will never be set*/
	Temp3 = pI2CHandle->pI2Cx->SR1 & (1u << I2C_SR1_STOPF);
	if(Temp1 && Temp3)
	{
		/*Clear the STOPF flag*/
		/*To clear 1st read the SR1 register (done before this if condition) then write into the CR1 register*/
		pI2CHandle->pI2Cx->CR1 |= 0x00000;
		/*Notify application about stop is generated*/
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}
	/*Handle For interrupt generated by TXE event */
	Temp3 = pI2CHandle->pI2Cx->SR1 & (1u << I2C_SR1_TXE);
	if(Temp1 && Temp2 && Temp3)
	{
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			if (I2C_BUSY_IN_TX == pI2CHandle->TxRxState)
			{
				I2C_MasterTxInterruptHandle(pI2CHandle);
			}
		}
	}
	/*Handle For interrupt generated by RXNE event*/
	Temp3 = pI2CHandle->pI2Cx->SR1 & (1u << I2C_SR1_RXNE);
	if(Temp1 && Temp2 && Temp3)
	{
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			if (I2C_BUSY_IN_RX == pI2CHandle->TxRxState)
			{
				I2C_MasterRxInterruptHandle(pI2CHandle);
			}
		}
	}
}

/***********************************Function Header***********************************
 * Function	: I2C_CloseSendData
 * Brief	: Reset the I2C handle structure and close the transmission
 * Param1	: Pointer to I2Cx handle structure
 * Return	: None
 * Note		: None
 *************************************************************************************/
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	/*Disable ITBUFEN control bit*/
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	/*Disable ITEVTEN control bit*/
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	/*Reset handle parameters*/
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0u;
}

/***********************************Function Header***********************************
 * Function	: I2C_CloseReceiveData
 * Brief	: Reset the I2C handle structure and close the reception
 * Param1	: Pointer to I2Cx handle structure
 * Return	: None
 * Note		: None
 *************************************************************************************/
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	/*Disable ITBUFEN control bit*/
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	/*Disable ITEVTEN control bit*/
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	/*Reset handle parameters*/
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0u;
	pI2CHandle->RxSize = 0u;
	/*Enable acking*/
	if (I2C_ACK_ENABLE == pI2CHandle->I2C_Config.I2C_ACKControl)
	{
		I2C_AckControl(pI2CHandle->pI2Cx, ENABLE);
	}
}

/***********************************Function Header***********************************
 * Function	: I2C_ER_IRQHandler
 * Brief	: Interrupt handler for I2C error
 * Param1	: Pointer to I2Cx handle structure
 * Return	: None
 * Note		: None
 *************************************************************************************/
void I2C_ER_IRQHandler(I2C_Handle_t *pI2CHandle)
{
	uint32_t Temp1,Temp2;
	/*Know the status of  ITERREN control bit in the CR2*/
	Temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);
/***********************Check for Bus error************************************/
	Temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(Temp1  && Temp2 )
	{
		/*This is Bus error detected*/
		/*clear the buss error flag*/
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);
		/*notify the application about the error*/
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}
/***********************Check for arbitration lost error************************************/
	Temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(Temp1  && Temp2)
	{
		/*This is arbitration lost error*/
		/*clear the arbitration lost error flag*/
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);
		/*notify the application about the error*/
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}
/***********************Check for ACK failure  error************************************/

	Temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(Temp1  && Temp2)
	{
		/*This is ACK failure error*/
		/*clear the ACK failure error flag*/
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);
		/*notify the application about the error */
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	Temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(Temp1  && Temp2)
	{
		/*This is Overrun/underrun*/
		/*clear the Overrun/underrun error flag*/
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);
		/*notify the application about the error*/
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	Temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(Temp1  && Temp2)
	{
		/*This is Time out error*/
		/*clear the Time out error flag*/
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
		/*notify the application about the error*/
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}
}

/***********************************Function Header***********************************
 * Function	: I2C_GetFlagStatus
 * Brief	: This function gets the status of the passed I2C flag
 * Param1	: Pointer to I2Cx base address
 * Param2	: Flag name
 * Return	: None
 * Note		: None
 *************************************************************************************/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}
}

/***********************************Function Header***********************************
 * Function	: I2C_AckControl
 * Brief	: This function disables or enables acking
 * Param1	: Pointer to I2Cx base address
 * Param2	: Enable(1) or Disable(0)
 * Return	: None
 * Note		: None
 *************************************************************************************/
void I2C_AckControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (I2C_ACK_ENABLE == EnOrDi)
	{
		pI2Cx->CR1 |= (1u << I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->CR1 &= ~(1u << I2C_CR1_ACK);
	}
}

/***********************************Function Header***********************************
 * Function	: I2C_CloseReception
 * Brief	: This function is called to close the I2C reception
 * Param1	: Pointer to SPIx handle
 * Return	: None
 * Note		: None
 *************************************************************************************/
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{
	/*This is a weak function that can be overwritten by application*/
}
