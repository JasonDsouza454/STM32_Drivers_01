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
 * Function	: To Generate start condition
 * Param	: Pointer to I2Cx base address
 * Return	: None
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);

/*
 * Function	: To Generate stop condition
 * Param	: Pointer to I2Cx base address
 * Return	: None
 */
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

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
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
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
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
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
	uint32_t DummyRead;
	/*Generate start condition*/
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	/*Verify start generation is completed by checking the SB flag in SR1*/
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));
	/*Send the address of the slave with r/nw bit set to w(0) (in total 8 bits)*/
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2CHandle->pI2Cx->DR = SlaveAddr;
	/*Verify the address is sent/ matched to slave address by verifying the ADDR flag*/
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));
	/*Clear the Addr flag by dummy reading the SR1 register then reading SR2 register*/
	DummyRead = pI2CHandle->pI2Cx->SR1;
	DummyRead = pI2CHandle->pI2Cx->SR2;
	(void)DummyRead;/*To avoid variable unused warning*/
	/*Verify data register is zero and send data till length is zero*/
	while (Length > 0)
	{
		while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Length--;
	}
	/*When length is zero wait for TXE = 1 and BTF = 1 befor generating stop condition*/
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
	uint32_t DummyRead, i;
	/*Generate start condition*/
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	/*Verify start generation is completed by checking the SB flag in SR1*/
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));
	/*Send the address of the slave with r/nw bit set to R(1) (in total 8 bits)*/
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;
	pI2CHandle->pI2Cx->DR = SlaveAddr;
	/*Verify the address is sent/ matched to slave address by verifying the ADDR flag*/
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));
	/*Reading 1byte*/
	if (1 == Length)
	{
		/*Disable Acking*/
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
		/*Clear the Addr flag by dummy reading the SR1 register then reading SR2 register*/
		DummyRead = pI2CHandle->pI2Cx->SR1;
		DummyRead = pI2CHandle->pI2Cx->SR2;
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
	else if (Length > 1)
	{
		/*Clear the ADDR flag*/
		DummyRead = pI2CHandle->pI2Cx->SR1;
		DummyRead = pI2CHandle->pI2Cx->SR2;
		/*Read data until length is zero*/
		for (i = Length; i > 0; i--)
		{
			/*Wait till RxNE flag is set*/
			while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));
			if (2 == i)
			{
				/*Disable Acking*/
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
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
	(void)DummyRead; /*To avoid variable unused warning*/
	if (I2C_ACK_ENABLE == pI2CHandle->I2C_Config.I2C_ACKControl)
	{
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
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
