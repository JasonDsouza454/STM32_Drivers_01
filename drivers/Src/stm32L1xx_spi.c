/*
 * stm32L1xx_spi.c
 *
 *  Created on: Nov. 15, 2020
 *      Author: admin
 */

#include "stm32L1xx_spi.h"

/********************************************************************************************
 * 									SPI Driver API's
 * 									Function Prototypes
 ********************************************************************************************/
/*
 * Function	: Handle for SPI transmit interrupt
 * Param	: None
 * Return	: None
 */
static void SPI_Txe_Interrupt_Handle(SPI_Handle_t *pSPIHandle);

/*
 * Function	: Handle for SPI receive interrupt
 * Param	: None
 * Return	: None
 */
static void SPI_Rxne_Interrupt_Handle(SPI_Handle_t *pSPIHandle);

/*
 * Function	: Handle for SPI error interrupt due to OVR flag
 * Param	: None
 * Return	: None
 */
static void SPI_Ovr_Interrupt_Handle(SPI_Handle_t *pSPIHandle);


/********************************************************************************************
 * 									SPI Driver API's
 * 									Function Implementation
 ********************************************************************************************/
/***********************************Function Header***********************************
 * Function	: SPI_Init
 * Brief	: This function initializes the SPIx
 * Param	: Pointer to SPI Handler
 * Return	: None
 * Note		: None
 *************************************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	/*Configuring CR1 register*/
	uint32_t TempReg = 0U;

	/*Enable SPI peripheral clock*/
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	/*Configure device mode*/
	TempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2U;

	/*Configure the bus config*/
	if (SPI_BUS_CONFIG_FD == pSPIHandle->SPIConfig.SPI_BusConfig)
	{
		/*BIDI mode should be cleared*/
		TempReg &= ~(1U << SPI_CR1_BIDIMODE);
	}
	else if (SPI_BUS_CONFIG_HD == pSPIHandle->SPIConfig.SPI_BusConfig)
	{
		/*BIDI mode should be set*/
		TempReg |= (1U << SPI_CR1_BIDIMODE);
	}
	else if (SPI_BUS_CONFIG_SIMPLEX_RXONLY == pSPIHandle->SPIConfig.SPI_BusConfig)
	{
		/*BIDI mode should be cleared*/
		TempReg &= ~(1U << SPI_CR1_BIDIMODE);
		/*RXONLY should be set*/
		TempReg |= (1U << SPI_CR1_RXONLY);
	}
	else
	{}

	/*Configuring serial clock speed (baud rate)*/
	TempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	/*Configuring data frame format*/
	TempReg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	/*Configuring CPOL*/
	TempReg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	/*Configure CPHA*/
	TempReg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	/*Configure CPHA*/
	TempReg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = TempReg;

}

/***********************************Function Header***********************************
 * Function	: SPI_DeInit
 * Brief	: This function de-initializes the SPIx
 * Param	: Pointer to SPIx base address
 * Return	: None
 * Note		: None
 *************************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (SPI1 == pSPIx)
	{
		SPI1_REG_RESET();
	}
	else if (SPI2 == pSPIx)
	{
		SPI2_REG_RESET();
	}
	else if (SPI3 == pSPIx)
	{
		SPI3_REG_RESET();
	}
	else
	{}
}

/***********************************Function Header***********************************
 * Function	: SPI_PeriClockControl
 * Brief	: This function enables or disables the peripheral clock for the SPIx
 * Param1	: Pointer to SPIx base address
 * Param2	: Enable or Disable
 * Return	: None
 * Note		: None
 *************************************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (ENABLE == EnOrDi)
	{
		if (SPI1 == pSPIx)
		{
			SPI1_PCLK_EN();
		}
		else if (SPI2 == pSPIx)
		{
			SPI2_PCLK_EN();
		}
		else if (SPI3 == pSPIx)
		{
			SPI3_PCLK_EN();
		}
		else
		{}
	}
	else
	{
		if (SPI1 == pSPIx)
		{
			SPI1_PCLK_DI();
		}
		else if (SPI2 == pSPIx)
		{
			SPI2_PCLK_DI();
		}
		else if (SPI3 == pSPIx)
		{
			SPI3_PCLK_DI();
		}
		else
		{}
	}
}

/***********************************Function Header***********************************
 * Function	: SPI_SendData
 * Brief	: This function transmits data via the SPI peripheral in blocking mode
 * Param1	: Pointer to SPIx base address
 * Param2	: Pointer to the transmit buffer
 * Param3	: Data length
 * Return	: None
 * Note		: This is a blocking call
 *************************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0U)
	{
		while (FLAG_SET == SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE))
		{
			if (pSPIx->CR1 & (1U << SPI_CR1_DFF))
			{
				pSPIx->DR = *(uint16_t *)(pTxBuffer);
				(uint16_t *)(pTxBuffer)++;
				Len--;
				Len--;
			}
			else
			{
				pSPIx->DR = *pTxBuffer;
				pTxBuffer++;
				Len--;
			}
		}
	}
}

/***********************************Function Header***********************************
 * Function	: SPI_ReceiveData
 * Brief	: This function receives data via the SPI peripheral in blocking mode
 * Param1	: Pointer to SPIx base address
 * Param2	: Pointer to the receive buffer
 * Param3	: Data length
 * Return	: None
 * Note		: None
 *************************************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while (Len > 0U)
		{
			while (FLAG_SET == SPI_GetFlagStatus(pSPIx, SPI_FLAG_RXNE))
			{
				if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
				{
					 *(uint16_t *)(pRxBuffer) = pSPIx->DR;
					(uint16_t *)(pRxBuffer)++;
					Len--;
					Len--;
				}
				else
				{
					*pRxBuffer = pSPIx->DR;
					pRxBuffer--;
					Len--;
				}
			}
		}
}

/***********************************Function Header***********************************
 * Function	: SPI_IRQInterruptConfig
 * Brief	: This function configures the SPI interrupts
 * Param1	: Interrupt number
 * Param2	: Interrupt priority
 * Param3	: Enable or Disable
 * Return	: None
 * Note		: None
 *************************************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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
 * Function	: SPI_IRQPriorityConfig
 * Brief	: This function configures the SPI interrupts priority
 * Param1	: Interrupt number
 * Param2	: Interrupt priority
 * Return	: None
 * Note		: None
 *************************************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	/*Find ipr register*/
	uint8_t iprx = IRQNumber / 4U;
	uint8_t iprx_section = IRQNumber % 4U;
	uint8_t shift_amount = (8U * iprx_section) + (8U - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (4U *iprx)) |= (IRQPriority << shift_amount);
}

/***********************************Function Header***********************************
 * Function	: SPI_IRQHandling
 * Brief	: This function implements the interrupt handler
 * Param	: Pointer to SPI Handler
 * Return	: None
 * Note		: None
 *************************************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t FlagState, CR_BitSate;

	/*Interrupt due to transmit*/
	FlagState = pSPIHandle->pSPIx->SR & (1U << SPI_SR_TXE);
	CR_BitSate = pSPIHandle->pSPIx->CR2 & (1U << SPI_CR2_TXEIE);

	if (FlagState && CR_BitSate)
	{
		SPI_Txe_Interrupt_Handle(pSPIHandle);
	}
	/*Interrupt due to receive*/
	FlagState = pSPIHandle->pSPIx->SR & (1U << SPI_SR_RXNE);
	CR_BitSate = pSPIHandle->pSPIx->CR2 & (1U << SPI_CR2_RXNEIE);

	if (FlagState && CR_BitSate)
	{
		SPI_Rxne_Interrupt_Handle(pSPIHandle);
	}
	/*Interrupt due to OVR flag*/
	FlagState = pSPIHandle->pSPIx->SR & (1U << SPI_SR_OVR);
	CR_BitSate = pSPIHandle->pSPIx->CR2 & (1U << SPI_CR2_ERRIE);

	if (FlagState && CR_BitSate)
	{
		SPI_Ovr_Interrupt_Handle(pSPIHandle);
	}
}

/***********************************Function Header***********************************
 * Function	: SPI_GetFlagStatus
 * Brief	: This function gets the status of the passed SPI flag
 * Param1	: Pointer to SPIx base address
 * Param2	: Flag mask value
 * Return	: SET or RESET
 * Note		: None
 *************************************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}
}

/***********************************Function Header***********************************
 * Function	: SPI_PeripheralControl
 * Brief	: This function is called to enable or disable the SPI peripheral
 * Param1	: Pointer to SPIx base address
 * Param2	: Enable or Disable
 * Return	: None
 * Note		: None
 *************************************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (ENABLE == EnOrDi)
	{
		pSPIx->CR1 |= (1U << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1U << SPI_CR1_SPE);
	}
}

/***********************************Function Header***********************************
 * Function	: SPI_SSIConfig
 * Brief	: This function is called to configure the SSI bit
 * Param1	: Pointer to SPIx base address
 * Param2	: Enable or Disable
 * Return	: None
 * Note		: None
 *************************************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (ENABLE == EnOrDi)
	{
		pSPIx->CR1 |= (1U << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1U << SPI_CR1_SSI);
	}
}

/***********************************Function Header***********************************
 * Function	: SPI_SendDataIT
 * Brief	: To send data through SPI peripheral in interrupt mode
 * Param1	: Pointer to SPIx handle structure
 * Param2	: Pointer to transmit buffer
 * Param3	: Data length
 * Return	: State of SPI
 * Note		: None
 *************************************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t CurrentSPIState = pSPIHandle->TxState;
	/*Check if SPI is not busy*/
	if (CurrentSPIState != SPI_BUSY_IN_TX)
	{
		/*Save the Tx buffer and data length address in to the SPI handle*/
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		/*Make SPI state as busy in Tx so that other code cannot take over the SPI until transmission is done*/
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		/*Enable the TXIE control bit to get interrupt whenever TXE flag is set*/
		pSPIHandle->pSPIx->CR2 |= (1U << SPI_CR2_TXEIE);
	}

	return CurrentSPIState;
}

/***********************************Function Header***********************************
 * Function	: SPI_ReceiveDataIT
 * Brief	: To receive data through SPI peripheral in interrupt mode
 * Param1	: Pointer to SPIx handle structure
 * Param2	: Pointer to transmit buffer
 * Param3	: Data length
 * Return	: State of SPI
 * Note		: None
 *************************************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t CurrentSPIState = pSPIHandle->RxState;
	/*Check if SPI is not busy*/
	if (CurrentSPIState != SPI_BUSY_IN_RX)
	{
		/*Save the Rx buffer and data length address in to the SPI handle*/
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		/*Make SPI state as busy in Rx so that other code cannot take over the SPI until transmission is done*/
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		/*Enable the RXIE control bit to get interrupt whenever RXE flag is set*/
		pSPIHandle->pSPIx->CR2 |= (1U << SPI_CR2_RXNEIE);
	}

	return CurrentSPIState;
}

/***********************************Function Header***********************************
 * Function	: SPI_Txe_Interrupt_Handle
 * Brief	: Handle for SPI transmit interrupt
 * Param1	: None
 * Return	: None
 * Note		: None
 *************************************************************************************/
static void SPI_Txe_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	if (pSPIHandle->pSPIx->CR1 & (1U << SPI_CR1_DFF))
	{
		pSPIHandle->pSPIx->DR = *(uint16_t *)(pSPIHandle->pTxBuffer);
		(uint16_t *)(pSPIHandle->pTxBuffer)++;
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
	}
	else
	{
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->pTxBuffer++;
		pSPIHandle->TxLen--;
	}
	if (!pSPIHandle->TxLen)
	{
		/*If TxLen is zero close the SPI transmission and inform application Tx is over and disable the inerrupt*/
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPTL);
	}
}

/***********************************Function Header***********************************
 * Function	: SPI_Rxne_Interrupt_Handle
 * Brief	: Handle for SPI receive interrupt
 * Param1	: None
 * Return	: None
 * Note		: None
 *************************************************************************************/
static void SPI_Rxne_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	if (pSPIHandle->pSPIx->CR1 & (1U << SPI_CR1_DFF))
	{
		*(uint16_t *)(pSPIHandle->pRxBuffer) = (uint16_t ) pSPIHandle->pSPIx->DR;
		(uint16_t *)(pSPIHandle->pRxBuffer)++;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
	}
	else
	{
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen--;
	}
	if (!pSPIHandle->RxLen)
	{
		/*If RxLen is zero close the SPI transmission and inform application Rx is over and disable the interrupt*/
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPTL);
	}

}

/***********************************Function Header***********************************
 * Function	: SPI_Ovr_Interrupt_Handle
 * Brief	: Handle for SPI error interrupt due to OVR flag
 * Param1	: None
 * Return	: None
 * Note		: None
 *************************************************************************************/
static void SPI_Ovr_Interrupt_Handle(SPI_Handle_t *pSPIHandle)
{
	uint32_t Temp;
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		Temp = pSPIHandle->pSPIx->DR;
		Temp = pSPIHandle->pSPIx->SR;
		(void)Temp;
	}
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

/***********************************Function Header***********************************
 * Function	: SPI_ClearOVRFlag
 * Brief	: This function is called to clear the OVR flag
 * Param1	: Pointer to SPIx base address
 * Return	: None
 * Note		: None
 *************************************************************************************/
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint32_t Temp;
	Temp = pSPIx->DR;
	Temp = pSPIx->SR;
	(void)Temp;
}

/***********************************Function Header***********************************
 * Function	: SPI_CloseTransmission
 * Brief	: This function is called to close the SPI transmission
 * Param1	: Pointer to SPIx handle
 * Return	: None
 * Note		: None
 *************************************************************************************/
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1U << SPI_CR2_TXEIE);
	/*Reset the SPI handle*/
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0u;
	pSPIHandle->TxState = SPI_READY;
}

/***********************************Function Header***********************************
 * Function	: SPI_CloseReception
 * Brief	: This function is called to close the SPI reception
 * Param1	: Pointer to SPIx handle
 * Return	: None
 * Note		: None
 *************************************************************************************/
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1U << SPI_CR2_RXNEIE);
	/*Reset the SPI handle*/
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0u;
	pSPIHandle->RxState = SPI_READY;
}

/***********************************Function Header***********************************
 * Function	: SPI_CloseReception
 * Brief	: This function is called to close the SPI reception
 * Param1	: Pointer to SPIx handle
 * Return	: None
 * Note		: None
 *************************************************************************************/
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	/*This is a weak function that can be overwritten by application*/
}
