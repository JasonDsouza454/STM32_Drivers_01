/*
 * stm32L1xx_gpio.c
 *
 *  Created on: Sep 11, 2020
 *      Author: admin
 */

#include "stm32L1xx_gpio.h"



/********************************************************************************************
 * 									GPIO Driver API's
 * 									Function Implementation
 ********************************************************************************************/
/***********************************Function Header***********************************
 * Function	: GPIO_Init
 * Brief	: This function initializes the GPIO port
 * Param	: Pointer to GPIO Handler
 * Return	: None
 * Note		: None
 *************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0U;
	uint8_t temp1 = 0U, temp2 = 0U, portcode = 0u;

	/*Enable GPIO peripheral clock*/
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	/*1. Configure pin mode type*/
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		/*Non interrupt mode*/
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2U * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x03U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); /*Clearing*/
		pGPIOHandle->pGPIOx->MODER |= temp;/*Setting*/
	}
	else
	{
		/*Interrupt mode*/
		if (GPIO_MODE_IN_FT == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
		{
			/*Configure the FTSR */
			EXTI->FTSR |= (1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/*Clear the corresponding RTSR*/
			EXTI->RTSR &= ~(1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (GPIO_MODE_IN_RT == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
		{
			/*Configure the RTSR */
			EXTI->RTSR |= (1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/*Clear the corresponding FTSR*/
			EXTI->FTSR &= ~(1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (GPIO_MODE_IN_RFT == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
		{
			/*Configure the FTSR */
			EXTI->FTSR |= (1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/*Configure the RTSR */
			EXTI->RTSR |= (1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else
		{}

		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4U);
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4U);
		portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4U);

		EXTI->IMR |= (1U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;
	/*2. Configure pin speed*/
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2U * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); /*Clearing*/
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;/*Setting*/

	temp = 0;
	/*3. Configure pupd*/
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPupPdControl << (2U * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); /*Clearing*/
	pGPIOHandle->pGPIOx->PUPDR |= temp;/*Setting*/

	temp = 0;
	/*4. Configure output type*/
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x03U << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); /*Clearing*/
	pGPIOHandle->pGPIOx->OTYPER |= temp;/*Setting*/
	/*5. Configure alternate function*/
	if (GPIO_MODE_ALTFN == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
	{
		temp1 = temp2 = 0U;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8U;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8U;
		pGPIOHandle->pGPIOx->AFR[temp1] &= (0xFU << (4U * temp2)); /*Clearing*/
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunction << (4U * temp2));/*Setting*/
	}
}

/***********************************Function Header***********************************
 * Function	: GPIO_DeInit
 * Brief	: This function de-initializes the GPIO port
 * Param	: Pointer to GPIO base address
 * Return	: None
 * Note		: None
 *************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (GPIOA == pGPIOx)
	{
		GPIOA_REG_RESET();
	}
	else if (GPIOB == pGPIOx)
	{
		GPIOB_REG_RESET();
	}
	else if (GPIOC == pGPIOx)
	{
		GPIOC_REG_RESET();
	}
	else if (GPIOD == pGPIOx)
	{
		GPIOD_REG_RESET();
	}
	else if (GPIOE == pGPIOx)
	{
		GPIOE_REG_RESET();
	}
	else if (GPIOF == pGPIOx)
	{
		GPIOF_REG_RESET();
	}
	else if (GPIOG == pGPIOx)
	{
		GPIOG_REG_RESET();
	}
	else if (GPIOH == pGPIOx)
	{
		GPIOH_REG_RESET();
	}
	else
	{}
}

/***********************************Function Header***********************************
 * Function	: GPIO_PeriClockControl
 * Brief	: This function enables or disables the peripheral clock for the GPIO port
 * Param1	: Pointer to GPIO base address
 * Param2	: Enable or Disable
 * Return	: None
 * Note		: None
 *************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (ENABLE == EnorDi)
	{
		if (GPIOA == pGPIOx)
		{
			GPIOA_PCLK_EN();
		}
		else if (GPIOB == pGPIOx)
		{
			GPIOB_PCLK_EN();
		}
		else if (GPIOC == pGPIOx)
		{
			GPIOC_PCLK_EN();
		}
		else if (GPIOD == pGPIOx)
		{
			GPIOD_PCLK_EN();
		}
		else if (GPIOE == pGPIOx)
		{
			GPIOE_PCLK_EN();
		}
		else if (GPIOF == pGPIOx)
		{
			GPIOF_PCLK_EN();
		}
		else if (GPIOG == pGPIOx)
		{
			GPIOG_PCLK_EN();
		}
		else if (GPIOH == pGPIOx)
		{
			GPIOH_PCLK_EN();
		}
		else
		{}
	}
	else
	{
		if (GPIOA == pGPIOx)
		{
			GPIOA_PCLK_DI();
		}
		else if (GPIOB == pGPIOx)
		{
			GPIOB_PCLK_DI();
		}
		else if (GPIOC == pGPIOx)
		{
			GPIOC_PCLK_DI();
		}
		else if (GPIOD == pGPIOx)
		{
			GPIOD_PCLK_DI();
		}
		else if (GPIOE == pGPIOx)
		{
			GPIOE_PCLK_DI();
		}
		else if (GPIOF == pGPIOx)
		{
			GPIOF_PCLK_DI();
		}
		else if (GPIOG == pGPIOx)
		{
			GPIOG_PCLK_DI();
		}
		else if (GPIOH == pGPIOx)
		{
			GPIOH_PCLK_DI();
		}
		else
		{}
	}
}

/***********************************Function Header***********************************
 * Function	: GPIO_ReadFromInputPin
 * Brief	: This function read form input pin
 * Param1	: Pointer to GPIO base address
 * Param2	: Pin Number
 * Return	: Pin value
 * Note		: None
 *************************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/***********************************Function Header***********************************
 * Function	: GPIO_ReadFromInputPort
 * Brief	: This function reads form input port
 * Param	: Pointer to GPIO base address
 * Return	: Port value
 * Note		: None
 *************************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) (pGPIOx->IDR);
	return value;
}

/***********************************Function Header***********************************
 * Function	: GPIO_WriteToOutputPin
 * Brief	: This function writes to output pin
 * Param1	: Pointer to GPIO base address
 * Param2	: Pin Number
 * Param3	: Value
 * Return	: None
 * Note		: None
 *************************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (GPIO_PIN_SET == Value)
	{
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}

/***********************************Function Header***********************************
 * Function	: GPIO_WriteToOutputPort
 * Brief	: This function writes to output port
 * Param1	: Pointer to GPIO base address
 * Param2	: Value
 * Return	: None
 * Note		: None
 *************************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/***********************************Function Header***********************************
 * Function	: GPIO_ToggleOutputPin
 * Brief	: This function toggles the GPIO pin
 * Param1	: Pointer to GPIO base address
 * Param2	: Pin number
 * Return	: None
 * Note		: None
 *************************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/***********************************Function Header***********************************
 * Function	: GPIO_IRQConfig
 * Brief	: This function configures the GPIO interrupts
 * Param1	: Interrupt number
 * Param2	: Interrupt priority
 * Param3	: Enable or Disable
 * Return	: None
 * Note		: None
 *************************************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (ENABLE == EnorDi)
	{
		if (IRQNumber <= 31U)
		{
			/*Program ISER0 register*/
			*NVIC_ISER0 = (1 << IRQNumber);
		}
		else if ((IRQNumber > 32U) && (IRQNumber < 64U))
		{
			/*Program ISER1 register*/
			*NVIC_ISER1 = (1 << (IRQNumber % 32U));
		}
		else if ((IRQNumber >= 64U) && (IRQNumber < 96U))
		{
			/*Program ISER2 register*/
			*NVIC_ISER2 = (1 << (IRQNumber % 64U));
		}
		else
		{}
	}
	else
	{
		if (IRQNumber <= 31U)
		{
			/*Program ISER0 register*/
			*NVIC_ICER0 = (1 << IRQNumber);
		}
		else if ((IRQNumber > 32U) && (IRQNumber < 64U))
		{
			/*Program ISER1 register*/
			*NVIC_ICER1 = (1 << (IRQNumber % 32U));
		}
		else if ((IRQNumber >= 64U) && (IRQNumber < 96U))
		{
			/*Program ISER2 register*/
			*NVIC_ICER2 = (1 << (IRQNumber % 64U));
		}
		else
		{}
	}
}

/***********************************Function Header***********************************
 * Function	: GPIO_IRQPriorityConfig
 * Brief	: This function configures the GPIO interrupts priority
 * Param1	: Interrupt number
 * Param2	: Interrupt priority
 * Return	: None
 * Note		: None
 *************************************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	/*Find ipr register*/
	uint8_t iprx = IRQNumber / 4U;
	uint8_t iprx_section = IRQNumber % 4U;
	uint8_t shift_amount = (8U * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (4U *iprx)) |= (IRQPriority << shift_amount);
}

/***********************************Function Header***********************************
 * Function	: GPIO_IRQHandling
 * Brief	: This function implements the interrupt handler
 * Param	: Pin Number
 * Return	: None
 * Note		: None
 *************************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	/*Clear the EXTI PR register corresponding to PinNumber*/
	if(EXTI->PR & (1 << PinNumber))
	{
		/*Clearing is done by writing 1*/
		EXTI->PR |= (1 << PinNumber);
	}
}

