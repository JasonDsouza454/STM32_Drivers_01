/*
 * stm32L1xx_rcc.c
 *
 *  Created on: Dec. 21, 2020
 *      Author: admin
 */

#include "stm32L1xx_rcc.h"

/********************************************************************************************
 * 									RCC Driver API's
 * 									Function Implementation
 ********************************************************************************************/

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScaler[4] = {2,4,8,16};

/***********************************Function Header***********************************
 * Function	: RCC_GetPLLOutputClock
 * Brief	: This function initializes the I2Cx
 * Param	: None
 * Return	: None
 * Note		: None
 *************************************************************************************/
uint32_t RCC_GetPLLOutputClock()
{
	return 1;
}
/***********************************Function Header***********************************
 * Function	: RCC_GetPClk1Value
 * Brief	: This function returns that PClk1 Value
 * Param	: None
 * Return	: PClk1 value
 * Note		: None
 *************************************************************************************/

uint32_t RCC_GetPClk1Value(void)
{
	uint32_t PClk1, SystemClk;
	uint8_t ClkSrc, Temp, Ahbp, Apb1;

	ClkSrc = (RCC->CFGR >> 2) & 0x3;

	if (0 == ClkSrc)
	{
		SystemClk = 16000000;
	}
	else if (1 == ClkSrc)
	{
		SystemClk = 8000000;
	}
	else if (2 == ClkSrc)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	Temp = ((RCC->CFGR >> 4) & 0xF);
	if(Temp < 8)
	{
		Ahbp = AHB_PreScaler[Temp - 8];
	}
	Temp = ((RCC->CFGR >> 8) & 0x7);
	if(Temp < 4)
	{
		Apb1 = APB1_PreScaler[Temp - 4];
	}
	PClk1 = (SystemClk / Ahbp) / Apb1;
	return PClk1;
}

