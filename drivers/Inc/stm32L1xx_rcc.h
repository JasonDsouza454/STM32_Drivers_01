/*
 * stm32L1xx_rcc.h
 *
 *  Created on: Dec. 21, 2020
 *      Author: admin
 */

#ifndef INC_STM32L1XX_RCC_H_
#define INC_STM32L1XX_RCC_H_

#include "stm32L152xx.h"

/********************************************************************************************
 * 									RCC Driver API's
 * 									Function Prototypes
 ********************************************************************************************/

/*
 * Function	: This function returns that PClk1 Value
 * Param	: None
 * Return	: PClk1 value
 */
uint32_t RCC_GetPClk1Value(void);

/*
 * Function	: This function returns that PClk1 Value
 * Param	: None
 * Return	: None
 */
uint32_t RCC_GetPLLOutputClock();

#endif /* INC_STM32L1XX_RCC_H_ */
