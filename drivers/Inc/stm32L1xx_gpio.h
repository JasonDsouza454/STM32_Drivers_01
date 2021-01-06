/*
 * stm32L1xx_gpio.h
 *
 *  Created on: Sep 11, 2020
 *      Author: admin
 */

#ifndef INC_STM32L1XX_GPIO_H_
#define INC_STM32L1XX_GPIO_H_

#include "stm32L152xx.h"

#define GPIO_BASEADDR_TO_CODE(xGPIO)	(	(xGPIO == GPIOA) ? 0 : \
											(xGPIO == GPIOB) ? 1 : \
											(xGPIO == GPIOC) ? 2 : \
											(xGPIO == GPIOD) ? 3 : \
											(xGPIO == GPIOE) ? 4 : \
											(xGPIO == GPIOF) ? 5 : \
											(xGPIO == GPIOG) ? 6 : \
											(xGPIO == GPIOH) ? 7 : 0)


/*
 * The GPIOx Pin configuration structure
 */
typedef struct
{
	uint8_t GPIO_PinNumber;			/*Possible values form @GPIN Pin number*/
	uint8_t GPIO_PinMode;			/*Possible values from @GPIO Pin Modes*/
	uint8_t GPIO_PinSpeed;			/*Possible values from @GPIO Pin Speed*/
	uint8_t GPIO_PinPupPdControl;	/*Possible values form @GPIO Pull up and pull down control*/
	uint8_t GPIO_PinOpType;			/*Possible values form @GPIO pin output types*/
	uint8_t GPIO_PinAltFunction;	/*Possible values form @GPIO Alternate functions*/
}GPIO_PinConfig_t;

/*
 * GPIO Handle structure
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;				/*Holds the base address of the GPIO port to which the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig;	/*Holds the pin configuration settings*/
}GPIO_Handle_t;

/*
 * @GPIO Pin Modes
 * Possible GPIO pin modes
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IN_FT		4
#define GPIO_MODE_IN_RT		5
#define GPIO_MODE_IN_RFT	6

/*
 * @GPIO pin output types
 * Possible GPIO pin output types
 */
#define GPIO_OP_TYPE_PP		0		/*Push pull*/
#define GPIO_OP_TYPE_OD		1		/*Open Drain*/

/*
 * @GPIO Pin Speed
 * Possible GPIO pin output speed
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO Pull up and pull down control
 * GPIO pull up, pull down configuration macros
 */
#define GPIO_PIN_NO_PUPD	0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/*
 * @GPIN Pin number
 * GPIO Pin numbers
 */
#define GPIO_PIN_NUM_0		0
#define GPIO_PIN_NUM_1		1
#define GPIO_PIN_NUM_2		2
#define GPIO_PIN_NUM_3		3
#define GPIO_PIN_NUM_4		4
#define GPIO_PIN_NUM_5		5
#define GPIO_PIN_NUM_6		6
#define GPIO_PIN_NUM_7		7
#define GPIO_PIN_NUM_8		8
#define GPIO_PIN_NUM_9		9
#define GPIO_PIN_NUM_10		10
#define GPIO_PIN_NUM_11		11
#define GPIO_PIN_NUM_12		12
#define GPIO_PIN_NUM_13		13
#define GPIO_PIN_NUM_14		14
#define GPIO_PIN_NUM_15		15


/********************************************************************************************
 * 									GPIO Driver API's
 * 									Function Prototypes
 ********************************************************************************************/
/*
 * Function	: To initialize GPIOx
 * Param	: Pointer to GPIO Handler
 * Return	: None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

/*
 * Function	: To de-initialize GPIOx
 * Param	: Pointer to GPIO base address
 * Return	: None
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Function	: To setup GPIOx peripheral clock
 * Param	: Pointer to GPIO base address, Enable or Disable
 * Return	: None
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Function	: To read form input pin
 * Param	: Pointer to GPIO base address, Pin Number
 * Return	: Pin value
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * Function	: To read form input port
 * Param	: Pointer to GPIO base address
 * Return	: Port value
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/*
 * Function	: To write to output pin
 * Param	: Pointer to GPIO base address, Pin Number, Value
 * Return	: None
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);

/*
 * Function	: To write to output port
 * Param	: Pointer to GPIO base address, Value
 * Return	: None
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);

/*
 * Function	: To toggle output pin
 * Param	: Pointer to GPIO base address, Pin number
 * Return	: None
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * Function	: To configure interrupt
 * Param	: Interrupt number, Enable or Disable
 * Return	: None
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

/*
 * Function	: Interrupt handler
 * Param	: Pin Number
 * Return	: None
 */
void GPIO_IRQHandling(uint8_t PinNumber);

/*
 * Function	: To configure interrupt priority
 * Param	: Interrupt number, Interrupt priority
 * Return	: None
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);




#endif /* INC_STM32L1XX_GPIO_H_ */
