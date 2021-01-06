/*
 * stm32L152xx.h
 *
 *  Created on: Sep 6, 2020
 *      Author: HJD
 */

#ifndef INC_STM32L152XX_H_
#define INC_STM32L152XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))

/**********************************PROCESSOR SPECIFIC DETAILS**********************************/
/*
 * ARM Cortex M3 Processor NVIC ISERx Register Address
 */

#define NVIC_ISER0				((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1				((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2				((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3				((__vo uint32_t *)0xE000E10C)

/*
 * ARM Cortex M3 Processor NVIC ICERx Register Address
 */

#define NVIC_ICER0				((__vo uint32_t *)0XE000E180)
#define NVIC_ICER1				((__vo uint32_t *)0XE000E184)
#define NVIC_ICER2				((__vo uint32_t *)0XE000E188)
#define NVIC_ICER3				((__vo uint32_t *)0XE000E18C)

#define NVIC_PR_BASE_ADDR		((__vo uint32_t *)0xE000E400)

/*
 * Arm Cortex Mx processor number of priority bits implemented in priority register
 */

#define NO_PR_BITS_IMPLEMENTED		4U
/**********************************************************************************************/
/*
 * base address of SRAM, Flash, ROM address
 */
#define FLASH_BASEADDR			0x08000000U /*Base address for Flash or program memory*/
#define ROM						0x1FF00000U /*Base address for ROM or system memory*/
#define SRAM					0x20000000U /*Base address for Embedded SRAM*/

/*
 * AHBx and APBx bus Peripheral base address
 */
#define PERIPH_BASEADDR			0x40000000U		/*Peripheral base address*/
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR /*APB1 peripheral base address*/
#define APB2PERIPH_BASEADDR		0x40010000U 	/*APB2 peripheral base address*/
#define AHBPERIPH_BASEADDR		0x40020000U 	/*AHB peripheral base address*/

/*
 * Base address of peripherals which are part of AHB bus
 */
#define GPIOA_BASEADDR			(AHBPERIPH_BASEADDR + 0x0000U) /*GPIO A Register base address*/
#define GPIOB_BASEADDR			(AHBPERIPH_BASEADDR + 0x0400U) /*GPIO B Register base address*/
#define GPIOC_BASEADDR			(AHBPERIPH_BASEADDR + 0x0800U) /*GPIO C Register base address*/
#define GPIOD_BASEADDR			(AHBPERIPH_BASEADDR + 0x0C00U) /*GPIO D Register base address*/
#define GPIOE_BASEADDR			(AHBPERIPH_BASEADDR + 0x1000U) /*GPIO E Register base address*/
#define GPIOF_BASEADDR			(AHBPERIPH_BASEADDR + 0x1800U) /*GPIO F Register base address*/
#define GPIOG_BASEADDR			(AHBPERIPH_BASEADDR + 0x1C00U) /*GPIO G Register base address*/
#define GPIOH_BASEADDR			(AHBPERIPH_BASEADDR + 0x1400U) /*GPIO H Register base address*/
#define CRC_BASEADDR			(AHBPERIPH_BASEADDR + 0x3000U) /*CRC Register base address*/
#define RCC_BASEADDR			(AHBPERIPH_BASEADDR + 0x3800U) /*RCC Register base address*/
#define EEPROM_BASEADDR			(AHBPERIPH_BASEADDR + 0x3C00U) /*Flash Register base address*/
#define DMA1_BASEADDR			(AHBPERIPH_BASEADDR + 0x6000U) /*DMA1 Register base address*/
#define DMA2_BASEADDR			(AHBPERIPH_BASEADDR + 0x6400U) /*DMA2 Register base address*/

/*
 * Base address of peripherals which are part of APB1 bus
 */
#define TIM2_BASEADDR			(APB1PERIPH_BASEADDR + 0x0000U) /*TIM 2 Register base address*/
#define TIM3_BASEADDR			(APB1PERIPH_BASEADDR + 0x0400U) /*TIM 3 Register base address*/
#define TIM4_BASEADDR			(APB1PERIPH_BASEADDR + 0x0800U) /*TIM 4 Register base address*/
#define TIM5_BASEADDR			(APB1PERIPH_BASEADDR + 0x0C00U) /*TIM 5 Register base address*/
#define TIM6_BASEADDR			(APB1PERIPH_BASEADDR + 0x1000U) /*TIM 6 Register base address*/
#define TIM7_BASEADDR			(APB1PERIPH_BASEADDR + 0x1400U) /*TIM 7 Register base address*/
#define LCD_BASEADDR			(APB1PERIPH_BASEADDR + 0x2400U) /*LCD Register base address*/
#define RTC_BASEADDR			(APB1PERIPH_BASEADDR + 0x2800U) /*RTC Register base address*/
#define WWDG_BASEADDR			(APB1PERIPH_BASEADDR + 0x2C00U) /*WWDG Register base address*/
#define IWDG_BASEADDR			(APB1PERIPH_BASEADDR + 0x3000U) /*IWDG Register base address*/
#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800U) /*SPI 2 Register base address*/
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00U) /*SPI 3 Register base address*/
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400U) /*USART 2 Register base address*/
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800U) /*USART 3 Register base address*/
#define USART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00U) /*USART 4 Register base address*/
#define USART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000U) /*USART 5 Register base address*/
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400U) /*I2C 1 Register base address*/
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800U) /*I2C 2 Register base address*/

/*
 * Base address of peripherals which are part of APB2 bus
 */
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x0000U) /*Sys Config Register base address*/
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x0400U) /*External interrupt EXTI Register base address*/
#define TIM9_BASEADDR			(APB2PERIPH_BASEADDR + 0x0800U) /*TIM 9 Register base address*/
#define TIM10_BASEADDR			(APB2PERIPH_BASEADDR + 0x0C00U) /*TIM 10 Register base address*/
#define TIM11_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000U) /*TIM 11 Register base address*/
#define ADC_BASEADDR			(APB2PERIPH_BASEADDR + 0x2400U) /*ADC Register base address*/
#define SDIO_BASEADDR			(APB2PERIPH_BASEADDR + 0x2C00U) /*SDIO Register base address*/
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000U) /*SPI 1 Register base address*/
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800U) /*USART 1 Register base address*/

/*******************************************Peripheral register definition structure*******************************************/
/*
 * Peripheral register definition for GPIO
 */
typedef struct
{
	__vo uint32_t MODER;		/*GPIO port mode register																		Address offset 0x00*/
	__vo uint32_t OTYPER;		/*GPIO port output type register																Address offset 0x04*/
	__vo uint32_t OSPEEDR;		/*GPIO port output speed register																Address offset 0x08*/
	__vo uint32_t PUPDR;		/*GPIO port pull-up/pull-down register															Address offset 0x0C*/
	__vo uint32_t IDR;			/*GPIO port input data register																	Address offset 0x10*/
	__vo uint32_t ODR;			/*GPIO port output data register																Address offset 0x14*/
	__vo uint32_t BSRR;			/*GPIO port bit set/reset register																Address offset 0x18*/
	__vo uint32_t LCKR;			/*GPIO port configuration lock register															Address offset 0x1C*/
	__vo uint32_t AFR[2];		/*AFR[0]: GPIO alternate function low register; AFR[1]: GPIO alternate function low register 	Address offset 0x20 & 0x24*/
}GPIO_RegDef_t;

/*
 * Peripheral definition for RCC
 */
typedef struct
{
	__vo uint32_t CR;			/*Clock control register*/
	__vo uint32_t ICSCR;		/*Internal clock sources calibration register*/
	__vo uint32_t CFGR;			/*Clock configuration register*/
	__vo uint32_t CIR;			/*Clock interrupt register*/
	__vo uint32_t AHBRSTR;		/*AHB peripheral reset register*/
	__vo uint32_t APB2RSTR;		/*APB2 peripheral reset register*/
	__vo uint32_t APB1RSTR;		/*APB1 peripheral reset register*/
	__vo uint32_t AHBENR;		/*AHB peripheral clock enable register*/
	__vo uint32_t APB2ENR;		/*APB2 peripheral clock enable register*/
	__vo uint32_t APB1ENR;		/*APB1 peripheral clock enable register*/
	__vo uint32_t AHBLPENR;		/*AHB peripheral clock enable in low-power mode register*/
	__vo uint32_t APB2LPENR;	/*APB2 peripheral clock enable in low-power mode register*/
	__vo uint32_t APB1LPENR;	/*APB1 peripheral clock enable in low-power mode register*/
	__vo uint32_t CSR;			/*Control/status register*/
}RCC_RegDef_t;

/*
 * Peripheral definition for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;			/*EXTI interrupt mask register*/
	__vo uint32_t EMR;			/*EXTI event mask register*/
	__vo uint32_t RTSR;			/*EXTI rising edge trigger selection register*/
	__vo uint32_t FTSR;			/*Falling edge trigger selection register*/
	__vo uint32_t SWIER;		/*EXTI software interrupt event register*/
	__vo uint32_t PR;			/*EXTI pending register*/
}EXTI_RegDef_t;

/*
 * Peripheral definition for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;		/*SYSCFG memory remap register*/
	__vo uint32_t PMC;			/*SYSCFG peripheral mode configuration register*/
	__vo uint32_t EXTICR[4];		/*SYSCFG external interrupt configuration register 1 - 4*/
}SYSCFG_RegDef_t;

/*
 * Peripheral definition for SPI - I2S
 */
typedef struct
{
	__vo uint32_t CR1;		/*SPI control register 1*/
	__vo uint32_t CR2;		/*SPI control register 2*/
	__vo uint32_t SR;		/*SPI status register*/
	__vo uint32_t DR;		/*SPI data register*/
	__vo uint32_t CRCPR;	/*SPI CRC polynomial register*/
	__vo uint32_t RXCRCR;	/*SPI RX CRC register*/
	__vo uint32_t TXCRCR;	/*SPI TX CRC register*/
	__vo uint32_t I2SCFG;	/*SPI_I2S configuration register*/
	__vo uint32_t I2SPR;	/*SPI_I2S prescaler register*/
}SPI_RegDef_t;

/*
 * Peripheral definition for I2C
 */
typedef struct
{
	__vo uint32_t CR1;		/*I2C control register 1*/
	__vo uint32_t CR2;		/*I2C control register 2*/
	__vo uint32_t OAR1;		/*I2C own address register*/
	__vo uint32_t OAR2;		/*I2C own address register*/
	__vo uint32_t DR;		/*I2C data register*/
	__vo uint32_t SR1;		/*I2C status register*/
	__vo uint32_t SR2;		/*I2C status register*/
	__vo uint32_t CCR;		/*I2C clock control register*/
	__vo uint32_t TRISE;	/*I2C Trise register*/
}I2C_RegDef_t;

/*
 * Peripheral definition (Peripheral base address type cast to xxx_RegDef_t
 */

#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t*)I2C2_BASEADDR)

/*
 * Clock enable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()		(RCC->AHBENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHBENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHBENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHBENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHBENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHBENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHBENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHBENR |= (1 << 7))

/*
 * Clock enable macros for I2C peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))

/*
 * Clock enable macros for SPI peripherals
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))

/*
 * Clock enable macros for USART peripherals
 */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define USART4_PCLK_EN()	(RCC->APB1ENR |= (1 << 19))
#define USART5_PCLK_EN()	(RCC->APB1ENR |= (1 << 20))

/*
 * Clock enable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 0))

/*
 * Clock disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		(RCC->AHBENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHBENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHBENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHBENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHBENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHBENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHBENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHBENR &= ~(1 << 7))

/*
 * Clock disable macros for I2C peripherals
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))

/*
 * Clock disable macros for SPI peripherals
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))

/*
 * Clock disable macros for USART peripherals
 */
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define USART4_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 19))
#define USART5_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 20))

/*
 * Clock disable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 0))

/*
 * Macros to disable GPIOx peripherals
 */
#define GPIOA_REG_RESET()		do{ (RCC->AHBRSTR |= (1 << 0)); (RCC->AHBRSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHBRSTR |= (1 << 1)); (RCC->AHBRSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHBRSTR |= (1 << 2)); (RCC->AHBRSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHBRSTR |= (1 << 3)); (RCC->AHBRSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHBRSTR |= (1 << 4)); (RCC->AHBRSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()		do{ (RCC->AHBRSTR |= (1 << 5)); (RCC->AHBRSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()		do{ (RCC->AHBRSTR |= (1 << 6)); (RCC->AHBRSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHBRSTR |= (1 << 7)); (RCC->AHBRSTR &= ~(1 << 7)); } while(0)

/*
 * Macros to disable SPIx peripherals
 */
#define SPI1_REG_RESET()		do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); } while(0)
#define SPI2_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); } while(0)
#define SPI3_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); } while(0)

/*
 * Macros to disable I2Cx peripherals
 */
#define I2C1_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); } while(0)
#define I2C2_REG_RESET()		do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); } while(0)

/*
 * IRQ (Interrupt Request) Numbers of STM32L15xRC MCU
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				46

/*
 * NVIC priority numbers
 */
#define NVIC_IRQ_PRI1			1
#define NVIC_IRQ_PRI2			2
#define NVIC_IRQ_PRI3			3
#define NVIC_IRQ_PRI4			4
#define NVIC_IRQ_PRI5			5
#define NVIC_IRQ_PRI6			6
#define NVIC_IRQ_PRI7			7
#define NVIC_IRQ_PRI8			8
#define NVIC_IRQ_PRI9			9
#define NVIC_IRQ_PRI10			10
#define NVIC_IRQ_PRI11			11
#define NVIC_IRQ_PRI12			12
#define NVIC_IRQ_PRI13			13
#define NVIC_IRQ_PRI14			14
#define NVIC_IRQ_PRI15			15

/********************************************************************************************
 * 					Bit position definition of SPI peripheral
 ********************************************************************************************/

/*
 * Bit position definition of SPI_CR1
 */
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

/*
 * Bit position definition of SPI_CR2
 */
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

/*
 * Bit position definition of SPI_SR
 */
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

/********************************************************************************************
 * 					Bit position definition of I2C peripheral
 ********************************************************************************************/
/*
 * Bit position definition of I2C_CR1
 */
#define I2C_CR1_PE				0
#define I2C_CR1_SMBUS			1
#define I2C_CR1_SMBYTE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15

/*
 * Bit position definition of I2C_CR2
 */
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12

/*
 * Bit position definition of I2C_SR1
 */
#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15

/*
 * Bit position definition of I2C_SR2
 */
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEFAULT		5
#define I2C_SR2_SMBHOST			6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC				8


/*
 * General macros
 */
#define SET						1U
#define RESET					0U
#define ENABLE					SET
#define DISABLE					RESET
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_SET				SET
#define FLAG_RESET				RESET



#endif /* INC_STM32L152XX_H_ */
