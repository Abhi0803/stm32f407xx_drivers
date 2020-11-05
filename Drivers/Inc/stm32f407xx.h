/*
 * stm32f407xx.h
 *
 *  Created on: Sep 24, 2020
 *      Author: Abhinav
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stddef.h>
#include <stdint.h>
#include  <string.h>
#define __weak __attribute__((weak))
#define __vo volatile
// DELAY MAX TIME
#define MAX								(1250)						// 1ms = 1250 => 1000ms = 1250000

/*****************************************************************PROCESSOR SPECIFIC DETAILS********************************************/
/*
 * ARM Cotex - M4 Proccesor NVIC ISERx register Addresses
 */

#define NVIC_ISER0 						((__vo uint32_t* )(0xE000E100))
#define NVIC_ISER1 						((__vo uint32_t* )(0xE000E104))
#define NVIC_ISER2 						((__vo uint32_t* )(0xE000E108))
#define NVIC_ISER3 						((__vo uint32_t* )(0xE000E10C))
#define NVIC_ISER4 						((__vo uint32_t* )(0xE000E110))
#define NVIC_ISER5 						((__vo uint32_t* )(0xE000E114))
#define NVIC_ISER6 						((__vo uint32_t* )(0xE000E118))
#define NVIC_ISER7 						((__vo uint32_t* )(0xE000E11C))

/*
 * ARM Cotex - M4 Proccesor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 						((__vo uint32_t* )(0xE000E180))
#define NVIC_ICER1 						((__vo uint32_t* )(0xE000E184))
#define NVIC_ICER2 						((__vo uint32_t* )(0xE000E188))
#define NVIC_ICER3 						((__vo uint32_t* )(0xE000E18C))
#define NVIC_ICER4 						((__vo uint32_t* )(0xE000E190))
#define NVIC_ICER5 						((__vo uint32_t* )(0xE000E194))
#define NVIC_ICER6 						((__vo uint32_t* )(0xE000E198))
#define NVIC_ICER7 						((__vo uint32_t* )(0xE000E19C))

/*
 * ARM Cotex - M4 Proccesor NVIC Interrupt Priority register Base Addresses
 * As there are 240 Interrupt Priority that can be assigned and to store these
 * this processor has 60 32-bit Register each storing 4 Priority of 8-bit each
 * this is the base address of the IPR and by derefrencing it we can get to any
 * one of the 60 Registers.
 */
#define NVIC_IPR_BASEADDR 				((__vo uint32_t* )(0xE000E400))

/*
 * ARM Cotex - M4 Proccesor NVIC Interrupt Priority register Bit Implemented
 * As Mentioned Above that Each Register of IPR has 4 Priority of 8-bit each
 * but only the MSB bits are available the LSB bits are not available
 * so we need to shift the IRQ PRIORITY BY 4-Bit.
 * MSB-FIRST 4-bit & LSB-LAST 4-bit of the Priority 0xMMMM LLLL
 */
#define PR_BITS_IMPLEMENTED				(4)


/* ****************************************************** MCU SPECIFIC DETAILS ***************************************************/


// BASE ADDRESSES OF MAJOR STORAGE REGISTERS
#define FLASH_BASEADDR					(0x08000000U)
#define SRAM1_BASEADDR					(0x20000000U)
#define SRAM2_BASEADDR					(0x2001C000U)
#define ROM_BASEADDR					(0x1FFF0000U)

// ADDRESS OF ROM SRAM1 SRAM2
#define ROM								(ROM_BASEADDR)
#define SRAM1 							(SRAM1_BASEADDR)
#define SRAM2 							(SRAM2_BASEADDR)

// BASE ADDRESS OF PERIPHERAL REGISTER
#define PERIPH_BASEADDR					(0x40000000U)

// ADDRESS OF ABHx APBx BUS PERIPHERAL BASE REGISTERS
#define APB1PERIPH_BASEADDR				(PERIPH_BASEADDR )
#define APB2PERIPH_BASEADDR				(0x40010000U)
#define AHB1PERIPH_BASEADDR				(0x40020000U)
#define AHB2PERIPH_BASEADDR				(0x50000000U)

// BASE ADDRESS OF PERIPHERALS ON AHB1
#define GPIOA_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR 					(AHB1PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3800)

// BASE ADDRESS OF PERIPHERALS ON APB1
#define I2C1_BASEADDR 					(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR 					(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR 					(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR 					(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR 					(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR 				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR 				(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR 					(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR 					(APB1PERIPH_BASEADDR + 0x5000)

#define CAN1_BASEADDR 					(APB1PERIPH_BASEADDR + 0x6400)
#define CAN2_BASEADDR 					(APB1PERIPH_BASEADDR + 0x6800)

// BASE ADDRESS OF PERIPHERALS ON APB2
#define SPI1_BASEADDR 					(APB2PERIPH_BASEADDR + 0x3000)

#define USART1_BASEADDR 				(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR 				(APB2PERIPH_BASEADDR + 0x1400)

#define EXTI_BASEADDR	 				(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR 				(APB2PERIPH_BASEADDR + 0x3800)


// PERIPHERAL REGISTER DEFINITION STRUCTURE FOR RCC
typedef struct{
	__vo uint32_t RCC_CR;						// RCC Clock Control Register													(OFFSET 0x00)
	__vo uint32_t RCC_PLLCFGR;					// RCC PLL Configuration Register												(OFFSET 0x04)
	__vo uint32_t RCC_CFGR;						// RCC Clock Configuration Register												(OFFSET 0x08)
	__vo uint32_t RCC_CIR;						// RCC Clock Interrupt Register													(OFFSET 0x0C)
	__vo uint32_t RCC_AHB1RSTR;					// RCC AHB1 peripheral reset Register											(OFFSET 0x10)
	__vo uint32_t RCC_AHB2RSTR;					// RCC AHB2 peripheral reset Register											(OFFSET 0x14)
	__vo uint32_t RCC_AHB3RSTR;					// RCC AHB3 peripheral reset Register											(OFFSET 0x18)
	__vo uint32_t RCC_RESERVED1;				// RCC Clock Control RESERVED Register											(OFFSET 0x1C)
	__vo uint32_t RCC_APB1RSTR;					// RCC APB1 peripheral reset Register											(OFFSET 0x20)
	__vo uint32_t RCC_APB2RSTR;					// RCC APB2 peripheral reset Register											(OFFSET 0x24)
	__vo uint32_t RCC_RESERVED2;				// RCC Clock Control RESERVED Register											(OFFSET 0x28)
	__vo uint32_t RCC_RESERVED3;				// RCC Clock Control RESERVED Register											(OFFSET 0x2C)
	__vo uint32_t RCC_AHB1ENR;					// RCC AHB1 peripheral clock enable Register									(OFFSET 0x30)
	__vo uint32_t RCC_AHB2ENR;					// RCC AHB2 peripheral clock enable Register									(OFFSET 0x34)
	__vo uint32_t RCC_AHB3ENR;					// RCC AHB3 peripheral clock enable Register									(OFFSET 0x38)
	__vo uint32_t RCC_RESERVED4;				// RCC Clock Control RESERVED Register											(OFFSET 0x3C)
	__vo uint32_t RCC_APB1ENR;					// RCC APB1 peripheral clock enable Register									(OFFSET 0x40)
	__vo uint32_t RCC_APB2ENR;					// RCC APB2 peripheral clock enable Register									(OFFSET 0x44)
	__vo uint32_t RCC_RESERVED5;				// RCC Clock Control RESERVED Register											(OFFSET 0x48)
	__vo uint32_t RCC_RESERVED6;				// RCC Clock Control RESERVED Register											(OFFSET 0x4C)
	__vo uint32_t RCC_AHB1LPENR;				// RCC AHB1 peripheral clock enable in low power mode Register					(OFFSET 0x50)
	__vo uint32_t RCC_AHB2LPENR;				// RCC AHB2 peripheral clock enable in low power mode Register					(OFFSET 0x54)
	__vo uint32_t RCC_AHB3LPENR;				// RCC AHB3 peripheral clock enable in low power mode  Register					(OFFSET 0x58)
	__vo uint32_t RCC_RESERVED7;				// RCC Clock Control RESERVED Register											(OFFSET 0x5C)
	__vo uint32_t RCC_APB1LPENR;				// RCC APB1 peripheral clock enable in low power mode Register					(OFFSET 0x60)
	__vo uint32_t RCC_APB2LPENR;				// RCC APB2 peripheral clock enabled in low power mode Register					(OFFSET 0x64)
	__vo uint32_t RCC_RESERVED8;				// RCC Clock Control RESERVED Register											(OFFSET 0x68)
	__vo uint32_t RCC_RESERVED9;				// RCC Clock Control RESERVED Register											(OFFSET 0x6C)
	__vo uint32_t RCC_BDCR;						// RCC Backup domain control Register											(OFFSET 0x70)
	__vo uint32_t RCC_CSR;						// RCC clock control & status Register											(OFFSET 0x74)
	__vo uint32_t RCC_RESERVED10;				// RCC Clock Control RESERVED Register											(OFFSET 0x78)
	__vo uint32_t RCC_RESERVED11;				// RCC Clock Control RESERVED Register											(OFFSET 0x7C)
	__vo uint32_t RCC_SSCGR;					// RCC spread spectrum clock generation Register								(OFFSET 0x80)
	__vo uint32_t RCC_PLLI2SCFGR;				// RCC PLLI2S configuration Register											(OFFSET 0x84)
}RCC_RegDef_t;


// PERIPHERAL REGISTER DEFINITION STRUCTURE FOR EXTI
typedef struct{
	__vo uint32_t EXTI_IMR;						// EXTI Interrupt mask register 												(OFFSET 0x00)
	__vo uint32_t EXTI_EMR;						// EXTI Event mask register 													(OFFSET 0x04)
	__vo uint32_t EXTI_RTSR;					// EXTI Rising trigger selection register										(OFFSET 0x08)
	__vo uint32_t EXTI_FTSR; 					// EXTI Falling trigger selection register										(OFFSET 0x0C)
	__vo uint32_t EXTI_SWIER; 					// EXTI Software interrupt event register										(OFFSET 0x10)
	__vo uint32_t EXTI_PR; 						// EXTI Pending register 														(OFFSET 0x14)
}EXTI_RegDef_t;

// PERIPHERAL REGISTER DEFINITION STRUCTURE FOR GPIO
typedef struct{
	__vo uint32_t GPIO_MODER;					// GPIO port mode register														(OFFSET 0x00)
	__vo uint32_t GPIO_OTYPER;					// GPIO port output type register												(OFFSET 0x04)
	__vo uint32_t GPIO_OSPEEDR;					// GPIO port output speed register 												(OFFSET 0x08)
	__vo uint32_t GPIO_PUPDR;					// GPIO port pull-up/pull-down register											(OFFSET 0x0C)
	__vo uint32_t GPIO_IDR;						// GPIO port input data register												(OFFSET 0x10)
	__vo uint32_t GPIO_ODR;						// GPIO port output data register												(OFFSET 0x14)
	__vo uint32_t GPIO_BSRR;					// GPIO port bit set/reset register												(OFFSET 0x18)
	__vo uint32_t GPIO_LCKR;					// GPIO port configuration lock register										(OFFSET 0x1C)
	__vo uint32_t AFR[2];					 	// GPIO port alternate function high register    								(OFFSET 0x24)

}GPIO_RegDef_t;


// PERIPHERAL REGISTER DEFINITION STRUCTURE FOR SPI
typedef struct{
	__vo uint32_t SPI_CR1;						// SPI control register 1														(OFFSET 0x00)
	__vo uint32_t SPI_CR2;						// SPI control register 2														(OFFSET 0x04)
	__vo uint32_t SPI_SR;						// SPI status register  														(OFFSET 0x08)
	__vo uint32_t SPI_DR;						// SPI data register															(OFFSET 0x0C)
	__vo uint32_t SPI_CRCPR;					// SPI CRC polynomial register													(OFFSET 0x10)
	__vo uint32_t SPI_RXCRCR;					// SPI RX CRC register															(OFFSET 0x14)
	__vo uint32_t SPI_TXCRCR;					// SPI TX CRC register															(OFFSET 0x18)
	__vo uint32_t SPI_I2SCFGR;					// SPI_I2S configuration register												(OFFSET 0x1C)
	__vo uint32_t SPI_I2SPR;					// SPI_I2S prescaler register													(OFFSET 0x20)

}SPI_RegDef_t;


// PERIPHERAL REGISTER DEFINITION STRUCTURE FOR SYSCFG
typedef struct{
	__vo uint32_t SYSCFG_MEMRMP;				// SYSCFG memory remap register													(OFFSET 0x00)
	__vo uint32_t SYSCFG_PMC;					// SYSCFG peripheral mode configuration register								(OFFSET 0x04)
	__vo uint32_t SYSCFG_EXTICR[4];				// SYSCFG external interrupt configuration register 1							(OFFSET 0x08)
	__vo uint32_t SYSCFG_RESERVED[2];			// SYSCFG RESERVED Register														(OFFSET 0x18)
	__vo uint32_t SYSCFG_CMPCR;					// SYSCFG Compensation cell control register									(OFFSET 0x20)
}SYSCFG_RegDef_t;

// PERIPHERAL REGISTER DEFINITION STRUCTURE FOR I2C
typedef struct{
	__vo uint32_t I2C_CR1;						// I2C Control register 1														(OFFSET 0x00)
	__vo uint32_t I2C_CR2;						// I2C Control register 2														(OFFSET 0x04)
	__vo uint32_t I2C_OAR1;						// I2C Own address register 1													(OFFSET 0x08)
	__vo uint32_t I2C_OAR2;						// I2C Own address register 2													(OFFSET 0x0C)
	__vo uint32_t I2C_DR;						// I2C Data register															(OFFSET 0x10)
	__vo uint32_t I2C_SR1;						// I2C Status register 1														(OFFSET 0x14)
	__vo uint32_t I2C_SR2;						// I2C Status register 2														(OFFSET 0x18)
	__vo uint32_t I2C_CCR;						// I2C Clock control register													(OFFSET 0x1C)
	__vo uint32_t I2C_TRISE;					// I2C TRISE register															(OFFSET 0x20)
	__vo uint32_t I2C_FLTR;						// I2C FLTR register															(OFFSET 0x24)
}I2C_RegDef_t;

// PERIPHERAL REGISTER DEFINITION STRUCTURE FOR USART
typedef struct{
	__vo uint32_t USART_SR;						// USART Status register 														(OFFSET 0x00)
	__vo uint32_t USART_DR;							// USART Data register															(OFFSET 0x04)
	__vo uint32_t USART_BRR;						// USART BAUD RATE register 													(OFFSET 0x08)
	__vo uint32_t USART_CR1;						// USART Control register 1														(OFFSET 0x0C)
	__vo uint32_t USART_CR2;						// USART Control register 2														(OFFSET 0x10)
	__vo uint32_t USART_CR3;						// USART Control register 3														(OFFSET 0x14)
	__vo uint32_t USART_GPTR;						// USART Guard time and prescaler register										(OFFSET 0x18)

}USART_RegDef_t;




// PERIPHERAL DEFINITIONS MACROS ( PERIPHERAL BASE ADDRESS TYPCASTING TO xxx_RegDef )

#define GPIOA 							((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 							((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 							((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 							((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 							((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 							((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 							((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 							((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 							((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC 							((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI							((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG							((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1							((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2							((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3							((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1							((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2							((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3							((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1							((USART_RegDef_t*)USART1_BASEADDR)
#define USART2							((USART_RegDef_t*)USART2_BASEADDR)
#define USART3							((USART_RegDef_t*)USART3_BASEADDR)
#define UART4							((USART_RegDef_t*)UART4_BASEADDR)
#define UART5							((USART_RegDef_t*)UART5_BASEADDR)
#define USART6							((USART_RegDef_t*)USART6_BASEADDR)
/* CLOCK ENABLE */

// CLOCK ENABLE MACROS for GPIOx PERIPHERALS
#define GPIOA_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()					(RCC->RCC_AHB1ENR |= (1 << 8))

// CLOCK ENABLE MACROS for I2Cx PERIPHERALS
#define I2C1_PCLK_EN()					(RCC->RCC_APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()					(RCC->RCC_APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()					(RCC->RCC_APB1ENR |= (1 << 23))

// CLOCK ENABLE MACROS for SPIx PERIPHERALS
#define SPI1_PCLK_EN()					(RCC->RCC_APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()					(RCC->RCC_APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()					(RCC->RCC_APB1ENR |= (1 << 15))

// CLOCK ENABLE MACROS for USARTx PERIPHERALS
#define USART1_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 18))
#define USART6_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 5))
// CLOCK ENABLE MACROS for UARTx PERIPHERALS
#define UART4_PCLK_EN()					(RCC->RCC_APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()					(RCC->RCC_APB1ENR |= (1 << 20))

// CLOCK ENABLE MACROS for CANx PERIPHERALS
#define CAN1_PCLK_EN()					(RCC->RCC_APB1ENR |= (1 << 25))
#define CAN2_PCLK_EN()					(RCC->RCC_APB1ENR |= (1 << 26))

// CLOCK ENABLE MACROS for SYSCFG PERIPHERALS
#define SYSCFG_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 14))

/* CLOCK DISABLE */

// CLOCK DISABLE MACROS for GPIOx PERIPHERALS
#define GPIOA_PCLK_DI()					(RCC->RCC_AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()					(RCC->RCC_AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()					(RCC->RCC_AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()					(RCC->RCC_AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()					(RCC->RCC_AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()					(RCC->RCC_AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()					(RCC->RCC_AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()					(RCC->RCC_AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()					(RCC->RCC_AHB1ENR &= ~(1 << 8))

// CLOCK DISABLE MACROS for I2Cx PERIPHERALS
#define I2C1_PCLK_DI()					(RCC->RCC_APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()					(RCC->RCC_APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()					(RCC->RCC_APB1ENR &= ~(1 << 23))

// CLOCK DISABLE MACROS for SPIx PERIPHERALS
#define SPI1_PCLK_DI()					(RCC->RCC_APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()					(RCC->RCC_APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()					(RCC->RCC_APB1ENR &= ~(1 << 15))

// CLOCK DISABLE MACROS for USARTx PERIPHERALS
#define USART1_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 18))
#define USART6_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 5))
// CLOCK DISABLE MACROS for UARTx PERIPHERALS
#define UART4_PCLK_DI()					(RCC->RCC_APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()					(RCC->RCC_APB1ENR &= ~(1 << 20))

// CLOCK DISABLE MACROS for CANx PERIPHERALS
#define CAN1_PCLK_DI()					(RCC->RCC_APB1ENR &= ~(1 << 25))
#define CAN2_PCLK_DI()					(RCC->RCC_APB1ENR &= ~(1 << 26))

// CLOCK DISABLE MACROS for SYSCFG PERIPHERALS
#define SYSCFG_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 14))

// Some Generic Macro
#define ENABLE 							(1)
#define DISABLE 						(0)
#define SET 							(ENABLE)
#define RESET 							(DISABLE)
#define GPIO_PIN_SET 					(SET)
#define GPIO_PIN_RESET 					(RESET)
#define FLAG_RESET						(RESET)
#define FLAG_SET						(SET)

/************************************************************************************************************************************************************************
 * 																					BIT POSITION DEFINITIONS OF PERIPHERAL REGISTERS
 * 																		For More Details Please Check the definitions of the particular REGISTERS
 **************************************************************************************************************************************************************************/
// SPI_CR1
#define SPI_CR1_CPHA					(0)
#define SPI_CR1_CPOL					(1)
#define SPI_CR1_MSTR					(2)
#define SPI_CR1_SPEED					(3)
#define SPI_CR1_SPI_EN					(6)
#define SPI_CR1_LSB_FIRST				(7)
#define SPI_CR1_SSI						(8)
#define SPI_CR1_SSM						(9)
#define SPI_CR1_RX_ONLY					(10)
#define SPI_CR1_DFF						(11)
#define SPI_CR1_CRC_NEXT				(12)
#define SPI_CR1_CRC_EN					(13)
#define SPI_CR1_BIDIOE					(14)
#define SPI_CR1_BIDIMODE				(15)

// SPI_CR2
#define SPI_CR2_RXDMAEN					(0)
#define SPI_CR2_TXDMAEN					(1)
#define SPI_CR2_SSOE					(2)

#define SPI_CR2_FRF						(4)
#define SPI_CR2_ERRIE					(5)
#define SPI_CR2_RXNEIE					(6)
#define SPI_CR2_TXEIE					(7)

// SPI_SR
#define SPI_SR_RXNE						(0)
#define SPI_SR_TXE						(1)
#define SPI_SR_CHSIDE					(2)
#define SPI_SR_UDR						(3)
#define SPI_SR_CRC_ERR					(4)
#define SPI_SR_MODF						(5)
#define SPI_SR_OVR						(6)
#define SPI_SR_BSY						(7)
#define SPI_SR_FRE						(8)



// I2C_CR1
#define I2C_CR1_I2C_EN					(0)
#define I2C_CR1_SMBUS					(1)
#define I2C_CR1_SMBTYPE					(3)
#define I2C_CR1_ENARP					(4)
#define I2C_CR1_ENPEC					(5)
#define I2C_CR1_ENGC					(6)
#define I2C_CR1_NOSTRETCH				(7)
#define I2C_CR1_START					(8)
#define I2C_CR1_STOP					(9)
#define I2C_CR1_ACK						(10)
#define I2C_CR1_POS						(11)
#define I2C_CR1_PEC						(12)
#define I2C_CR1_ALERT					(13)
#define I2C_CR1_SWRST					(15)

// I2C_CR2
#define I2C_CR2_FREQ					(0)
#define I2C_CR2_ITERREN					(8)
#define I2C_CR2_ITEVTEN					(9)
#define I2C_CR2_ITBUFEN					(10)
#define I2C_CR2_DMAEN					(11)
#define I2C_CR2_LAST					(12)

// I2C_OAR1
#define I2C_OAR1_ADD0					(0)
#define I2C_OAR1_ADD_7_1				(1)
#define I2C_OAR1_ADD_9_8				(8)
#define I2C_OAR1_BIT14					(14)
#define I2C_OAR1_ADDMODE				(15)

// I2C_OAR2
#define I2C_OAR2_ENDUAL					(0)
#define I2C_OAR2_ADD2_7_1				(1)

// I2C_DR
#define I2C_DR_DR_7_0					(0)

// I2C_SR1
#define I2C_SR1_SB						(0)
#define I2C_SR1_ADDR					(1)
#define I2C_SR1_BTF						(2)
#define I2C_SR1_ADD10					(3)
#define I2C_SR1_STOPF					(4)
#define I2C_SR1_RxNE					(6)
#define I2C_SR1_TxE						(7)
#define I2C_SR1_BERR					(8)
#define I2C_SR1_ARLO					(9)
#define I2C_SR1_AF						(10)
#define I2C_SR1_OVR						(11)
#define I2C_SR1_PECERR					(12)
#define I2C_SR1_TIMEOUT					(14)
#define I2C_SR1_SMBALERT				(15)

// I2C_SR2
#define I2C_SR2_MSL						(0)
#define I2C_SR2_BUSY					(1)
#define I2C_SR2_TRA						(2)
#define I2C_SR2_GENCALL					(4)
#define I2C_SR2_SMBDEFAULT				(5)
#define I2C_SR2_SMBHOST					(6)
#define I2C_SR2_DUALF					(7)
#define I2C_SR2_PEC_15_8 				(8)

// I2C_CCR
#define I2C_CCR_CCR_11_0				(0)
#define I2C_CCR_DUTY					(14)
#define I2C_CCR_F_S						(15)

// I2C_TRISE
#define I2C_TRISE_TRISE_5_0				(0)

// I2C_FLTR
#define I2C_FLTR_DNF_3_0				(0)
#define I2C_FLTR_ANOFF					(4)


// USART_SR
#define USART_SR_PE						(0)
#define USART_SR_FE						(1)
#define USART_SR_NF						(2)
#define USART_SR_ORE					(3)
#define USART_SR_IDLE					(4)
#define USART_SR_RXNE					(5)
#define USART_SR_TC						(6)
#define USART_SR_TXE					(7)
#define USART_SR_LBD					(8)
#define USART_SR_CTS					(9)

// USART_DR
#define USART_DR_DR						(0)

// USART_BRR
#define USART_BRR_DIV_Fraction			(0)
#define USART_BRR_DIV_Mantissa			(1)

// USART_CR1
#define USART_CR1_SBK					(0)
#define USART_CR1_RWU					(1)
#define USART_CR1_RE					(2)
#define USART_CR1_TE					(3)
#define USART_CR1_IDLEIE				(4)
#define USART_CR1_RXNEIE				(5)
#define USART_CR1_TCIE					(6)
#define USART_CR1_TXEIE					(7)
#define USART_CR1_PEIE					(8)
#define USART_CR1_PS					(9)
#define USART_CR1_PCE					(10)
#define USART_CR1_WAKE					(11)
#define USART_CR1_M						(12)
#define USART_CR1_UE					(13)
#define USART_CR1_Reserved				(14)
#define USART_CR1_OVER8					(15)

// USART_CR2
#define USART_CR2_ADD					(0)
#define USART_CR2_LBDL					(5)
#define USART_CR2_LBDIE					(6)
#define USART_CR2_LBCL					(8)
#define USART_CR2_CPHA					(9)
#define USART_CR2_CPOL					(10)
#define USART_CR2_CLKEN					(11)
#define USART_CR2_STOP					(12)
#define USART_CR2_LINEN					(14)

// USART_CR3
#define USART_CR3_EIE					(0)
#define USART_CR3_IREN					(1)
#define USART_CR3_IRLP					(2)
#define USART_CR3_HDSEL					(3)
#define USART_CR3_NACK					(4)
#define USART_CR3_SCEN					(5)
#define USART_CR3_DMAR					(6)
#define USART_CR3_DMAT					(7)
#define USART_CR3_RTSE					(8)
#define USART_CR3_CTSE					(9)
#define USART_CR3_CTSIE					(10)
#define USART_CR3_ONEBIT				(11)

// USART_GTPR
#define USART_GTPR_PSC					(0)
#define USART_GTPR_GT					(8)

/************************************************************************************************************************************************************************
 * 																		RESET MACROS DEFINITIONS OF PERIPHERAL REGISTERS
 * 															For More Details Please Check the definitions of the particular REGISTERS
 **************************************************************************************************************************************************************************/
// GPIOx RESET MACROS
#define GPIOA_REG_RESET()				do{ (RCC->RCC_AHB1RSTR |= (1 << 0)) ;	(RCC->RCC_AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()				do{ (RCC->RCC_AHB1RSTR |= (1 << 1)) ;	(RCC->RCC_AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()				do{ (RCC->RCC_AHB1RSTR |= (1 << 2)) ;	(RCC->RCC_AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()				do{ (RCC->RCC_AHB1RSTR |= (1 << 3)) ;	(RCC->RCC_AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()				do{ (RCC->RCC_AHB1RSTR |= (1 << 4)) ;	(RCC->RCC_AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()				do{ (RCC->RCC_AHB1RSTR |= (1 << 5)) ;	(RCC->RCC_AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()				do{ (RCC->RCC_AHB1RSTR |= (1 << 6)) ;	(RCC->RCC_AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()				do{ (RCC->RCC_AHB1RSTR |= (1 << 7)) ;	(RCC->RCC_AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()				do{ (RCC->RCC_AHB1RSTR |= (1 << 8)) ;	(RCC->RCC_AHB1RSTR &= ~(1 << 8)); }while(0)

// SPI RESET MACROS
#define SPI1_REG_RESET()				do{ (RCC->RCC_APB2RSTR |= (1 << 12)) ;	(RCC->RCC_APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()				do{ (RCC->RCC_APB1RSTR |= (1 << 14)) ;	(RCC->RCC_APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()				do{ (RCC->RCC_APB1RSTR |= (1 << 15)) ;	(RCC->RCC_APB1RSTR &= ~(1 << 15)); }while(0)

// I2C RESET MACROS
#define I2C1_REG_RESET()				do{ (RCC->RCC_APB1RSTR |= (1 << 21)) ;	(RCC->RCC_APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()				do{ (RCC->RCC_APB1RSTR |= (1 << 22)) ;	(RCC->RCC_APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()				do{ (RCC->RCC_APB1RSTR |= (1 << 23)) ;	(RCC->RCC_APB1RSTR &= ~(1 << 23)); }while(0)

// USART RESET MACROS
#define USART1_REG_RESET()				do{ (RCC->RCC_APB2RSTR |= (1 << 4)) ;	(RCC->RCC_APB2RSTR &= ~(1 << 4)); }while(0)
#define USART2_REG_RESET()				do{ (RCC->RCC_APB1RSTR |= (1 << 17)) ;	(RCC->RCC_APB1RSTR &= ~(1 << 22)); }while(0)
#define USART3_REG_RESET()				do{ (RCC->RCC_APB1RSTR |= (1 << 18)) ;	(RCC->RCC_APB1RSTR &= ~(1 << 23)); }while(0)
#define UART4_REG_RESET()				do{ (RCC->RCC_APB1RSTR |= (1 << 19)) ;	(RCC->RCC_APB1RSTR &= ~(1 << 22)); }while(0)
#define UART5_REG_RESET()				do{ (RCC->RCC_APB1RSTR |= (1 << 20)) ;	(RCC->RCC_APB1RSTR &= ~(1 << 23)); }while(0)
#define USART6_REG_RESET()				do{ (RCC->RCC_APB2RSTR |= (1 << 5)) ;	(RCC->RCC_APB2RSTR &= ~(1 << 5)); }while(0)

#define Delay_ms(miliseconds)					for (uint32_t var = 0; var < (miliseconds * MAX); ++var)

#define GPIO_BASEADDR_TO_CODE(x)		(x == GPIOA) ? 0 : (x == GPIOB) ? 1 : (x == GPIOC) ? 2 : (x == GPIOD) ? 3 : (x == GPIOE) ? 4 : (x == GPIOF) ? 5 : (x == GPIOG) ? 6 : (x == GPIOH) ? 7 : 8

// IRQ Numbers Macros
#define IRQ_NO_WWDG						(0)
#define IRQ_NO_PVD						(1)
#define IRQ_NO_TAMP_STAMP				(2)
#define IRQ_NO_RTC_WKUP					(3)
#define IRQ_NO_FLASH					(4)
#define IRQ_NO_RCC						(5)
#define IRQ_NO_EXTI0					(6)
#define IRQ_NO_EXTI1					(7)
#define IRQ_NO_EXTI2					(8)
#define IRQ_NO_EXTI3					(9)
#define IRQ_NO_EXTI4					(10)
#define IRQ_NO_CAN1_TX					(19)
#define IRQ_NO_CAN1_RX0					(20)
#define IRQ_NO_CAN1_RX1					(21)
#define IRQ_NO_CAN1_SCE					(22)
#define IRQ_NO_EXTI9_5					(23)
#define IRQ_NO_I2C1_EV					(31)
#define IRQ_NO_I2C1_ER					(32)
#define IRQ_NO_I2C2_EV					(33)
#define IRQ_NO_I2C2_ER					(34)
#define IRQ_NO_SPI1						(35)
#define IRQ_NO_SPI2						(36)
#define IRQ_NO_USART1					(37)
#define IRQ_NO_USART2					(38)
#define IRQ_NO_USART3					(39)
#define IRQ_NO_EXTI15_10				(40)
#define IRQ_NO_RTC_Alarm				(41)
#define IRQ_NO_SDIO						(49)
#define IRQ_NO_SPI3						(51)
#define IRQ_NO_UART4					(52)
#define IRQ_NO_UART5					(53)
#define IRQ_NO_CAN2_TX					(63)
#define IRQ_NO_CAN2_RX0					(64)
#define IRQ_NO_CAN2_RX1					(65)
#define IRQ_NO_CAN2_SCE					(66)
#define IRQ_NO_USART6					(71)
#define IRQ_NO_I2C3_EV					(72)
#define IRQ_NO_I2C3_ER					(73)


// IRQ Priority Macros
#define NVIC_IRQ_PRI0					(0)
#define NVIC_IRQ_PRI1					(1)
#define NVIC_IRQ_PRI2					(2)
#define NVIC_IRQ_PRI3					(3)
#define NVIC_IRQ_PRI4					(4)
#define NVIC_IRQ_PRI5					(5)
#define NVIC_IRQ_PRI6					(6)
#define NVIC_IRQ_PRI7					(7)
#define NVIC_IRQ_PRI8					(8)
#define NVIC_IRQ_PRI9					(9)
#define NVIC_IRQ_PRI10					(10)
#define NVIC_IRQ_PRI11					(11)
#define NVIC_IRQ_PRI12					(12)
#define NVIC_IRQ_PRI13					(13)
#define NVIC_IRQ_PRI14					(14)
#define NVIC_IRQ_PRI15					(15)
#define NVIC_IRQ_PRI16					(16)
#define NVIC_IRQ_PRI17					(17)
#define NVIC_IRQ_PRI18					(18)
#define NVIC_IRQ_PRI19					(19)
#define NVIC_IRQ_PRI20					(20)
#define NVIC_IRQ_PRI21					(21)
#define NVIC_IRQ_PRI22					(22)
#define NVIC_IRQ_PRI23					(23)
#define NVIC_IRQ_PRI24					(24)
#define NVIC_IRQ_PRI25					(25)
#define NVIC_IRQ_PRI26					(26)
#define NVIC_IRQ_PRI27					(27)
#define NVIC_IRQ_PRI28					(28)
#define NVIC_IRQ_PRI29					(29)
#define NVIC_IRQ_PRI30					(30)
#define NVIC_IRQ_PRI31					(31)
#define NVIC_IRQ_PRI32					(32)
#define NVIC_IRQ_PRI33					(33)
#define NVIC_IRQ_PRI34					(34)
#define NVIC_IRQ_PRI35					(35)
#define NVIC_IRQ_PRI36					(36)
#define NVIC_IRQ_PRI37					(37)
#define NVIC_IRQ_PRI38					(38)
#define NVIC_IRQ_PRI39					(39)
#define NVIC_IRQ_PRI40					(40)
#define NVIC_IRQ_PRI41					(41)
#define NVIC_IRQ_PRI42					(42)
#define NVIC_IRQ_PRI43					(43)
#define NVIC_IRQ_PRI44					(44)
#define NVIC_IRQ_PRI45					(45)
#define NVIC_IRQ_PRI46					(46)
#define NVIC_IRQ_PRI47					(47)
#define NVIC_IRQ_PRI48					(48)
#define NVIC_IRQ_PRI49					(49)
#define NVIC_IRQ_PRI50					(50)
#define NVIC_IRQ_PRI51					(51)
#define NVIC_IRQ_PRI52					(52)
#define NVIC_IRQ_PRI53					(53)
#define NVIC_IRQ_PRI54					(54)
#define NVIC_IRQ_PRI55					(55)
#define NVIC_IRQ_PRI56					(56)
#define NVIC_IRQ_PRI57					(57)
#define NVIC_IRQ_PRI58					(58)
#define NVIC_IRQ_PRI59					(59)
#define NVIC_IRQ_PRI60					(60)
#define NVIC_IRQ_PRI61					(61)
#define NVIC_IRQ_PRI62					(62)
#define NVIC_IRQ_PRI63					(63)
#define NVIC_IRQ_PRI64					(64)
#define NVIC_IRQ_PRI65					(65)
#define NVIC_IRQ_PRI66					(66)
#define NVIC_IRQ_PRI67					(67)
#define NVIC_IRQ_PRI68					(68)
#define NVIC_IRQ_PRI69					(69)
#define NVIC_IRQ_PRI70					(70)
#define NVIC_IRQ_PRI71					(71)
#define NVIC_IRQ_PRI72					(72)
#define NVIC_IRQ_PRI73					(73)
#define NVIC_IRQ_PRI74					(74)
#define NVIC_IRQ_PRI75					(75)
#define NVIC_IRQ_PRI76					(76)
#define NVIC_IRQ_PRI77					(77)
#define NVIC_IRQ_PRI78					(78)
#define NVIC_IRQ_PRI79					(79)
#define NVIC_IRQ_PRI80					(80)
#define NVIC_IRQ_PRI81					(81)
#define NVIC_IRQ_PRI82					(82)
#define NVIC_IRQ_PRI83					(83)
#define NVIC_IRQ_PRI84					(84)
#define NVIC_IRQ_PRI85					(85)
#define NVIC_IRQ_PRI86					(86)
#define NVIC_IRQ_PRI87					(87)
#define NVIC_IRQ_PRI88					(88)


#include "stm32f407xxgpio_drivers.h"
#include "stm32f407xxspi_drivers.h"
#include "stm32f407xxi2c_drivers.h"
#include "stm32f407xxusart_drivers.h"
#include "stm32f407xxrcc_drivers.h"
#endif /* INC_STM32F407XX_H_ */
