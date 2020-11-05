/*
 * stm32f407xxgpio_drivers.h
 *
 *  Created on: Sep 25, 2020
 *      Author: jhaab
 */

#ifndef INC_STM32F407XXGPIO_DRIVERS_H_
#define INC_STM32F407XXGPIO_DRIVERS_H_

#include "stm32f407xx.h"



/* GPIO PIN CONFIGURATION STRUCTURE */
typedef struct{
	uint8_t GPIO_PinNumber;												// POSSIBLE VALUES @GPIO_PINS
	uint8_t GPIO_PinMode;												// POSSIBLE VALUES @GPIO_PIN_MODE
	uint8_t GPIO_PinSpeed;												// POSSIBLE VALUES @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;										// POSSIBLE VALUES @GPIO_PIN_PUPD
	uint8_t GPIO_PinOPType;												// POSSIBLE VALUES @GPIO_PIN_OP_TYPE
	uint8_t GPIO_PinAltFunMode;											// POSSIBLE VALUES @GPIO_PIN_ALT_FUN_MODE
}GPIO_PinConfig_t;


/* GPIO HANDLE STRUCTURE */
typedef struct{
	// POINTER TO HOLD THE BASE ADDRESS OF GPIO PERIPHERAL
	GPIO_RegDef_t *pGPIOx; //GPIOA GPIOB...GPIOI
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_HANDLE_t;


/*@GPIO_PINS
 *  GPIO PINS Macros
 */
#define GPIO_PIN_0						(0)
#define GPIO_PIN_1						(1)
#define GPIO_PIN_2						(2)
#define GPIO_PIN_3						(3)
#define GPIO_PIN_4						(4)
#define GPIO_PIN_5						(5)
#define GPIO_PIN_6						(6)
#define GPIO_PIN_7						(7)
#define GPIO_PIN_8						(8)
#define GPIO_PIN_9						(9)
#define GPIO_PIN_10						(10)
#define GPIO_PIN_11						(11)
#define GPIO_PIN_12						(12)
#define GPIO_PIN_13						(13)
#define GPIO_PIN_14						(14)
#define GPIO_PIN_15						(15)

/*@LED_PINS
 *  LED PINS Macros
 */
#define LED_PIN_12						(1)
#define LED_PIN_13						(2)
#define LED_PIN_14						(4)
#define LED_PIN_15						(8)

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT     4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6

/*@GPIO_PIN_OP_TYPE
 *  GPIO OUTPUT TYPE Macros
 */
#define GPIO_OP_TYPE_PP  				(0)									// GPIO OUPUT PUSHPULL
#define GPIO_OP_TYPE_OD  				(1) 								// GPIO OUTPUT Open Drain

/*@GPIO_PIN_SPEED
 *  GPIO SPEED Macros
 */
#define GPIO_SPEED_LOW 					(0)									// GPIO OUTPUT SPPED LOW
#define GPIO_SPEED_MEDIUM				(1)									// GPIO OUTPUT SPPED MEDIUM
#define GPIO_SPEED_FAST					(2)									// GPIO OUTPUT SPPED FAST
#define GPIO_SPEED_HIGH 				(3)									// GPIO OUTPUT SPPED HIGH

/*@GPIO_PIN_PUPD
 *  GPIO PUPD Macros
 */
#define GPIO_NO_PUPD 					(0)									// GPIO OUTPUT NO PULLUP and NO PULLDOWN
#define GPIO_PIN_PU						(1)									// GPIO OUTPUT PULLUP
#define GPIO_PIN_PD						(2)									// GPIO OUTPUT PULLDOWN

/*@GPIO_PIN_ALT_FUN_MODE
 *  GPIO ALT FUNCTION Macros
 */

#define GPIO_ALT__AF0					(0)
#define GPIO_ALT__AF1					(1)
#define GPIO_ALT__AF2					(2)
#define GPIO_ALT__AF3					(3)
#define GPIO_ALT__AF4					(4)
#define GPIO_ALT__AF5					(5)
#define GPIO_ALT__AF6					(6)
#define GPIO_ALT__AF7					(7)
#define GPIO_ALT__AF8					(8)
#define GPIO_ALT__AF9					(9)
#define GPIO_ALT__AF10					(10)
#define GPIO_ALT__AF11					(11)
#define GPIO_ALT__AF12					(12)
#define GPIO_ALT__AF13					(13)
#define GPIO_ALT__AF14					(14)
#define GPIO_ALT__AF15					(15)


/************************************************************************************************************************************************************************
 * 																					API Supported by this Driver
 * 															For More Details Please Check the definitions of the particular functions
 **************************************************************************************************************************************************************************/

/*
 *  Peripheral Clock Setup
 */
void GPIO_PClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t STATE);
/*
 *  Peripheral Init and DeInit Setup
 */
void GPIO_Init(GPIO_HANDLE_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
/*
 *  Peripheral Data Read And Write Setup
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
void GPIO_ToggleLEDPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
/*
 *  Peripheral IRQ Configuration and ISR Handling Setup
 */
void GPIO_IRQITConfig(uint8_t IRQNumber,  uint8_t IRQEN_DI);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F407XXGPIO_DRIVERS_H_ */
