/*
 * stm32f407xxuart_drivers.h
 *
 *  Created on: Oct 10, 2020
 *      Author: jhaab
 */

#ifndef INC_STM32F407XXUSART_DRIVERS_H_
#define INC_STM32F407XXUSART_DRIVERS_H_

#include "stm32f407xx.h"


/*
 *  USART PIN CONFIGURATION STRUCTURE
 */
typedef struct{
	uint8_t USART_Mode;													// POSSIBLE VALUES @USART_Mode
	uint32_t USART_Buad	;												// POSSIBLE VALUES @USART_Baud
	uint8_t USART_STOPBits;												// POSSIBLE VALUES @USART_NoOfStopBits
	uint8_t USART_WORDLEN;												// POSSIBLE VALUES @USART_WordLength
	uint8_t USART_PARITY;												// POSSIBLE VALUES @USART_ParityControl
	uint8_t USART_HWFlowCtrl;											// POSSIBLE VALUES @USART_HWFlowControl
}USART_PinConfig_t;


/*
 * USART HANDLE STRUCTURE
 */
typedef struct{

	USART_RegDef_t *pUSARTx; 											// POINTER TO HOLD THE BASE ADDRESS OF USART PERIPHERAL USART1,USART2,SP3
	USART_PinConfig_t USART_Config;
	uint8_t *pTxBuffer;													// To store the Tx Buffer Address
	uint8_t *pRxBuffer;													// To store the Rx Buffer Address
	uint32_t TxLen;														// To store the Tx length
	uint32_t RxLen;														// To store the Rx length
	uint32_t TxState;													// To store the Tx State
	uint32_t RxState;													// To store the Rx State
}USART_HANDLE_t;

/*
 * USART application states
 */
#define USART_READY 					0
#define USART_BUSY_IN_RX 				1
#define USART_BUSY_IN_TX 				2

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 					0
#define USART_MODE_ONLY_RX 					1
#define USART_MODE_TXRX 					2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   				2
#define USART_PARITY_EN_EVEN  				1
#define USART_PARITY_DISABLE  			 	0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  				0
#define USART_WORDLEN_9BITS  				1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1    				0
#define USART_STOPBITS_0_5   				1
#define USART_STOPBITS_2     				2
#define USART_STOPBITS_1_5   				3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    			0
#define USART_HW_FLOW_CTRL_CTS    			1
#define USART_HW_FLOW_CTRL_RTS    			2
#define USART_HW_FLOW_CTRL_CTS_RTS			3

/*
 * USART flags
 */

#define USART_FLAG_TXE 						( 1 << USART_SR_TXE)
#define USART_FLAG_RXNE 					( 1 << USART_SR_RXNE)
#define USART_FLAG_TC 						( 1 << USART_SR_TC)

/*
 * Application states
 */
#define USART_BUSY_IN_RX 					1
#define USART_BUSY_IN_TX 					2
#define USART_READY 						0


#define 	USART_EVENT_TX_CMPLT  			0
#define		USART_EVENT_RX_CMPLT   			1
#define		USART_EVENT_IDLE     		 	2
#define		USART_EVENT_CTS       			3
#define		USART_EVENT_PE        			4
#define		USART_ERR_FE     				5
#define		USART_ERR_NE    	 			6
#define		USART_ERR_ORE    				7



/************************************************************************************************************************************************************************
 * 																					API Supported by this Driver
 * 															For More Details Please Check the definitions of the particular functions
 **************************************************************************************************************************************************************************/

/*
 *  Peripheral Clock Setup
 */
void USART_PClkCtrl(USART_RegDef_t *pUSARTx, uint8_t STATE);

/*
 *  Peripheral Init and DeInit Setup
 */
void USART_Init(USART_HANDLE_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 *  Peripheral Data Read And Write Setup
 */
void USART_SendData(USART_HANDLE_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_RecieveData(USART_HANDLE_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * Interrupt Peripheral Data Read And Write Setup
 */
uint8_t USART_SendDataIT(USART_HANDLE_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_RecieveDataIT(USART_HANDLE_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len);

/*
 *  Peripheral IRQ Configuration and ISR Handling Setup
 */
void USART_IRQITConfig(uint8_t IRQNumber,  uint8_t IRQEN_DI);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void USART_IRQHandling(USART_HANDLE_t *pUSARTHandle);

/*
 * 	Other Peripheral API Setup
 */
// USART ENABLE OR DISABLE
void USART_PCtrl(USART_RegDef_t *pUSARTx, uint8_t STATE);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName);
void USART_ClearFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);


/*
 * 	Application Callback
 */
void USART_ApplicationEventCallback(USART_HANDLE_t *pUSARTHandle,uint8_t AppEv);

#endif /* INC_STM32F407XXUSART_DRIVERS_H_ */
