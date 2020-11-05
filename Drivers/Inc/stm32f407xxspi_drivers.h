/*
 * stm32f407xxspi_drivers.h
 *
 *  Created on: Sep 29, 2020
 *      Author: jhaab
 */

#ifndef INC_STM32F407XXSPI_DRIVERS_H_
#define INC_STM32F407XXSPI_DRIVERS_H_

#include "stm32f407xx.h"

/*
 *  SPI PIN CONFIGURATION STRUCTURE
 */
typedef struct{
	uint8_t SPI_DeviceMode;												// POSSIBLE VALUES @SPI_PIN_MODE
	uint8_t SPI_BusConfig;												// POSSIBLE VALUES @SPI_BUS_CONFIG
	uint8_t SPI_SclkSpeed;												// POSSIBLE VALUES @SPI_SCLK_SPEED
	uint8_t SPI_DFF;													// POSSIBLE VALUES @SPI_DATA_FRAME_FORMAT
	uint8_t SPI_CPOL;													// POSSIBLE VALUES @SPI_CPOL
	uint8_t SPI_CPHA;													// POSSIBLE VALUES @SPI_CPHA
	uint8_t SPI_SSM;													// POSSIBLE VALUES @SPI_SSM
}SPI_PinConfig_t;


/*
 * SPI HANDLE STRUCTURE
 */
typedef struct{

	SPI_RegDef_t *pSPIx; 												// POINTER TO HOLD THE BASE ADDRESS OF SPI PERIPHERAL SPI1,SPI2,SP3
	SPI_PinConfig_t SPI_Config;
	uint8_t *pTxBuffer;													// To store the Tx Buffer Address
	uint8_t *pRxBuffer;													// To store the Rx Buffer Address
	uint32_t TxLen;														// To store the Tx length
	uint32_t RxLen;														// To store the Rx length
	uint32_t TxState;													// To store the Tx State
	uint32_t RxState;													// To store the Rx State
}SPI_HANDLE_t;


/*@SPI_PIN_MODE
 *  SPI_PIN_MODE Macros
 */
#define SPI_MASTER_Mode					(1)
#define SPI_SLAVE_Mode					(0)

/*@SPI_BUS_CONFIG
 *  SPI_BUS_CONFIG Macros
 */
#define SPI_BUS_FD						(1)									// SPI FULL DUPLEX MODE
#define SPI_BUS_HD_TX					(2)									// SPI HALF DUPLEX MODE TX
#define SPI_BUS_HD_RX					(4)									// SPI HALF DUPLEX MODE RX
#define SPI_BUS_S_RX					(3)									// SPI SIMPLEX RX MODE ONLY

/*@SPI_SCLK_SPEED
 *  SPI_SCLK_SPEED Macros
 */
#define SPI_SPEED_DIV2					(0)									// SPI SCLK SPEED = PCLK/2(Peripheral Clock Speed)
#define SPI_SPEED_DIV4					(1)									// SPI SCLK SPEED = PCLK/4(Peripheral Clock Speed)
#define SPI_SPEED_DIV8					(2)									// SPI SCLK SPEED = PCLK/8(Peripheral Clock Speed)
#define SPI_SPEED_DIV16					(3)									// SPI SCLK SPEED = PCLK/16(Peripheral Clock Speed)
#define SPI_SPEED_DIV32					(4)									// SPI SCLK SPEED = PCLK/32(Peripheral Clock Speed)
#define SPI_SPEED_DIV64					(5)									// SPI SCLK SPEED = PCLK/64(Peripheral Clock Speed)
#define SPI_SPEED_DIV128				(6)									// SPI SCLK SPEED = PCLK/128(Peripheral Clock Speed)
#define SPI_SPEED_DIV256				(7)									// SPI SCLK SPEED = PCLK/256(Peripheral Clock Speed)

/*@SPI_DATA_FRAME_FORMAT
 *  SPI_DATA_FRAME_FORMAT Macros
 */
#define SPI_DFF_8BITS 					(0)									// SPI DATA FRAME FORMAT = 8-BIT SHIFT REGISTER
#define SPI_DFF_16BITS 					(1)									// SPI DATA FRAME FORMAT = 16-BIT SHIFT REGISTER

/*@SPI_CPOL
 *  SPI_CPOL Macros
 */
#define SPI_CPOL_HIGH					(1)									// SPI CLOCK POLARITY HIGH
#define SPI_CPOL_LOW					(0)									// SPI CLOCK POLARITY LOW

/*@SPI_CPHA
 *  SPI_CPHA Macros
 */
#define SPI_CPHA_HIGH					(1)									// SPI CLOCK PHASE HIGH
#define SPI_CPHA_LOW					(0)									// SPI CLOCK PHASE LOW

/*@SPI_CPHA
 *  SPI_CPHA Macros
 */
#define SPI_SSM_EN 						(1)									// SPI SOFTWARE SLAVE MANAGEMENT ON
#define SPI_SSM_DI 						(0)									// SPI SOFTWARE SLAVE MANAGEMENT OFF

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG    ( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG   ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG   ( 1 << SPI_SR_BSY)



// SPI TX and RX Buffer
#define TX_Buffer_EMPTY					(1)
#define TX_Buffer_NOT_EMPTY				(0)
#define RX_Buffer_EMPTY					(0)
#define RX_Buffer_NOT_EMPTY				(1)

// Possible SPI Application States
#define SPI_READY						(0)
#define SPI_BUSY_TX						(1)
#define SPI_BUSY_RX						(2)

// Possible SPI Application Event States
#define SPI_EVENT_TX_CMPLT				(1)
#define SPI_EVENT_RX_CMPLT				(2)
#define SPI_EVENT_OVR_ERR				(3)

/************************************************************************************************************************************************************************
 * 																					API Supported by this Driver
 * 															For More Details Please Check the definitions of the particular functions
 **************************************************************************************************************************************************************************/

/*
 *  Peripheral Clock Setup
 */
void SPI_PClkCtrl(SPI_RegDef_t *pSPIx, uint8_t STATE);

/*
 *  Peripheral Init and DeInit Setup
 */
void SPI_Init(SPI_HANDLE_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 *  Peripheral Data Read And Write Setup
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * Interrupt Peripheral Data Read And Write Setup
 */
uint8_t SPI_SendDataIT(SPI_HANDLE_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_RecieveDataIT(SPI_HANDLE_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 *  Peripheral IRQ Configuration and ISR Handling Setup
 */
void SPI_IRQITConfig(uint8_t IRQNumber,  uint8_t IRQEN_DI);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_HANDLE_t *pSPIHandle);

/*
 * 	Other Peripheral API Setup
 */
// SPI ENABLE OR DISABLE
void SPI_PCtrl(SPI_RegDef_t *pSPIx, uint8_t STATE);
void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t STATE);
void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t STATE);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
uint8_t SPI_VerifyResponse(uint8_t ack_byte);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmisson(SPI_HANDLE_t *pSPIHandle);
void SPI_CloseReception(SPI_HANDLE_t *pSPIHandle);


/*
 * 	Application Callback
 */
void SPI_ApplicationEventCallback(SPI_HANDLE_t *pSPIHandle,uint8_t AppEv);
#endif /* INC_STM32F407XXSPI_DRIVERS_H_ */
