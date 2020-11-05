/*
 * stm32f407xxi2c_drivers.h
 *
 *  Created on: Oct 7, 2020
 *      Author: jhaab
 */

#ifndef INC_STM32F407XXI2C_DRIVERS_H_
#define INC_STM32F407XXI2C_DRIVERS_H_

#include "stm32f407xx.h"

/* I2C PIN CONFIGURATION STRUCTURE */
typedef struct{
	uint32_t I2C_SCL_Speed;												// POSSIBLE VALUES @I2C_SCL_SPEED
	uint8_t I2C_DeviceAdress;											// POSSIBLE VALUES @I2C_DeviceAdress
	uint8_t I2C_ACKControl;												// POSSIBLE VALUES @I2C_ACKControl
	uint16_t I2C_FMDutyCycle;											// POSSIBLE VALUES @I2C_FMDutyCycle
}I2C_PinConfig_t;


/* I2C HANDLE STRUCTURE */
typedef struct{

	I2C_RegDef_t *pI2Cx; 												// POINTER TO HOLD THE BASE ADDRESS OF SPI PERIPHERAL I2C1,I2C2,I2C3
	I2C_PinConfig_t I2C_Config;
	uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Tx len > */
	uint8_t 		TxRxState;	/* !< To store Communication state > */
	uint8_t 		DevAddr;	/* !< To store slave/device address > */
    uint32_t        RxSize;		/* !< To store Rx size  > */
    uint8_t         Sr;			/* !< To store repeated start value  > */
}I2C_HANDLE_t;


/*
 * I2C application states
 */
#define I2C_READY 					0
#define I2C_BUSY_IN_RX 				1
#define I2C_BUSY_IN_TX 				2

#define I2C_SEND_RW 				0
#define I2C_RECV_RW 				1

/*@I2C_SCL_SPEED
 *  I2C_SCL_Speed Macros
 */
#define I2C_SCL_SPEED_SM 				(100000)						// Standard Mode Speed is 100 KHz
#define I2C_SCL_SPEED_FM 				(400000)						// Standard Mode Speed is 400 KHz

/*@I2C_ACKControl
 *  I2C_ACKControl Macros
 */
#define I2c_ACK_EN						(1)								// Enable the ACknowledgement Control
#define I2c_ACK_DI						(0)								// Disable the ACknowledgement Control

/*@I2C_FMDutyCycle
 *  I2C_FMDutyCycle Macros
 */
#define I2C_FM_DUTY_2					(0)								// Duty Cycle is 2
#define I2C_FM_DUTY_16_9				(1)								// Duty Cycle is 9/16

/*
 * I2C related status flags definitions SR1 and SR2
 */
#define I2C_TXE_FLAG    				( 1 << I2C_SR1_TxE)
#define I2C_RXNE_FLAG   				( 1 << I2C_SR1_RxNE)
#define I2C_ADDR_FLAG   				( 1 << I2C_SR1_ADDR)
#define I2C_SB_FLAG   					( 1 << I2C_SR1_SB)
#define I2C_BTF_FLAG    				( 1 << I2C_SR1_BTF)
#define I2C_STOPF_FLAG   				( 1 << I2C_SR1_STOPF)
#define I2C_BERR_FLAG   				( 1 << I2C_SR1_BERR)
#define I2C_ARLO_FLAG    				( 1 << I2C_SR1_ARLO)
#define I2C_AF_FLAG   					( 1 << I2C_SR1_AF)
#define I2C_OVR_FLAG   					( 1 << I2C_SR1_OVR)
#define I2C_PECERR_FLAG    				( 1 << I2C_SR1_PECERR)
#define I2C_TIMEOUT_FLAG   				( 1 << I2C_SR1_TIMEOUT)
#define I2C_SMBALERT_FLAG   			( 1 << I2C_SR1_SMBALERT)

#define I2C_MSL_FLAG    				( 1 << I2C_SR2_MSL)
#define I2C_BUSY_FLAG   				( 1 << I2C_SR2_BUSY)
#define I2C_TRA_FLAG   					( 1 << I2C_SR2_TRA)
#define I2C_GENCALL_FLAG    			( 1 << I2C_SR2_GENCALL)
#define I2C_SMBDEFAULT_FLAG   			( 1 << I2C_SR2_SMBDEFAULT)
#define I2C_SMBHOST_FLAG   				( 1 << I2C_SR2_SMBHOST)
#define I2C_DUALF_FLAG    				( 1 << I2C_SR2_DUALF)
#define I2C_PEC_15_8_FLAG   			( 1 << I2C_SR2_PEC_15_8)

#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET

/*
 * I2C application events macros
 */
#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_ERROR_BERR 	 		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9


/************************************************************************************************************************************************************************
 * 																					API Supported by this Driver
 * 															For More Details Please Check the definitions of the particular functions
 **************************************************************************************************************************************************************************/

/*
 *  Peripheral Clock Setup
 */
void I2C_PClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t STATE);

/*
 *  Peripheral Init and DeInit Setup
 */
void I2C_Init(I2C_HANDLE_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 *  Peripheral MASTER SEND AND RECIEVE Configuration Setup
 */
void I2C_MasterSendData(I2C_HANDLE_t *pI2CHandle, uint8_t * pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterRecvData(I2C_HANDLE_t *pI2CHandle, uint8_t * pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

/*
 *  Peripheral SLAVE SEND AND RECIEVE Configuration Setup
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveRecvData(I2C_RegDef_t *pI2Cx);

/*
 *  Peripheral INTERRUPT MASTER SEND AND RECIEVE Configuration Setup
 */
uint8_t I2C_MasterSendDataIT(I2C_HANDLE_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterRecvDataIT(I2C_HANDLE_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
/*
 *  Peripheral IRQ Configuration and ISR Handling Setup
 */
void I2C_IRQITConfig(uint8_t IRQNumber,  uint8_t IRQEN_DI);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_HANDLE_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_HANDLE_t *pI2CHandle);

/*
 * 	Other Peripheral API Setup
 */
// I2C ENABLE OR DISABLE
void I2C_PCtrl(I2C_RegDef_t *pI2Cx, uint8_t STATE);
uint8_t I2C_GetFlagStatus_SR1(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
uint8_t I2C_GetFlagStatus_SR2(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_SlaveEnableOrDisable_CallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t STATE);

/*
 * 	Application Callback
 */
void I2C_ApplicationEventCallback(I2C_HANDLE_t *pI2CHandle,uint8_t AppEv);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_CloseSendData(I2C_HANDLE_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_HANDLE_t *pI2CHandle);
#endif /* INC_STM32F407XXI2C_DRIVERS_H_ */
