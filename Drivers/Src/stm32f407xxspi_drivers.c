/*
 * stm32f407xxspi_drivers.c
 *
 *  Created on: Sep 29, 2020
 *      Author: jhaab
 */

#include "stm32f407xxspi_drivers.h"

/*
 *  Peripheral Private helper Functions Declaration
 */
static void spi_txe_interrupt_handle(SPI_HANDLE_t *pSPIHandle);
static void spi_rxe_interrupt_handle(SPI_HANDLE_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_HANDLE_t *pSPIHandle);

/*
 *  Peripheral Clock Setup
 */
void SPI_PClkCtrl(SPI_RegDef_t *pSPIx, uint8_t STATE){

	if(STATE == ENABLE){

			if(pSPIx == SPI1){

				SPI1_PCLK_EN();

			}else if(pSPIx == SPI2){

				SPI2_PCLK_EN();

			}else if(pSPIx == SPI3){

				SPI3_PCLK_EN();
			}
		}else if(STATE == DISABLE){

			if(pSPIx == SPI1){

				SPI1_PCLK_DI();

			}else if(pSPIx == SPI2){

				SPI2_PCLK_DI();

			}else if(pSPIx == SPI3){

				SPI3_PCLK_DI();

			}
		}
}

/*
 *  Peripheral Init and DeInit Setup
 */
void SPI_Init(SPI_HANDLE_t *pSPIHandle){

	// Enable the Peripheral Clock
	SPI_PClkCtrl(pSPIHandle->pSPIx, ENABLE);

	// CONFIGURE SPI_CR1 Register

	uint32_t tempReg = 0;

	// CONFIGURE DEVICE MODE
	tempReg |= (pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);

	// CONFIGURE DEVICE BUS
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_FD){

		// Full Duplex: BIDIMODE=0, RXONLY=0
		tempReg &= ~( 1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_HD_TX){

		// Half-Duplex, Tx: BIDIMODE=1, BIDIOE=1
		tempReg |= ( 1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_S_RX){

		// Simplex (unidirectional receive-only): BIDIMODE=0, RXONLY=1
		//BIDI mode should be cleared
		tempReg &= ~( 1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempReg |= ( 1 << SPI_CR1_RX_ONLY);
	}

	// CONFIGURE DEVICE CLOCK SPEED
	tempReg |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_SPEED);

	// CONFIGURE DEVICE DATA_FRAME_FORMAT
	tempReg |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	// CONFIGURE DEVICE SPI_CPOL
	tempReg |= (pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	// CONFIGURE DEVICE SPI_CPHA
	tempReg |= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	// CONFIGURE DEVICE SPI_SSM
	 tempReg |= (pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM);


	pSPIHandle->pSPIx->SPI_CR1 = tempReg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx){
		if(pSPIx == SPI1){
			SPI1_REG_RESET();
		}else if(pSPIx == SPI2){
			SPI2_REG_RESET();
		}else if(pSPIx == SPI3){
			SPI3_REG_RESET();
		}
}

/*
 *  Peripheral Data Send And Recieve Setup
 *  This is a Blocking Call as the while loop will not exit untill it has trnsmitted all its data
 *  Later we will write a code whis is not blocking
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

	while(Len > 0){

		// 1. Wait till TXE is Empty
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)  == FLAG_RESET );


		// 2.Check the DFF
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){
			// 16-Bit
			pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);  // Writing 16 bit Data to Data Register by typecasting the pTXBuffer to 16 bit and the DeRefrencing it.
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else{
			// 8-Bit
			pSPIx->SPI_DR = *pTxBuffer;  // Writing 8 bit Data to Data Register.
			Len--;
			pTxBuffer++;
		}
	}
}

void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){

	while(Len > 0){

		// 1. wait till the buffer Flag is set
		while( (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG)) == FLAG_RESET );

		// 2. Check the DFF
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){
			// 16-Bit
			*((uint16_t*)pRxBuffer) = pSPIx->SPI_DR;  // Reading 16 bit Data to Data Register by typecasting the pRxBuffer to 16 bit and the DeRefrencing it.
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else{
			// 8-Bit
			*pRxBuffer = pSPIx->SPI_DR;  // Reading 8 bit Data to Data Register.
			Len--;
			pRxBuffer++;
		}
	}
}

/*
 * Interrupt Peripheral Data Read And Write Setup
 */
uint8_t SPI_SendDataIT(SPI_HANDLE_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){
  // Only Do the Below if the State is not In Tx
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_TX){

		// 1. First save the Tx buffer address and Len in some global variable
			pSPIHandle->pTxBuffer = pTxBuffer;
			pSPIHandle->TxLen = Len;
		// 2. Mark the SPI State as Busy in transmission so that no other code takes over SPI peripheral during transmission
			pSPIHandle->TxState = SPI_BUSY_TX;
		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is SET in Status Register
			pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);
		// 4. Data Transmission will be handled by the ISR code

	}
	return state;
}

uint8_t SPI_RecieveDataIT(SPI_HANDLE_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){

  // Only Do the Below if the State is not in Rx
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_RX){

		// 1. First save the Tx buffer address and Len in some global variable
			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->RxLen = Len;
		// 2. Mark the SPI State as Busy in transmission so that no other code takes over SPI peripheral during transmission
			pSPIHandle->RxState = SPI_BUSY_RX;
		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is SET in Status Register
			pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);
		// 4. Data Transmission will be handled by the ISR code

	}
	return state;
}

/*
 *  Peripheral IRQ Configuration and ISR Handling Setup
 */
void SPI_IRQITConfig(uint8_t IRQNumber,  uint8_t IRQEN_DI){
	if(IRQEN_DI == ENABLE){
		if(IRQNumber <= 31){
			// ISER0 is Configured
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){
			// ISER1 is Configured
			*NVIC_ISER1 |= (1 << ( IRQNumber % 32 ) );
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			// ISER2 is Configured
			*NVIC_ISER2 |= (1 << ( IRQNumber % 64 ) );
		}
	}else{
		if(IRQNumber <= 31){
			// ICER0 is Configured
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64){
			// ICER1 is Configured
			*NVIC_ICER1 |= (1 << ( IRQNumber % 32 ) );
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			// ICER2 is Configured
			*NVIC_ICER2 |= (1 << ( IRQNumber % 64 ) );
		}
	}


}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	// First find Out IPR Register
	uint8_t iprx = (IRQNumber / 4);
	uint8_t iprx_section = (IRQNumber % 4);
	uint8_t shift_amout = (8 * iprx_section) + (8 - PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR + (iprx)) |= ((IRQPriority) << shift_amout);
}

void SPI_IRQHandling(SPI_HANDLE_t *pSPIHandle){
	// Determining Which Status Flag is triggered by checking the Status Register
	uint8_t temp1, temp2;

	// 1. First Check for TXE Flag
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_TXE_FLAG);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2){
		// handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	// 2. Check for RXNE Flag
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_RXNE_FLAG);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2){
		// handle RXE
		spi_rxe_interrupt_handle(pSPIHandle);
	}


	// 3. Check For Overrun Flag
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2){
		// handle OVR
		spi_ovr_interrupt_handle(pSPIHandle);
	}

}


/*
 * 	Other Peripheral API Setup
 */
// Get the Flag Status Of the Flag Given
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SPI_SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

// SPI ENABLE OR DISABLE
void SPI_PCtrl(SPI_RegDef_t *pSPIx, uint8_t STATE){
	if(STATE == ENABLE){

		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPI_EN);

	}else{

		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPI_EN);

	}
}

// SPI SSI BIT SET OR RESET
void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t STATE){
	if(STATE == ENABLE)
	{
		pSPIx->SPI_CR1 |=  (1 << SPI_CR1_SSI);

	}else
	{
		pSPIx->SPI_CR1 &=  ~(1 << SPI_CR1_SSI);
	}
}

// SPI SSOE BIT SET OR RESET
void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t STATE){
	if(STATE == ENABLE){
		pSPIx->SPI_CR2 |= (1 << SPI_CR2_SSOE);
	}else{
		pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

// SPI Acknowledgement Bit Check
uint8_t SPI_VerifyResponse(uint8_t ack_byte){

	if(ack_byte == 0xF5){ // this 0xF5 can be changed as it is only a check of values
		// ACK
		return 1;
	}
	// NACK
	return 0;
}

void SPI_CloseTransmisson(SPI_HANDLE_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= !( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_HANDLE_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= !( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}



/*
 *  Peripheral Private helper Functions
 */
static void spi_txe_interrupt_handle(SPI_HANDLE_t *pSPIHandle){

	// 1.Check the DFF
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){
		// 16-Bit
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuffer);  // Writing 16 bit Data to Data Register by typecasting the pTXBuffer to 16 bit and the DeRefrencing it.
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else{
		// 8-Bit
		pSPIHandle->pSPIx->SPI_DR = *(pSPIHandle->pTxBuffer);  // Writing 8 bit Data to Data Register.
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	if(! pSPIHandle->TxLen){
		// CLOSE THE SPI TRANSMISSION and DISABLE THE TXEIE BIT
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
//		SPI_PCtrl(pSPIHandle->pSPIx, DISABLE);
	}
}

static void spi_rxe_interrupt_handle(SPI_HANDLE_t *pSPIHandle){
	// 1.Check the DFF
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF)){

		// 16-Bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->SPI_DR;  // Writing 16 bit Data to Data Register by typecasting the pRxBuffer to 16 bit and the DeRefrencing it.
		pSPIHandle->RxLen-= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;
	}else{
		// 8-Bit
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->SPI_DR;  // Writing 8 bit Data to Data Register.
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}
	if(! pSPIHandle->RxLen){
		// CLOSE THE SPI TRANSMISSION and DISABLE THE TXEIE BIT
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_interrupt_handle(SPI_HANDLE_t *pSPIHandle){

	uint8_t temp;
	// 1. Clear the OVR FLag
	if(pSPIHandle->TxState != SPI_BUSY_TX){
		temp = pSPIHandle->pSPIx->SPI_DR;
		temp = pSPIHandle->pSPIx->SPI_SR;
	}
	(void)temp;
	// 2. Inform the Application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){

	uint8_t temp;
	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_HANDLE_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}

