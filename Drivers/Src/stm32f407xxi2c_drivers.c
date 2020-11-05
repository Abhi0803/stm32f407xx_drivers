/*
 * stm32f407xxi2c_drivers.c
 *
 *  Created on: Oct 7, 2020
 *      Author: jhaab
 */


#include "stm32f407xxi2c_drivers.h"

uint16_t AHB_Prescaler[8] =  {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_Prescaler[4] =  {2, 4, 8, 16};

/*
 *  Peripheral Private helper Functions Declaration
 */



static void I2C_SendSlaveAddress(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t SR);

void I2C_ClearADDRFlag(I2C_HANDLE_t *pI2CHandle);

static void I2C_MasterHandleTXEInterrupt(I2C_HANDLE_t *pI2CHandle );

static void I2C_MasterHandleRXNEInterrupt(I2C_HANDLE_t *pI2CHandle );





/*
 *  Peripheral Clock Setup
 */




void I2C_PClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t STATE){
	if(STATE == ENABLE){

				if(pI2Cx == I2C1){

					I2C1_PCLK_EN();

				}else if(pI2Cx == I2C2){

					I2C2_PCLK_EN();

				}else if(pI2Cx == I2C3){

					I2C3_PCLK_EN();
				}
			}else if(STATE == DISABLE){

				if(pI2Cx == I2C1){

					I2C1_PCLK_DI();

				}else if(pI2Cx == I2C2){

					I2C2_PCLK_DI();

				}else if(pI2Cx == I2C3){

					I2C3_PCLK_DI();

				}
			}
}




/*
 *  Peripheral Init and DeInit Setup
 */




void I2C_Init(I2C_HANDLE_t *pI2CHandle){

	uint32_t tempReg = 0;

	 //enable the peripheral clock

	I2C_PClkCtrl(pI2CHandle->pI2Cx, ENABLE);


	// ack control bit can only be set if the Peripheral Enable is SET, i.e. PE is set to 1
	// which does not happen now only after all the initialization is done then only PE is enabled
	// so we Enable the ACK after that and not here

	//configure the FREQ field of CR2
	tempReg = 0;
	tempReg |= RCC_GetPCLK1Value() /1000000U ;
	pI2CHandle->pI2Cx->I2C_CR2 =  (tempReg & 0x3F);

   //program the device own address
	tempReg = 0;
	tempReg |= pI2CHandle->I2C_Config.I2C_DeviceAdress << I2C_OAR1_ADD_7_1;
	tempReg |= ( 1 << I2C_OAR1_BIT14);
	pI2CHandle->pI2Cx->I2C_OAR1 = tempReg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempReg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCL_Speed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCL_Speed ) );
		tempReg |= (ccr_value & 0xFFF);
	}else
	{
		//mode is fast mode
		tempReg |= ( 1 << I2C_CCR_F_S);
		tempReg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCL_Speed ) );
		}else
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCL_Speed ) );
		}
		tempReg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->I2C_CCR = tempReg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCL_Speed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode

		tempReg = (RCC_GetPCLK1Value() /1000000U) + 1 ;

	}else
	{
		//mode is fast mode
		tempReg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;

	}

	pI2CHandle->pI2Cx->I2C_TRISE = (tempReg & 0x3F);
}




void I2C_DeInit(I2C_RegDef_t *pI2Cx){
		if(pI2Cx == I2C1){
			I2C1_REG_RESET();
		}else if(pI2Cx == I2C2){
			I2C2_REG_RESET();
		}else if(pI2Cx == I2C3){
			I2C3_REG_RESET();
		}
}




/*
 *  Peripheral MASTER SEND AND RECIEVE Configuration Setup
 */




void I2C_MasterSendData(I2C_HANDLE_t *pI2CHandle, uint8_t * pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr){

	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( !  I2C_GetFlagStatus_SR1(pI2CHandle->pI2Cx,I2C_SB_FLAG)   );

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	I2C_SendSlaveAddress(pI2CHandle->pI2Cx,SlaveAddr, I2C_SEND_RW);

	//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
	while( !  I2C_GetFlagStatus_SR1(pI2CHandle->pI2Cx,I2C_ADDR_FLAG)   );

	//5. clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6. send the data until len becomes 0

	while(Len > 0)
	{
		while(! I2C_GetFlagStatus_SR1(pI2CHandle->pI2Cx,I2C_TXE_FLAG) ); //Wait till TXE is set
		pI2CHandle->pI2Cx->I2C_DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)

	while(! I2C_GetFlagStatus_SR1(pI2CHandle->pI2Cx,I2C_TXE_FLAG) );

	while(! I2C_GetFlagStatus_SR1(pI2CHandle->pI2Cx,I2C_BTF_FLAG) );


	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_DISABLE_SR )
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}




void I2C_MasterRecvData(I2C_HANDLE_t *pI2CHandle, uint8_t * pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr){

	// 1. Generate the START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. confirm that start generation is completed by checking the SB flag in the SR1
		//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
		while( !  I2C_GetFlagStatus_SR1(pI2CHandle->pI2Cx,I2C_SB_FLAG)   );

	// 3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
		I2C_SendSlaveAddress(pI2CHandle->pI2Cx,SlaveAddr, I2C_RECV_RW);

	// 4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
		while( !  I2C_GetFlagStatus_SR1(pI2CHandle->pI2Cx,I2C_ADDR_FLAG)   );

	// 5. Procedure for 1 Byte
		if(Len == 1){

			// first disable the ack
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2c_ACK_DI);

			// clear the addr Flag
			I2C_ClearADDRFlag(pI2CHandle);

			// wait until RXNE becomes 1
			while( !  I2C_GetFlagStatus_SR1(pI2CHandle->pI2Cx,I2C_RXNE_FLAG)  );

			// generate the STOP Condition
			if(Sr == I2C_DISABLE_SR )
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


			// Read data in to buffer
			* pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

		}


	// 6. Procedure for Len > 1
		if(Len > 1){

			// clear the ADDR Flag
			I2C_ClearADDRFlag(pI2CHandle);

			// Read data in to buffer until Len becomes zero
			for(uint32_t i = Len; i > 0; i-- ){

				// wait until RXNE becomes 1
//				pI2CHandle->pI2Cx->I2C_SR1 |= I2C_RXNE_FLAG;
				while( !  I2C_GetFlagStatus_SR1(pI2CHandle->pI2Cx,I2C_RXNE_FLAG)  );



				if(i == 2){ // if last 2bytes are remaining

					// first disable the ack
					I2C_ManageAcking(pI2CHandle->pI2Cx, I2c_ACK_DI);

					//generate STOP condition
					if(Sr == I2C_DISABLE_SR )
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}

				// Read data in to buffer
				* pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

				// increment the buffer address
				pRxBuffer++;
			}


		}

	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2c_ACK_EN)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2c_ACK_EN);
	}

}






/*
 *  Peripheral SLAVE SEND AND RECIEVE Configuration Setup
 */




void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data){

	pI2Cx->I2C_DR = data;

}




uint8_t I2C_SlaveRecvData(I2C_RegDef_t *pI2Cx){

	return (uint8_t)pI2Cx->I2C_DR;

}





/*
 *  Peripheral INTERRUPT MASTER SEND AND RECIEVE Configuration Setup
 */




uint8_t  I2C_MasterSendDataIT(I2C_HANDLE_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;

}




uint8_t I2C_MasterRecvDataIT(I2C_HANDLE_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}




/*
 *  Peripheral IRQ Configuration and ISR Handling Setup
 */




void I2C_IRQITConfig(uint8_t IRQNumber,  uint8_t IRQEN_DI){
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




void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	// First find Out IPR Register
	uint8_t iprx = (IRQNumber / 4);
	uint8_t iprx_section = (IRQNumber % 4);
	uint8_t shift_amout = (8 * iprx_section) + (8 - PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR + (iprx)) |= ((IRQPriority) << shift_amout);
}




void I2C_EV_IRQHandling(I2C_HANDLE_t *pI2CHandle){
	//Interrupt handling for both master and slave mode of a device

	uint32_t temp1, temp2, temp3;

	temp1   = pI2CHandle->pI2Cx->I2C_CR2 & ( 1 << I2C_CR2_ITEVTEN) ;
	temp2   = pI2CHandle->pI2Cx->I2C_CR2 & ( 1 << I2C_CR2_ITBUFEN) ;

	//1. Handle For interrupt generated by SB event
		//	Note : SB flag is only applicable in Master mode
	temp3  = pI2CHandle->pI2Cx->I2C_SR1 & ( 1 << I2C_SR1_SB);


	if(temp1 && temp3)
	{
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block lets executed the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_SendSlaveAddress(pI2CHandle->pI2Cx,pI2CHandle->DevAddr, I2C_SEND_RW);
		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			I2C_SendSlaveAddress(pI2CHandle->pI2Cx,pI2CHandle->DevAddr, I2C_RECV_RW);
		}
	}

	//2. Handle For interrupt generated by ADDR event
		//Note : When master mode : Address is sent

	temp3  = pI2CHandle->pI2Cx->I2C_SR1 & ( 1 << I2C_SR1_ADDR);

	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3)
	{
		// interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event

	temp3  = pI2CHandle->pI2Cx->I2C_SR1 & ( 1 << I2C_SR1_BTF);

	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure that TXE is also set .
			if(pI2CHandle->pI2Cx->I2C_SR1 & ( 1 << I2C_SR1_TxE) )
			{
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0 )
				{
					//1. generate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2. reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//3. notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);

				}
			}

		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			;
		}
	}

	//4. Handle For interrupt generated by STOPF event
		// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
		//The below code block will not be executed by the master since STOPF will not set in master mode

	temp3  = pI2CHandle->pI2Cx->I2C_SR1 & ( 1 << I2C_SR1_STOPF);

	if(temp1 && temp3)
	{
		//STOF flag is set
		//Clear the STOPF ( i.e 1) read SR1 2) Write to CR1 )

		pI2CHandle->pI2Cx->I2C_CR1 |= 0x0000;

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
	}


	//5. Handle For interrupt generated by TXE event

	temp3  = pI2CHandle->pI2Cx->I2C_SR1 & ( 1 << I2C_SR1_TxE);

	if(temp1 && temp2 && temp3)
	{
		//Check for device mode
		if(pI2CHandle->pI2Cx->I2C_SR2 & ( 1 << I2C_SR2_MSL))
		{
			//TXE flag is set
			//We have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else
		{
			//slave
			//make sure that the slave is really in transmitter mode
		    if(pI2CHandle->pI2Cx->I2C_SR2 & ( 1 << I2C_SR2_TRA))
		    {
		    	I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
		    }
		}
	}

	//6. Handle For interrupt generated by RXNE event

	temp3  = pI2CHandle->pI2Cx->I2C_SR1 & ( 1 << I2C_SR1_RxNE);

	if(temp1 && temp2 && temp3)
	{
		//check device mode .
		if(pI2CHandle->pI2Cx->I2C_SR2 & ( 1 << I2C_SR2_MSL))
		{
			//The device is master

			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);

			}

		}else
		{
			//slave
			//make sure that the slave is really in receiver mode
			if(!(pI2CHandle->pI2Cx->I2C_SR2 & ( 1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
			}
		}
	}
}




void I2C_ER_IRQHandling(I2C_HANDLE_t *pI2CHandle){
	{

		uint32_t temp1,temp2;

	    //Know the status of  ITERREN control bit in the CR2
		temp2 = (pI2CHandle->pI2Cx->I2C_CR2) & ( 1 << I2C_CR2_ITERREN );


	/***********************Check for Bus error************************************/
		temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1<< I2C_SR1_BERR);
		if(temp1  && temp2 )
		{
			//This is Bus error

			//Implement the code to clear the buss error flag
			pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_BERR);

			//Implement the code to notify the application about the error
		   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
		}

	/***********************Check for arbitration lost error************************************/
		temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_ARLO );
		if(temp1  && temp2)
		{
			//This is arbitration lost error

			//Implement the code to clear the arbitration lost error flag
			pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_ARLO);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

		}

	/***********************Check for ACK failure  error************************************/

		temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_AF);
		if(temp1  && temp2)
		{
			//This is ACK failure error

		    //Implement the code to clear the ACK failure error flag
			pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_AF);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
		}

	/***********************Check for Overrun/underrun error************************************/
		temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_OVR);
		if(temp1  && temp2)
		{
			//This is Overrun/underrun

		    //Implement the code to clear the Overrun/underrun error flag
			pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_OVR);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
		}

	/***********************Check for Time out error************************************/
		temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_TIMEOUT);
		if(temp1  && temp2)
		{
			//This is Time out error

		    //Implement the code to clear the Time out error flag
			pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

			//Implement the code to notify the application about the error
			I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
		}

	}
}




/*
 * 	Other Peripheral API Setup
 */




// I2C ENABLE OR DISABLE
void I2C_PCtrl(I2C_RegDef_t *pI2Cx, uint8_t STATE){
	if(STATE == ENABLE){

		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_I2C_EN);

	}else{

		pI2Cx->I2C_CR1 &= ~(1 << 0);

	}
}




// Get the Flag Status Of the Flag Given
uint8_t I2C_GetFlagStatus_SR1(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->I2C_SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}




uint8_t I2C_GetFlagStatus_SR2(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->I2C_SR2 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}




__weak void I2C_ApplicationEventCallback(I2C_HANDLE_t *pI2CHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}




void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2c_ACK_EN)
	{
		//enable the ack
		pI2Cx->I2C_CR1 |= ( 1 << I2C_CR1_ACK);
	}else
	{
		//disable the ack
		pI2Cx->I2C_CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}




void I2C_CloseSendData(I2C_HANDLE_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}




void I2C_CloseReceiveData(I2C_HANDLE_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2c_ACK_EN)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}

}




void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->I2C_CR1 |= ( 1 << I2C_CR1_START);
}




void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->I2C_CR1 |= ( 1 << I2C_CR1_STOP);
}




void I2C_SlaveEnableOrDisable_CallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t STATE){
	if(STATE == ENABLE){

		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);

	}else{

		pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITERREN);

	}

}




/*
 *  Peripheral Private helper Functions Defintion
 */





static void I2C_SendSlaveAddress(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t SR){

	SlaveAddr = SlaveAddr << 1;
	if(SR){
		SlaveAddr |=  (1); //SlaveAddr is Slave address + r/nw bit=1 RECIEVE
	}else{
		SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/nw bit=0 SEND
	}

	pI2Cx->I2C_DR = SlaveAddr;
}




void I2C_ClearADDRFlag(I2C_HANDLE_t *pI2CHandle){
	uint32_t dummy_read;
		//check for device mode
		if(pI2CHandle->pI2Cx->I2C_SR2 & ( 1 << I2C_SR2_MSL))
		{
			//device is in master mode
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				if(pI2CHandle->RxSize  == 1)
				{
					//first disable the ack
					I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

					//clear the ADDR flag ( read SR1 , read SR2)
					dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
					dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
					(void)dummy_read;
				}

			}
			else
			{
				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
				dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
				(void)dummy_read;

			}

		}
		else
		{
			//device is in slave mode
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
			dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
			(void)dummy_read;
		}
}




static void I2C_MasterHandleTXEInterrupt(I2C_HANDLE_t *pI2CHandle )
{
	if(pI2CHandle->TxLen > 0)
	{
		//1. load the data in to DR
		pI2CHandle->pI2Cx->I2C_DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;

	}

}




static void I2C_MasterHandleRXNEInterrupt(I2C_HANDLE_t *pI2CHandle )
{
	//We have to do the data reception
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
		pI2CHandle->RxLen--;

	}


	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
		}

			//read DR
			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0 )
	{
		//close the I2C data reception and notify the application

		//1. generate the stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2 . Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
	}
}




