/*
 * 204_I2C_Slave_Tx_Testing.c
 *
 *  Created on: Oct 9, 2020
 *      Author: jhaab
 */




#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"


#define SLAVE_ADDR  0x68

#define MY_ADDR 0x68;
void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_HANDLE_t I2C1Handle;

//rcv buffer
uint8_t Tx_buf[32]  = "This is abhinav ";

/*
 * PB6-> SCL
 * PB7 -> SDA
 */

void I2C1_GPIOInits(void)
{
	GPIO_HANDLE_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CPins);


	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2c_ACK_EN;
	I2C1Handle.I2C_Config.I2C_DeviceAdress= MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCL_Speed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_HANDLE_t GPIOBtn,GpioLed;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioLed);

}


int main(void)
{


	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//enable the i2c peripheral
	I2C_PCtrl(I2C1,ENABLE);


	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1,I2c_ACK_EN);

	//I2C IRQ configurations
	I2C_IRQITConfig(IRQ_NO_I2C1_EV,ENABLE);
	I2C_IRQITConfig(IRQ_NO_I2C1_ER,ENABLE);

	I2C_SlaveEnableOrDisable_CallbackEvents(I2C1,ENABLE);




	while(1);




}


void I2C1_EV_IRQHandler (void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}


void I2C1_ER_IRQHandler (void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}




void I2C_ApplicationEventCallback(I2C_HANDLE_t *pI2CHandle,uint8_t AppEv)
{

	static uint8_t commandCode = 0;
	static  uint8_t Cnt = 0;

	if(AppEv == I2C_EV_DATA_REQ)
	{
		//Master wants some data. slave has to send it
		if(commandCode == 0x51)
		{
			//send the length information to the master
			I2C_SlaveSendData(I2C1,strlen((char*)Tx_buf));
		}else if (commandCode == 0x52)
		{
			//Send the contents of Tx_buf
			I2C_SlaveSendData(I2C1,Tx_buf[Cnt++]);

		}
	}else if (AppEv == I2C_EV_DATA_RCV)
	{
		//Data is waiting for the slave to read . slave has to read it
		commandCode = I2C_SlaveRecvData(I2C1);

	}else if (AppEv == I2C_ERROR_AF)
	{
		//This will happen during slave transmitting data to master .
		// slave should understand master needs no more data
		//slave concludes end of Tx


		//if the current active code is 0x52 then dont invalidate
		if(! (commandCode == 0x52))
			commandCode = 0XFF;

		//reset the cnt variable because its end of transmission
		Cnt = 0;



	}
	else if (AppEv == I2C_EV_STOP)
	{
		Cnt = 0;
		//This happens only during slave reception .
		//Master has ended the I2C communication with the slave.
	}

}














