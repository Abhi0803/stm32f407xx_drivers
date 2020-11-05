/*
 * 104SPI_cmd_handling_it.c
 *
 *  Created on: Oct 6, 2020
 *      Author: jhaab
 */


#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"



//command codes
#define COMMAND_LED_CTRL      		0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ      		0x52
#define COMMAND_PRINT      			0x53
#define COMMAND_ID_READ      		0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1
#define ANALOG_PIN2 	2
#define ANALOG_PIN3 	3
#define ANALOG_PIN4 	4

//arduino led

#define LED_PIN  9

SPI_HANDLE_t SPI2handle;


uint8_t RcvBuff[100];

uint8_t ReadByte;

uint8_t RxContFlag = RESET;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

void SPI2_GPIOInits(void)
{
	GPIO_HANDLE_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPIPins);


	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);


}

void SPI2_Inits(void)
{

	SPI_HANDLE_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_FD;
	SPI2handle.SPI_Config.SPI_DeviceMode = SPI_MASTER_Mode;
	SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SPEED_DIV8;//generates sclk of 2MHz
	SPI2handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_DI; //Hardware slave management enabled for NSS pin

	SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_HANDLE_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

}



int main(void)
{


	GPIO_ButtonInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOE_Config(SPI2,ENABLE);

	SPI_IRQITConfig(IRQ_NO_SPI2,ENABLE);

	//wait till button is pressed
	while( ! GPIO_ReadPin(GPIOA,GPIO_PIN_0) );

	//to avoid button de-bouncing related issues 200ms of delay
	delay();


	//enable the SPI2 peripheral
	SPI_PCtrl(SPI2,ENABLE);

	RxContFlag = SET;

	while(RxContFlag == SET)
	{
	   while ( ! (SPI_RecieveDataIT(&SPI2handle,&ReadByte,1) == SPI_READY) );
	}


	//lets confirm SPI is not busy
	while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

	//Disable the SPI2 peripheral
	SPI_PCtrl(SPI2,DISABLE);

	return 0;

}

void SPI2_IRQHandler(void)
{

	SPI_IRQHandling(&SPI2handle);
}



void SPI_ApplicationEventCallback(SPI_HANDLE_t *pSPIHandle,uint8_t AppEv)
{
	static uint32_t i =0;
	static uint8_t  rcv_start = 0;
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
		if(ReadByte == 0XF1)
		{
			rcv_start = 1;
		}else
		{
			if(rcv_start)
			{
				if(ReadByte == '\r')
				{
					RxContFlag = RESET;
					rcv_start =0;
					RcvBuff[i++] = ReadByte; //place the \r
					i=0;
				}else
				{
					RcvBuff[i++] = ReadByte;

				}
			}
		}


	}

}
