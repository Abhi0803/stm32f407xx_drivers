/*
 * 103SPI_Arduino_Tx_Rx.c
 *
 *  Created on: Oct 1, 2020
 *      Author: jhaab
 */


#include<string.h>
#include <stdio.h>
#include "stm32f407xx.h"

extern void initialise_monitor_handles(void);


//command codes
#define COMMAND_LED_CTRL          0x50
#define COMMAND_SENSOR_READ       0x51
#define COMMAND_LED_READ          0x52
#define COMMAND_PRINT         	  0x53
#define COMMAND_ID_READ       	  0x54

#define LED_ON    				  1
#define LED_OFF    				  0

//arduino analog pins
#define ANALOG_PIN0   			  0
#define ANALOG_PIN1               1
#define ANALOG_PIN2               2
#define ANALOG_PIN3  			  3
#define ANALOG_PIN4 			  4

#define LED_PIN					  9

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
	initialise_monitor_handles();

	GPIO_ButtonInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();
	printf("SPI is Initialised\n");
	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOE_Config(SPI2,ENABLE);

	while(1)
	{
		//wait till button is pressed
		while( ! GPIO_ReadPin(GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//enable the SPI2 peripheral
		SPI_PCtrl(SPI2,ENABLE);

		printf("SPI is Enabled\n");
	// 1. CMD_CTRL_LED  <pin no.>	<value>

		uint8_t command = COMMAND_LED_CTRL;
		uint8_t dummy_write = 0x0f;
		uint8_t dummy_read;
		uint8_t ack_byte;
		uint8_t	args[2];

		// Sending the first argument the Command
		SPI_SendData(SPI2, &command, 1);

		// Recieving a dummy byte to clear RXNE flag
		SPI_RecieveData(SPI2, &dummy_read, 1);

		// Sending a dummy byte so that slave responses the ACK or NACK
		SPI_SendData(SPI2, &dummy_write, 1);

		// Recieve ACK or NAck from Arduino
		SPI_RecieveData(SPI2, &ack_byte, 1);

		// Lets verify the ACK or NACK
		if(SPI_VerifyResponse(ack_byte)){
			// send arguments

			args[0] = LED_PIN;
			args[1] = LED_ON;

			SPI_SendData(SPI2, args, 2);

			printf("Command 1 is sent.\n");
		}

	// 2. COMMAND_SENSOR_READ

		//wait till button is pressed
		while( ! GPIO_ReadPin(GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();


		command = COMMAND_SENSOR_READ;
		uint8_t output;

		// Sending the first argument the Command
		SPI_SendData(SPI2, &command, 1);

		// Recieving a dummy byte to clear RXNE flag
		SPI_RecieveData(SPI2, &dummy_read, 1);

		// Sending a dummy byte so that slave responses the ACK or NACK
		SPI_SendData(SPI2, &dummy_write, 1);

		// Recieve ACK or NAck from Arduino
		SPI_RecieveData(SPI2, &ack_byte, 1);

		// Lets verify the ACK or NACK
		if(SPI_VerifyResponse(ack_byte)){
			// send arguments
			args[0] = ANALOG_PIN0;

			// Send the argument
			SPI_SendData(SPI2, args, 1);

			// Recieving a dummy byte to clear RXNE flag
			SPI_RecieveData(SPI2, &dummy_read, 1);

			//Insert a little delay so that arduino convert the the Signal in ADC
			delay();

			// Sending a dummy byte
			SPI_SendData(SPI2, &dummy_write, 1);

			// Recieve the Ouput
			SPI_RecieveData(SPI2, &output, 1);

			printf("This is the Output of the Second Command of Sensor Read :  %d\n ", output);

		}

		// 3. COMMAND_LED_READ

		//wait till button is pressed
		while( ! GPIO_ReadPin(GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		command = COMMAND_LED_READ;

		// Sending the first argument the Command
		SPI_SendData(SPI2, &command, 1);

		// Recieving a dummy byte to clear RXNE flag
		SPI_RecieveData(SPI2, &dummy_read, 1);

		// Sending a dummy byte so that slave responses the ACK or NACK
		SPI_SendData(SPI2, &dummy_write, 1);

		// Recieve ACK or NAck from Arduino
		SPI_RecieveData(SPI2, &ack_byte, 1);

		// Lets verify the ACK or NACK
		if(SPI_VerifyResponse(ack_byte)){

			// send arguments
			args[0] = LED_PIN;

			// Send the argument
			SPI_SendData(SPI2, args, 1);

			// Recieving a dummy byte to clear RXNE flag
			SPI_RecieveData(SPI2, &dummy_read, 1);

			//Insert a little delay so that arduino convert the the Signal in ADC
			delay();

			// Sending a dummy byte
			SPI_SendData(SPI2, &dummy_write, 1);

			// Recieve the Ouput
			SPI_RecieveData(SPI2, &output, 1);
			printf("This is the Output of the Third Command of Read Pin:  %d\n ", output);
	}



	// 4. COMMAND_PRINT

	//wait till button is pressed
		while( ! GPIO_ReadPin(GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		command = COMMAND_PRINT;
		char user_data[] = "Hello I am Abhinav Jha";

		// Sending the first argument the Command
		SPI_SendData(SPI2, &command, 1);

		// Recieving a dummy byte to clear RXNE flag
		SPI_RecieveData(SPI2, &dummy_read, 1);

		// Sending a dummy byte so that slave responses the ACK or NACK
		SPI_SendData(SPI2, &dummy_write, 1);

		// Recieve ACK or NAck from Arduino
		SPI_RecieveData(SPI2, &ack_byte, 1);

		// Lets verify the ACK or NACK
		if(SPI_VerifyResponse(ack_byte)){
			// send arguments
			args[0] = strlen(user_data);

			// Send the argument
			SPI_SendData(SPI2, args, 1);

			// Recieving a dummy byte to clear RXNE flag
			SPI_RecieveData(SPI2, &dummy_read, 1);

			//Insert a little delay so that arduino convert the the Signal in ADC
			delay();

			// Sending a User_data byte
			SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));
			printf("Sent the Data to the Slave for print, the Data sent is %s\n", user_data);

		}

	// 5. COMMAND_ID_READ

	//wait till button is pressed
			while( ! GPIO_ReadPin(GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();


		command = COMMAND_ID_READ;
		uint8_t id[11];
		uint32_t i=0;
		// Sending the first argument the Command
		SPI_SendData(SPI2, &command, 1);

		// Recieving a dummy byte to clear RXNE flag
		SPI_RecieveData(SPI2, &dummy_read, 1);

		// Sending a dummy byte so that slave responses the ACK or NACK
		SPI_SendData(SPI2, &dummy_write, 1);

		// Recieve ACK or NAck from Arduino
		SPI_RecieveData(SPI2, &ack_byte, 1);

		// Lets verify the ACK or NACK
		if(SPI_VerifyResponse(ack_byte)){
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI2,&dummy_write,1);
				// recieve the Data one by one
				SPI_RecieveData(SPI2,&id[i],1);

				//Insert a little delay so that arduino convert the the Signal in ADC
				delay();
			}

			id[11] = '\0';
			printf("This is the Output of the Fouth Command to give Arduino ID:  %s\n ", id);

		}
		//lets confirm SPI is not busy

		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_PCtrl(SPI2,DISABLE);

	}

	return 0;

}

