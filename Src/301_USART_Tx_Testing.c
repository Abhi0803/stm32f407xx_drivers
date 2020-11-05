/*
 * 301_USART_Tx_Testing.c
 *
 *  Created on: Oct 12, 2020
 *      Author: jhaab
 */

#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

USART_HANDLE_t USART2Handle;

/*
 * USART2 Pin
 * USART 2 Rx ----> PA3
 * USART 2 Tx ----> PA2
 *
 * ALT FN MODE --> AF7
 */

//rcv buffer
char Tx_buf[1024]  = "This is abhinav \n";

void USART2_GPIOInits(void)
{
	GPIO_HANDLE_t USARTPins;

	USARTPins.pGPIOx = GPIOA;
	USARTPins.GPIO_PinConfig.GPIO_PinMode 			= GPIO_MODE_ALTFN;
	USARTPins.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OP_TYPE_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PIN_PU;
	USARTPins.GPIO_PinConfig.GPIO_PinAltFunMode 	= 7;
	USARTPins. GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;

	//scl
	USARTPins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_2;
	GPIO_Init(&USARTPins);


	//sda
	USARTPins.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_3;
	GPIO_Init(&USARTPins);


}

void USART2_Inits(void)
{
	USART2Handle.pUSARTx 							 = USART2;
	USART2Handle.USART_Config.USART_Buad			 = USART_STD_BAUD_115200;
	USART2Handle.USART_Config.USART_HWFlowCtrl 		 = USART_HW_FLOW_CTRL_NONE;
	USART2Handle.USART_Config.USART_Mode 		 	 = USART_MODE_ONLY_TX;
	USART2Handle.USART_Config.USART_PARITY 		 	 = USART_PARITY_DISABLE;
	USART2Handle.USART_Config.USART_STOPBits 		 = USART_STOPBITS_1;
	USART2Handle.USART_Config.USART_WORDLEN 		 = USART_WORDLEN_8BITS;


	USART_Init(&USART2Handle);

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

	USART2_GPIOInits();

	USART2_Inits();

	USART_PCtrl(USART2,ENABLE);


	while(1){

		//wait till button is pressed
		while( ! GPIO_ReadPin(GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		USART_SendData(&USART2Handle, (uint8_t *)Tx_buf, strlen(Tx_buf));
	}


 return 0;


}

///*
// * uart_tx.c
// *
// *  Created on: Jan 22, 2019
// *      Author: admin
// */
//
//#include<stdio.h>
//#include<string.h>
//#include "stm32f407xx.h"
//
//char msg[1024] = "This is Abhinav...\n\r";
//
//USART_HANDLE_t usart2_handle;
//
//void USART2_Init(void)
//{
//	usart2_handle.pUSARTx = USART2;
//	usart2_handle.USART_Config.USART_Buad = USART_STD_BAUD_115200;
//	usart2_handle.USART_Config.USART_HWFlowCtrl = USART_HW_FLOW_CTRL_NONE;
//	usart2_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
//	usart2_handle.USART_Config.USART_STOPBits = USART_STOPBITS_1;
//	usart2_handle.USART_Config.USART_WORDLEN = USART_WORDLEN_8BITS;
//	usart2_handle.USART_Config.USART_PARITY = USART_PARITY_DISABLE;
//	USART_Init(&usart2_handle);
//}
//
//void 	USART2_GPIOInit(void)
//{
//	GPIO_HANDLE_t usart_gpios;
//
//	usart_gpios.pGPIOx = GPIOA;
//	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
//	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
//	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
//	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode =7;
//
//	//USART2 TX
//	usart_gpios.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_2;
//	GPIO_Init(&usart_gpios);
//
//	//USART2 RX
//	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
//	GPIO_Init(&usart_gpios);
//
//
//}
//
//void GPIO_ButtonInit(void)
//{
//	GPIO_HANDLE_t GPIOBtn,GpioLed;
//
//	//this is btn gpio configuration
//	GPIOBtn.pGPIOx = GPIOA;
//	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
//	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
//	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//
//	GPIO_Init(&GPIOBtn);
//
//	//this is led gpio configuration
//	GpioLed.pGPIOx = GPIOD;
//	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
//	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
//	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
//	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//
//	GPIO_Init(&GpioLed);
//
//}
//
//void delay(void)
//{
//	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
//}
//
//
//int main(void)
//{
//
//	GPIO_ButtonInit();
//
//	USART2_GPIOInit();
//
//    USART2_Init();
//
//    USART_PCtrl(USART2,ENABLE);
//
//    while(1)
//    {
//		//wait till button is pressed
//		while( ! GPIO_ReadPin(GPIOA,GPIO_PIN_0) );
//
//		//to avoid button de-bouncing related issues 200ms of delay
//		delay();
//
//		USART_SendData(&usart2_handle,(uint8_t*)msg,strlen(msg));
//
//    }
//
//	return 0;
//}


