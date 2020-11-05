/*
 * 101SPI_Tx.c
 *
 *  Created on: Sep 29, 2020
 *      Author: jhaab
 */

/*
 *	SPI2 in AF5 MODE(Alternate Function)
 *
 *		PB12 	- 	SPI2_NSS
 *		PB13 	- 	SPI2_SCLK
 * 		PB14	- 	SPI2_MISO
 * 		PB15	- 	SPI2_MOSI
 */
#include <string.h>
#include "stm32f407xx.h"

void SPI2_GPIO_Init(){
	GPIO_HANDLE_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALT__AF5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SPI2_NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	//GPIO_Init(&SPIPins);
	// SPI2_SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);
	// SPI2_MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
//	GPIO_Init(&SPIPins);
	// SPI2_MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);
}


void SPI2_Init(){

	SPI_HANDLE_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_FD;
	SPI2handle.SPI_Config.SPI_DeviceMode = SPI_MASTER_Mode;
	SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SPEED_DIV2;
	SPI2handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2handle);
}


int main() {
		char UserData[] = "Hello World";

	// This Function Initializes the GPIO Pins Of the SPI2 Peripheral
	// Later We can Make this Code to Initialize any SPIx
		SPI2_GPIO_Init();

	// This is the Peripheral Configuration
		SPI2_Init();

	// This is SPI SSI Bit Config which makes NSS Pin High and Avoid MODF Fault
		SPI_SSI_Config(SPI2, ENABLE);

	// Enable The SPI Peripheral
		SPI_PCtrl(SPI2, ENABLE);

	// Sending the Data to SPI2
		SPI_SendData(SPI2, (uint8_t *)UserData, strlen(UserData));

	// Checking Whether the SPI is Busy or Not
		while(SPI_GetFlagStatus(SPI2, SPI_SR_BSY));

	// Enable The SPI Peripheral
		SPI_PCtrl(SPI2, DISABLE);

	while(1);
	return 0;
}

