///*
// * 201_I2C_Master_Tx_Testing.c
// *
// *  Created on: Oct 8, 2020
// *      Author: jhaab
// */
//
//
//#include<stdio.h>
//#include<string.h>
//#include "stm32f407xx.h"
//
////extern void initialise_monitor_handles(void); //monitor arm semihosting enable
//
//I2C_HANDLE_t I2C1Handle;
//
//#define MY_ADDR  				0x61
//#define SLAVE_ADDR  			0x68
//
//uint8_t data[] = "This will be used for OLED Display";
//
///*
// * PB6 						--> I2C1_SCL
// * PB9  					--> I2C1_SDA
// * ALT function mode 		--> 4
// * Arduino Address(Slave) 	--> 0x68
// */
//
//void delay(void)
//{
//	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
//}
//
//void I2C1_GPIOInits(void)
//{
//	GPIO_HANDLE_t I2CPins;
//
//	I2CPins.pGPIOx = GPIOB;
//	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
//	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
//	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
//	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
//	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//
//	//I2C1_SCL
//	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
//	GPIO_Init(&I2CPins);
//
//	//I2C1_SDA
//	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
//	GPIO_Init(&I2CPins);
//}
//
//void I2C1_Inits(void)
//{
//	I2C1Handle.pI2Cx = I2C1;
//	I2C1Handle.I2C_Config.I2C_ACKControl 		= I2c_ACK_EN;
//	I2C1Handle.I2C_Config.I2C_DeviceAdress	 	= MY_ADDR;							// Check the Reserved Addresses in the manual and do not use those
//	I2C1Handle.I2C_Config.I2C_FMDutyCycle 		= I2C_FM_DUTY_2;
//	I2C1Handle.I2C_Config.I2C_SCL_Speed			= I2C_SCL_SPEED_SM;
//
//	I2C_Init(&I2C1Handle);
//}
//
//void GPIO_ButtonInit(void)
//{
//	GPIO_HANDLE_t GPIOBtn, GpioLed;
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
//}
// int main(){
//
////	 initialise_monitor_handles();
//
//	 // User button Init
//	 GPIO_ButtonInit();
//
//	 // I2C Pin Config
//	 I2C1_GPIOInits();
//
////	 printf("I2C Pin Configured\n");
//
//	 // I2C  Peripheral Config
//	 I2C1_Inits();
//
////	 printf("I2C Peripheral Configured\n");
//
//	 // Enable the I2C Peripheral
//	 I2C_PCtrl(I2C1, ENABLE);
//
////	 printf("I2C Peripheral Enabled\n");
//
//	while(1)
//	{
//		//wait till button is pressed
//		while( ! GPIO_ReadPin(GPIOA,GPIO_PIN_0) );
//
//		//to avoid button de-bouncing related issues 200ms of delay
//		delay();
//
//		//send some data to the slave
//		I2C_MasterSendData(&I2C1Handle,data,strlen((char*)data),SLAVE_ADDR, I2C_DISABLE_SR);
//	}
// }

/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Feb 24, 2019
 *      Author: admin
 */


#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

#define MY_ADDR  				0x61
#define SLAVE_ADDR  			0x68

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_HANDLE_t I2C1Handle;

//some data
//uint8_t some_data[] = "We are testing I2C master Tx\n";
uint8_t some_data[] = "Hi I am Abhinav\n";
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

	while(1)
	{
		//wait till button is pressed
		while( ! GPIO_ReadPin(GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//send some data to the slave
		I2C_MasterSendData(&I2C1Handle,some_data,strlen((char*)some_data),SLAVE_ADDR, I2C_DISABLE_SR);
	}

}
