/*
 * 001LED_Toggle.c
 *
 *  Created on: Sep 25, 2020
 *      Author: jhaab
 */
#include "stm32f407xx.h"



void GPIO_LED_Init(uint8_t GPIO_PIN){
	GPIO_HANDLE_t GPIOLED;
	GPIOLED.pGPIOx = GPIOD;
	GPIOLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN;
	GPIOLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

//	GPIO_PClkCtrl(GPIOD, ENABLE);

	GPIO_Init(&GPIOLED);


}
void GPIO_BTN_Init(uint8_t GPIO_PIN){
	GPIO_HANDLE_t GPIOBTN;
	GPIOBTN.pGPIOx = GPIOA;
	GPIOBTN.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN;
	GPIOBTN.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBTN.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBTN.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

//	GPIO_PClkCtrl(GPIOA, ENABLE);

	GPIO_Init(&GPIOBTN);


}
int main(){

	GPIO_LED_Init(GPIO_PIN_12);
	GPIO_LED_Init(GPIO_PIN_13);
	GPIO_LED_Init(GPIO_PIN_14);
	GPIO_LED_Init(GPIO_PIN_15);
	GPIO_BTN_Init(GPIO_PIN_0);
	while(1){

		if(GPIO_ReadPin(GPIOA,  GPIO_PIN_0)){
			Delay_ms(250);
			GPIO_ToggleLEDPin(GPIOD, LED_PIN_12|LED_PIN_15);
		}


	}
	return 0;
}
