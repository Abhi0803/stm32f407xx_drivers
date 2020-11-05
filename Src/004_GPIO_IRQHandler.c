/*
 * 004IRQHandler.c
 *
 *  Created on: Sep 26, 2020
 *      Author: jhaab
 */
#include <string.h>
#include "stm32f407xx.h"

#define HIGH 						(1)
#define LOW 						(0)
#define BTN_PRESSED					(LOW)

void GPIO_LED_Init(uint8_t GPIO_PIN){
	GPIO_HANDLE_t GPIOLED;
	memset(&GPIOLED, 0, sizeof(GPIOLED));
	GPIOLED.pGPIOx = GPIOD;
	GPIOLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN;
	GPIOLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PClkCtrl(GPIOD, ENABLE);

	GPIO_Init(&GPIOLED);


}
void GPIO_BTN_Init(uint8_t GPIO_PIN){
	GPIO_HANDLE_t GPIOBTN;
	memset(&GPIOBTN, 0, sizeof(GPIOBTN));
	GPIOBTN.pGPIOx = GPIOD;
	GPIOBTN.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN;
	GPIOBTN.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_FE;
	GPIOBTN.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBTN.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PClkCtrl(GPIOD, ENABLE);

	GPIO_Init(&GPIOBTN);


}

int main(){

	GPIO_LED_Init(GPIO_PIN_12);
	GPIO_LED_Init(GPIO_PIN_13);
	GPIO_LED_Init(GPIO_PIN_14);
	GPIO_LED_Init(GPIO_PIN_15);
	GPIO_BTN_Init(GPIO_PIN_6);
	GPIO_IRQITConfig(IRQ_NO_EXTI9_5,ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	while(1);
}


void EXTI9_5_IRQHandler(void){
	GPIO_IRQHandling( GPIO_PIN_6 );
	Delay(250);
	GPIO_ToggleLEDPin( GPIOD, LED_PIN_12 | LED_PIN_13 | LED_PIN_14 | LED_PIN_15 );
}
