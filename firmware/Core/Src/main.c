#include "stm32f446xx.h"

#define LED_PIN (1 << 5)

int main(void){

	RCC->AHB1ENR |= (1 << 0); 		// clk access
	GPIOA->MODER &= ~(3 << 10);		// op mode gpio
	GPIOA->MODER |=  (1 << 10);

	while(1){

		GPIOA->BSRR = LED_PIN;
		for (int i = 0; i < 1000000; i++){
		}

		GPIOA->BSRR =(1 << 21);
		for (int i = 0; i < 1000000; i++){
				}

}

}
