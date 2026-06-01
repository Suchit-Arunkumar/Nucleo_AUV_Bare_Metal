#include "stm32f446xx.h"

#define LED_PIN (1 << 5)

int main(void){

	RCC->AHB1ENR |= (1 << 0); 		// enable GPIOA clock
	GPIOA->MODER &= ~(3 << 10);		//clear bits 11:10 for PA5
	GPIOA->MODER |=  (1 << 10);		// set PA5 as output (01)

	while(1){

		GPIOA->BSRR = LED_PIN;				// set PA5 high
		for (int i = 0; i < 1000000; i++){
		}

		GPIOA->BSRR =(1 << 21);				// set PA5 low
		for (int i = 0; i < 1000000; i++){
				}

}

}
