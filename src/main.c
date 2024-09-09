#include "types.h"
#include "periph.h"

extern volatile uint32_t tick;

#define PIN 8

// application
void main(void) {
	(void)tick;

	RCC->AHB1ENR |= (0x1 << 2) | 0x1;
	GPIOA->MODER |= 0x1 << (2*PIN);
	while (1) {
		GPIOA->BSRR = 0x1 << PIN;
		GPIOA->BSRR = 0x10000 << PIN;
	}
}
