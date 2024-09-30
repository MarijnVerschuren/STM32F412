//
// Created by marijn on 9/25/24.
//

#include "ADC.h"



void config_ADC(uint16_t flags) {
	RCC->APB2ENR |=		0x00000100UL;	// enable ADC1 clk
	ADC1->CR2 =			0x00000001UL;	// power up ADC1
	ADC_COMMON->CCR =	(flags & 0xFFU) << 16;	// clock div, feature enable
	ADC1->CR1 =			(flags & 0xF00U) << 8;	// resolution
}

void config_injected_ADC_GPIO(GPIO_t* port, uint8_t pin, uint8_t EOC_int, uint8_t DISC, uint8_t res, uint8_t CONT, uint8_t align, uint8_t trig, uint8_t trig_mode) {
	config_GPIO(port, pin, GPIO_analog | GPIO_no_pull);
	ADC1->CR1 = (
		(EOC_int << 7)	|
		(DISC << 12)	|
		(res << 24)
	);	// TODO: watchdog, overrun
	ADC1->CR2 = (
		(0b1 << 0)			|	// ADON
		(CONT << 1)			|	// continuous mode
		(align << 11)		|	// align data
		(trig << 16)		|	// external trigger
		(trig_mode << 20)		// trigger mode
	);
	// TODO
}

void config_ADC_watchdog(uint32_t flags) { ADC1->CR1 |= flags; }
void config_ADC_IRQ(uint8_t priority) {
	NVIC_set_IRQ_priority(ADC_IRQn, priority);
	NVIC_enable_IRQ(ADC_IRQn);
}


