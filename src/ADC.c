//
// Created by marijn on 9/25/24.
//

#include "ADC.h"



void config_ADC(uint64_t flags) {
	RCC->APB2ENR |=		0x00000100UL;			// enable ADC1 clk
	ADC1->CR2 =			0x00000001UL;			// power up ADC1
	ADC_COMMON->CCR =	(flags & 0xFFU) << 16;	// clock div, feature enable
	ADC1->CR1 =			(flags & 0xFFFFFF00U);	// resolution, mode
	ADC1->CR2 |=		(flags >> 32);			// EOC, align
}
void config_ADC_watchdog(uint32_t flags, uint16_t high_threshold, uint16_t low_threshold) {
	ADC1->CR1 |= flags;
	ADC1->HTR = high_threshold;
	ADC1->LTR = low_threshold;
}
void config_ADC_IRQ(uint8_t priority, uint32_t flags) {
	ADC1->CR1 = (ADC1->CR1 & ~0x000000E0) | flags;
	NVIC_set_IRQ_priority(ADC_IRQn, priority);
	NVIC_enable_IRQ(ADC_IRQn);
}

void config_ADC_GPIO_channel(GPIO_t* port, uint8_t pin, ADC_SAMPLE_TIME_t sample_time) {
	config_GPIO(port, pin, GPIO_analog | GPIO_no_pull);

	// TODO
}

void start_ADC(uint8_t regular, uint8_t injected) {
	ADC1->CR2 |= (
		(regular << 30)	|
		(injected << 22)
	);
}

