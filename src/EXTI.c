//
// Created by marijn on 9/19/24.
//

#include "EXTI.h"


/*!< static */
static inline uint32_t EXTI_line_to_IRQn(uint8_t EXTI_line) {
	if (EXTI_line < 5)	{ return EXTI_line + EXTI0_IRQn; }
	if (EXTI_line < 10)	{ return EXTI9_5_IRQn; }
	if (EXTI_line < 16)	{ return EXTI15_10_IRQn; }
	switch (EXTI_line) {
	case 16: return PVD_IRQn;
	case 17: return RTC_Alarm_IRQn;
	case 18: return OTG_FS_WKUP_IRQn;
	case 21: return TAMP_STAMP_IRQn;
	case 22: return RTC_WKUP_IRQn;
	}
}


void config_GPIO_EXTI(uint8_t EXTI_line, GPIO_t* EXTI_port, uint8_t falling_edge, uint8_t rising_edge) {
	RCC->APB2ENR |= 0x00004000;  // enable SYSCFG
	uint8_t pos = (EXTI_line & 0x3u);  // index in the register [0:3]
	SYSCFG->EXTICR[EXTI_line >> 2u] &= ~(0xfu << (pos << 2));  // clear EXTI port in the register
	SYSCFG->EXTICR[EXTI_line >> 2u] |= GPIO_to_int(EXTI_port) << (pos << 2);  // set EXTI port in the register
	config_EXTI(EXTI_line, falling_edge, rising_edge);
}

void start_EXTI(uint8_t EXTI_line) {
	uint32_t irqn = EXTI_line_to_IRQn(EXTI_line);
	NVIC->ISER[((irqn) >> 5UL)] = (uint32_t)(1UL << ((irqn) & 0x1FUL));
}

void stop_EXTI(uint8_t EXTI_line) {
	uint32_t irqn = EXTI_line_to_IRQn(EXTI_line);
	NVIC->ICER[((irqn) >> 5UL)] = (uint32_t)(1UL << ((irqn) & 0x1FUL));
	//__DSB(); __ISB();  // flush processor pipeline before fetching
}