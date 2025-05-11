//
// Created by marijn on 3/2/25.
//

#include "pwm.h"



/*!< init / enable / disable */
void config_PWM(TIM_GPIO_t pwm_pin, uint32_t prescaler, uint32_t period) {
	dev_pin_t dpin = *((dev_pin_t*)&pwm_pin);
	TIM_t* tim = id_to_dev(pwm_pin);
	uint8_t channel = dpin.misc & 0x3;
	fconfig_GPIO(int_to_GPIO(dpin.port), dpin.pin, GPIO_alt_func | GPIO_no_pull | GPIO_push_pull | GPIO_high_speed, dpin.alt);
	config_TIM(tim, prescaler, period);
	(&tim->CCMR1)[channel >> 1] &=	~(0xFFu << ((channel & 0b1u) << 3));// clear register
	tim->CCER &=					~(0b1u << (1 + (channel << 2)));	// default polarity
	(&tim->CCMR1)[channel >> 1] |=	(
		(0b110 << (4 + ((channel & 0b1u) << 3))) |						// set PWM mode
		(0b1u << (3 + ((channel & 0b1u) << 3)))							// enable preload
	);
	tim->CR1 |= 0x00000080UL;											// enable auto-reload preload
	tim->CCER |= (0b1u << (channel << 2));								// enable capture compare
	(&tim->CCR1)[channel] = 0;											// set duty cycle to 0
	tim->CR1 |= 0x00000001UL;											// start timer
}