//
// Created by marijn on 9/21/24.
//

#ifndef STM32F412_TEST_H
#define STM32F412_TEST_H
#include "GPIO.h"


extern void test_0(GPIO_t* port, uint8_t pin, GPIO_MODE_t mode, GPIO_PULL_t pull, GPIO_OT_t output_type, GPIO_SPEED_t speed, uint8_t alternate_function);
extern void test_1(GPIO_t* port, uint8_t pin, uint32_t flags, uint8_t AF);

void test(void) {
	uint32_t i;
	uint64_t s0, s1;
	volatile uint64_t t0, t1;

	i = 0;
	s0 = tick;
	for (; i < 0x10000; i++) { test_0(GPIOA, 8, GPIO_output, GPIO_pull_up, GPIO_open_drain, GPIO_low_speed, 0xA); }
	t0 = tick - s0;

	i = 0;
	s1 = tick;
	for (; i < 0x10000; i++) { test_1(GPIOA, 8, GPIO_output | GPIO_pull_up | GPIO_open_drain | GPIO_low_speed, 0xA); }
	t1 = tick - s1;
	(void)t0; (void)t1;

	return;
}

#endif //STM32F412_TEST_H
