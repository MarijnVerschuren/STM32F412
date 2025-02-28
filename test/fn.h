//
// Created by marijn on 9/21/24.
//

#ifndef STM32F412_TEST_H
#define STM32F412_TEST_H


void* pipeline(void* in, void** funcs, uint32_t len);

void* fn_0(void* in);
void* fn_1(void* in);
void* fn_2(void* in);
void* fn_3(void* in);

static void test_fn_pipeline(void) {
	void* pipe[] = {
		fn_0,
		fn_1,
		fn_2,
		fn_3,
		fn_2,
		fn_1,
		fn_0,
		0		// replaced with return address
	};
	uint32_t test = 0;
	pipeline(&test, pipe, 7);
}

#endif //STM32F412_TEST_H
