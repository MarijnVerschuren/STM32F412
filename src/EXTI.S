.syntax unified
.thumb

// include memory map defines with the C preprocessor
#include "memory_map.h"	// define include

// functions
.global config_EXTI 



/*!<
 * functions
 * */
//void config_EXTI(uint8_t, uint8_t, uint8_t);
.section .text.config_EXTI
.type config_EXTI, %function
config_EXTI:
	ldr ip, =EXTI_BASE			// add EXTI_BASE
	add ip, ip, EXTI_RTSR		// add RTSR offset
	lsl r1, r1, r0				// shift faling edge trigger setting
	lsl r0, r2, r0				// shift rising edge trigger setting
	ldmia ip, {r2, r3}			// load RTSR and FTSR
	orr r2, r2, r0				// set rising edge trigger setting
	orr r3, r3, r1				// set falling edge trigger setting
	stm ip, {r2, r3}			// store RTSR and FTSR
	orr r0, r0, r1				// create enable mask from both triggers
	ldr r1, [ip, -EXTI_RTSR]	// load IMR
	orr r1, r1, r0				// set interrupt enable bit
	str r1, [ip, -EXTI_RTSR]	// store IMR
	bx lr						// return
.size config_EXTI, .-config_EXTI


// void config_GPIO_EXTI(uint8_t, GPIO_t*, uint8_t, uint8_t);
.section .text.config_GPIO_EXTI
.type config_GPIO_EXTI, %function
config_GPIO_EXTI:
	push {r0, r2, r3, lr}		// push arguments and LR to the stack
	bkpt #1		// TODO
	ldr ip, =RCC_BASE			// load RCC_BASE
	mov r2, #0x00004000			// load SYSCFG enable mask
	ldr r3, [ip, RCC_APB2ENR]	// load APB2ENR
	orr r3, r3, r2				// add mask
	str r3, [ip, RCC_APB2ENR]	// store APB2ENR
	and r2, r0, #0b11			// bit	index
	lsl r2, r2, #2				// bit	index
	lsr r3, r0, #2				// byte	index
	mov r0, r1					// move port (GPIO_t*) argument into R0
	bl GPIO_to_int				// port (int)
	lsl r0, r0, r2				// EXTICR setting mask
	mov r1, #0xF				// EXTICR reset mask
	lsl r1, r1, r2				// EXTICR reset mask
	ldr ip, =SYSCFG_BASE		// load SYSCFG_BASE
	add ip, SYSCFG_EXTICR		// add EXTICR offset
	ldr r2, [ip, r3]			// load EXTICR register
	bic r2, r2, r1				// clear current setting
	orr r2, r2, r0				// set setting
	str r2, [ip, r3]			// store EXTICR register
	pop {r0-r2}					// restore arguments
	bkpt #1		// TODO
	bl config_EXTI				// call config_EXTI
	pop {lr}					// restore LR
	bx lr						// return
.size config_GPIO_EXTI, .-config_GPIO_EXTI





