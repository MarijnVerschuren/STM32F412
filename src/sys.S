.syntax unified
.thumb

// include memory map defines with the C preprocessor
#include "memory_map.h"


.global tick
.global sys_config

.global sys_reset
.global sys_tick_handler


/*!<
 * variables
 * */
.section .data.tick
.type tick, %object				/* uint64_t tick; */
tick: .quad 0x0000000000000000
.size tick, .-tick

.section .data.sys_config
.type sys_config, %object
sys_config:
.word 0x00000000
.word 0x00000000
.word 0x00000000
.word 0x00000000
.size sys_config, .-sys_config


/*!<
 * functions
 * */
.section .text.sys_reset
.type sys_reset, %function		/* void sys_reset(void); */
sys_reset:
	ldr r0, =RCC_BASE			// save RCC BASE
	ldr r1, =FLASH_BASE			// save FLASH BASE
	ldr r2, =SCB_BASE			// save SCB_BASE
	ldr r3, =PWR_BASE			// save PWR_BASE

	// RCC (enable HSI, set HSI as sys_clk, reset other config)
	mov r4, #0x00000081			// HSI enable, HSI_trim: 0x10
	strb r4, [r0, RCC_CR]		// save CR[0:8]
	// reset RCC_CFGR (select HSI as sys clk)
	mov r5, #0x00000000			// zero
	str r5, [r0, RCC_CFGR]		// reset CFGR
	// reset RCC_CR
	str r4, [r0, RCC_CR]		// clear CR[8:32]
	// reset RCC_CIR
	str r5, [r0, RCC_CIR]		// clear CIR
	
	// FLASH (update flash latency for 16MHz)
	mov r4, #0x00				// default FLASH latency at 16MHz
	strb r4, [r1, FLASH_ACR]	// save ACR[0:8]
	
	// SCB (enable FPU)
	mov r4, #0x00F00000			// CP10 and CP11 mask
	ldr r5, [r2, SCB_CPACR]		// load CPACR
	orr r5, r4					// or value with mask
	str r5, [r2, SCB_CPACR]		// store CPACR

	// PWR (update VOS scale for 16MHz)
	mov r4, #0x00004000			// VOS scale 3 setting (mininum nominal)
	str r4, [r3, PWR_CR]		// save CR

	bx lr						// return
.size sys_reset, .-sys_reset


.section .text.sys_tick_handler
.type sys_tick_handler, %function	/* void sys_tick_handler(void); */
sys_tick_handler:
	push {r0, r1, r2}			// push used registers to the stack
	ldr r2, =tick				// load tick ptr 
	ldmia r2, {r0-r1}			// load multiple src  (2 * 32-bit)
	adds r0, r0, #1				// inc low word
	adc r1, r1, #0				// add carry to high word
	stm r2, {r0-r1}				// store multiple dst (2 * 32-bit)
	pop {r0, r1, r2}			// pop saved registers from the stack
	bx lr						// return
.size sys_tick_handler, .-sys_tick_handler

