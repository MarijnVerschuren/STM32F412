.syntax unified
.thumb

// include memory map defines with the C preprocessor
#include "memory_map.h"


.global sys_reset
.global sys_tick_handler
.global sys_init


.section .data
.type tick, &object
tick: .word 0x0000000000000000 
.size tick, .-tick


// sys_reset
.section .text.sys_reset
.type sys_reset, %function
sys_reset:
	ldr r0, =RCC_BASE			// save RCC BASE
	ldr r1, =FLASH_BASE			// save FLASH BASE
	ldr r2, =SCB_BASE			// save SCB_BASE

	// enable HSI
	mov r3, #0x00000081			// HSI enable, HSI_trim: 0x10
	strb r3, [r0, RCC_CR]		// save CR[0:8]
	// reset RCC_CFGR (select HSI as sys clk)
	mov r4, #0x00000000			// zero
	str r4, [r0, RCC_CFGR]		// reset CFGR
	// reset RCC_CR
	str r3, [r0, RCC_CR]		// clear CR[8:32]
	
	// reset FLASH_ACR
	mov r3, #0x00				// default FLASH latency at 16MHz
	strb r3, [r1, FLASH_ACR]	// save ACR[0:8]
	
	// FPU settings
	mov r3, #0x00F00000			// CP10 and CP11 mask
	ldr r4, [r2, SCB_CPACR]		// load CPACR
	orr r4, r3					// or value with mask
	str r4, [r2, SCB_CPACR]		// store CPACR

	bx lr						// return
.size sys_reset, .-sys_reset


// sys_tick_handler
.section .text.sys_tick_handler
.type sys_tick_handler, %function
sys_tick_handler:
	// TODO
	bx lr
.size sys_tick_handler, .-sys_tick_handler


// sys_init
.section .text.sys_init
.type sys_init, %function
sys_init:
	// TODO
	bx lr
.size sys_init, .-sys_init
