.syntax unified
.thumb

// include memory map defines with the C preprocessor
#include "memory_map.h"	// define include

// functions
.global enable_IRQ
.global disable_IRQ
.global set_IRQ_priority



/*!<
 * functions
 * */
//void enable_IRQ(IRQn_t);
.section .text.enable_IRQ
.type enable_IRQ, %function
enable_IRQ:
	lsr r1, r0, #5				// byte index in the ISER register
	and r0, r0, #0x1F			// bit	index in the ISER register
	mov r2, #1					// load bit
	lsl r0, r2, r0				// shift bit
	ldr ip, =NVIC_BASE			// load NVIC base into IP
	str r0, [ip, NVIC_ISER]		// set IRQ enable bit
	bx lr						// return
.size enable_IRQ, .-enable_IRQ


//void disable_IRQ(IRQn_t);
.section .text.disable_IRQ
.type disable_IRQ, %function
disable_IRQ:
	lsr r1, r0, #5				// byte index in the ICER register
	and r0, r0, #0x1F			// bit	index in the ICER register
	mov r2, #1					// load bit
	lsl r0, r2, r0				// shift bit
	ldr ip, =NVIC_BASE			// load NVIC base into IP
	str r0, [ip, NVIC_ICER]		// set IRQ enable bit
	dsb	sy						// wait until write complete
	isb	sy						// wait until the CPU pipeline is clear
	bx lr						// return
.size disable_IRQ, .-disable_IRQ


// void set_IRQ_priority(IRQn_t, uint8_t);
.section .text.set_IRQ_priority
.type set_IRQ_priority, %function
set_IRQ_priority:
	ldr ip, =NVIC_BASE			// load NVIC base into IP
	add ip, ip, NVIC_IP			// add IP register offset
	lsl r1, r1, #4				// shift priority
	strb r1, [ip, r0]			// store priority in the IP register
	bx lr						// return
.size set_IRQ_priority, .-set_IRQ_priority



