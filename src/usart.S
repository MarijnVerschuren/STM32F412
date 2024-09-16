.syntax unified
.thumb

// include memory map defines with the C preprocessor
#include "memory_map.h"

// functions
.global enable_USART
.global disable_USART


/*!<
 * local functions
 * */
.section .text.USART_to_int
.type USART_to_int, %function
USART_to_int:
	ldr r1, =APB2PERIPH_BASE
	cmp r0, r1
	ite lo
	subcc r0, r0, APB1PERIPH_BASE	// subtract APB1 base
	subcs r0, r0, r1				// subtract APB2 base
	lsr r0, r0, #10
	bx lr
.size USART_to_int, .-USART_to_int

/*!<
 * functions
 * */
.section .text.enable_USART
.type enable_USART, %function		/* void sys_reset(void); */
enable_USART:
	ldr ip, =RCC_BASE
	mov r3, lr
	bl USART_to_int					// sets CC accordingly
	ite lo
	addcc ip, RCC_APB1ENR
	addcs ip, RCC_APB2ENR
	mov r2, #1
	lsl r2, r2, r0
	ldr r1, [ip]
	orr r1, r1, r2
	str r1, [ip]
	bx r3
.size enable_USART, .-enable_USART

.section .text.disable_USART
.type disable_USART, %function		/* void sys_reset(void); */
disable_USART:
	ldr ip, =RCC_BASE
	mov r3, lr
	bl USART_to_int					// sets CC accordingly
	ite lo
	addcc ip, RCC_APB1ENR
	addcs ip, RCC_APB2ENR
	mov r2, #1
	lsl r2, r2, r0
	ldr r1, [ip]
	bic r1, r1, r2
	str r1, [ip]
	bx r3
.size disable_USART, .-disable_USART
