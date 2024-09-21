.syntax unified
.thumb

// include memory map defines with the C preprocessor
#include "memory_map.h"	// define include
.include "base.s"		// macro include

// externals
.global APB1_clock_frequency
.global APB2_clock_frequency

// functions
.global USART_to_int
.global enable_USART
.global disable_USART
.global fconfig_UART
.global config_UART


/*!< TODO: inline? or externalize?
 * local functions
 * */
.section .text.USART_to_int
.type USART_to_int, %function			/* uint8_t USART_to_int(USART_t*); */
USART_to_int:
	ldr r1, =APB2PERIPH_BASE
	cmp r0, r1
	ite lo
	subcc r0, r0, APB1PERIPH_BASE		// subtract APB1 base
	subcs r0, r0, r1					// subtract APB2 base
	lsr r0, r0, #10
	bx lr
.size USART_to_int, .-USART_to_int

/*!<
 * functions
 * */
.section .text.enable_USART
.type enable_USART, %function			/* void enable_USART(USART_t*); */
enable_USART:
	ldr ip, =RCC_BASE
	mov r3, lr
	bl USART_to_int						// sets CC accordingly
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
.type disable_USART, %function			/* void disable_USART(USART_t*); */
disable_USART:
	ldr ip, =RCC_BASE
	mov r3, lr
	bl USART_to_int						// sets CC accordingly
	ite lo
	addcc ip, RCC_APB1ENR
	addcs ip, RCC_APB2ENR
	mov r2, #1
	lsl r2, r2, r0
	ldr r1, [ip]
	bic r1, r1, r2
	str r1, [ip]
	bx r3
.size disable_USART, .-disable_USART	/* void fconfig_USART(USART_GPIO_t, USART_GPIO_t, uint32_t, USART_oversampling_t); */

.section .text.fconfig_UART
.type fconfig_UART, %function
fconfig_UART:
	push {r2-r8, lr}
	mov r8, #0x2008			// USART, TX enable
	// input switching
	cmp r1, #0
	beq fconfig_single_pin
	orr r8, r8, #0b100		// RX enable
	cmp r0, #0
	itt eq
	moveq r0, r1
	biceq r8, r8, #0b1000	// TX disable
	beq fconfig_single_pin
	push {r1}
	// check input
	dpin_clk r4, r0
	dpin_clk r5, r1
	dpin_periph r6, r0
	dpin_periph r7, r1
	cmp r4, r5
	it eq
	cmpeq r6, r7
	bne fconfig_error
	// configure the GPIO pin(s)
	dpin_alt r6, r0			// AF
	dpin_pin r4, r0			// pin
	dpin_port r0, r0		// port (int)
	bl int_to_GPIO			// port (GPIO_t*)
	mov r1, r4				// pin
	mov r2, #0b10			// mode
	mov r3, #0b00			// pull
	mov r4, #0b0			// otype
	mov r5, #0b00			// speed
	push {r4-r6}			// pass r4-r6 arguments on the stack
	bl fconfig_GPIO			// configure TX
	add sp, sp, #12			// restore stack
	pop {r0}				// load RX pin struct
fconfig_single_pin:
	mov r7, r0
	dpin_alt r6, r0			// AF
	dpin_pin r4, r0			// pin
	dpin_port r0, r0		// port (int)
	bl int_to_GPIO			// port (GPIO_t*)
	mov r1, r4				// pin
	mov r2, #0b10			// mode
	mov r3, #0b00			// pull
	mov r4, #0b0			// otype
	mov r5, #0b00			// speed
	push {r4-r6}			// pass r4-r6 arguments on the stack
	bl fconfig_GPIO			// configure
	add sp, sp, #12			// restore stack
	// config
	pop {r2, r3}
	dpin_clk r0, r7
	cmp r0, #1
	ittee lo
	ldrcc r0, =APB1_clock_frequency
	ldrcc ip, =APB1PERIPH_BASE
	ldrcs r0, =APB2_clock_frequency
	ldrcs ip, =APB2PERIPH_BASE
	ldr r1, [r0]
	
	udiv r5, r1, r2
	lsl r5, r5, r3
	dpin_periph r2, r7
	add r0, ip, r2, lsl #10
	mov r4, r0
	mov r6, r3
	bl enable_USART
	
	// R4 (uart*), R5 (uart_div), R6 (os)
	mov r0, #0xFFF0
	and r0, r5, r0
	and r5, r5, #0xF
	lsr r5, r5, r6
	orr r0, r0, r5
	str r0, [r4, USART_BRR]
	str r8, [r4, USART_CR1]
fconfig_error:
	pop {r4-r8, lr}
	bx lr
.size fconfig_UART, .-fconfig_UART	/* void config_USART(USART_GPIO_t, USART_GPIO_t, uint32_t); */

.section .text.config_UART
.type config_UART, %function
config_UART:
	mov r4, lr
	mov r3, #0b0
	bl fconfig_UART
	bx r4
.size config_UART, .-config_UART
