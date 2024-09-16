.syntax unified
.thumb

// include memory map defines with the C preprocessor
#include "memory_map.h"	// define include
.include "base.s"		// macro include


// functions
.global USART_to_int
.global enable_USART
.global disable_USART
.global fconfig_USART
.global config_USART


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

.section .text.fconfig_USART
.type fconfig_USART, %function
fconfig_USART:
	push {r2-r11, lr}
	// input switching
	cmp r1, #0
	beq fconfig_single_pin
	cmp r0, #0
	it eq
	moveq r0, r1
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
	movcc r0,	// todo APB1_freq
	movcc ip,	// todo APB1_base
	movcs r0,	// todo APB2_freq
	movcs ip,	// todo APB2_base

// uint16_t uart_div = UART_division(uart, baud) * (oversampling + 1);
// enable_USART_clock(uart);
// uart->BRR = ((uart_div & 0xfff0) | ((uart_div & 0xf) >> oversampling));
// uart->CR1 = (
// 	(USART_CR1_TE * tx_enable) |
// 	(USART_CR1_RE * rx_enable) |
// 	USART_CR1_UE
// );

fconfig_error:
	pop {r4-r11, lr}
	bx lr
.size fconfig_USART, .-fconfig_USART	/* void config_USART(USART_GPIO_t, USART_GPIO_t, uint32_t); */

.section .text.config_USART
.type config_USART, %function
config_USART:
	bx lr
.size config_USART, .-config_USART
