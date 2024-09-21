.syntax unified
.thumb

// include memory map defines with the C preprocessor
#include "memory_map.h"	// define include
.include "base.s"		// macro include

// externals
.global APB1_clock_frequency
.global APB2_clock_frequency

// functions
.global enable_USART
.global disable_USART
.global reset_USART
.global fconfig_UART
.global config_UART
// TODO: USART?


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
 // void enable_USART(USART_t*);
.section .text.enable_USART
.type enable_USART, %function
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


// void disable_USART(USART_t*);
.section .text.disable_USART
.type disable_USART, %function
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
.size disable_USART, .-disable_USART


// void reset_USART(USART_t*);
.section .text.reset_USART
.type reset_USART, %function
reset_USART:
	add ip, r0, =USART_BRR
	ldmia ip, {r0, r1}
	mov r0, #0x00000000
	mov r1, #0x00000000
	stm ip, {r0, r1}
	bx lr
.size reset_USART, .-reset_USART


// void fconfig_USART(USART_GPIO_t, USART_GPIO_t, uint32_t, USART_OS_t);
.section .text.fconfig_UART
.type fconfig_UART, %function
fconfig_UART:
	push {r2-r8, lr}				// TODO
	mov r8, #0x00002008				// USART, TX enable TODO: r8
	// check pin inputs
	cmp r1, #0x00000000				// RX enabled?
	beq fconfig_simplex				// configure for TX only
	orr r8, r8, #0x00000004			// RX enable
	cmp r0, #0x00000000				// TX enabled?
	itt eq
	moveq r0, r1					// move R1 to R0
	biceq r8, r8, #0x00000008		// TX disable
	beq fconfig_simplex				// configure for RX only
	push {r1}						// save RX pin
	// configure TX
	dpin_alt r3, r0					// AF
	dpin_pin r4, r0					// pin
	dpin_port r0, r0				// port (int)
	bl int_to_GPIO					// port (GPIO_t*)
	mov r1, r4						// pin
	mov r2, #0b0000010				// GPIO_AF_MODE, NO_PULL, LOW_SPEED, PUSH_PULL
	bl fconfig_GPIO					// init pin
	pop {r0}						// load RX
fconfig_simplex:
	// calculate dev ptr
	dpin_clk r1, r0					// clock
	cmp r1, #0x1					// compare clock
	ittee lo
	ldrcc r1, =APB1_clock_frequency	// &APB1_clock_frequency
	ldrcc r5, =APB1PERIPH_BASE		// usart peripheral ptr base = APB1
	ldrcs r1, =APB2_clock_frequency	// &APB2_clock_frequency
	ldrcs r5, =APB2PERIPH_BASE		// usart peripheral ptr base = APB2
	ldr r6, [r1]					// APBx_clock_frequency
	dpin_periph r1, r0				// usart (int)
	add, r5, r5, r1, lsl #10		// usart (USART_t*), R5 += usart(int) << 10
	// configure RX or TX (simplex)
	dpin_alt r3, r0					// AF
	dpin_pin r4, r0					// pin
	dpin_port r0, r0				// port (int)
	bl int_to_GPIO					// port (GPIO_t*)
	mov r1, r4						// pin
	mov r2, #0b0000010				// GPIO_AF_MODE, NO_PULL, LOW_SPEED, PUSH_PULL
	bl fconfig_GPIO					// init pin
	// config

	pop {r3, r4};
	// R3: baud
	// R4: OS
	// R5: USART
	// R6: APBx_clock_freq

	pop {r2-r8, lr}			// TODO
	bx lr
.size fconfig_UART, .-fconfig_UART


// void config_USART(USART_GPIO_t, USART_GPIO_t, uint32_t);
.section .text.config_UART
.type config_UART, %function
config_UART:
	mov r4, lr
	mov r3, #0b0
	bl fconfig_UART
	bx r4
.size config_UART, .-config_UART
