.syntax unified
.thumb

// include memory map defines with the C preprocessor
#include "memory_map.h"

// functions
.global GPIO_to_int
.global int_to_GPIO
.global enable_GPIO
.global disable_GPIO
.global reset_GPIO
.global fconfig_GPIO
.global aconfig_GPIO
.global config_GPIO
.global GPIO_write
.global GPIO_toggle
.global GPIO_read


/*!<
 * functions
 * */
.section .text.GPIO_to_int
.type GPIO_to_int, %function
GPIO_to_int:
	ldr r1, =GPIOA_BASE
	sub r0, r0, r1
	lsr r0, #10
	and r0, #0b111
	bx lr
.size GPIO_to_int, .-GPIO_to_int

.section .text.int_to_GPIO
.type int_to_GPIO, %function
int_to_GPIO:
	ldr r1, =GPIOA_BASE
	and r0, #0b111
	lsl r0, #10
	add r0, r0, r1
	bx lr
.size int_to_GPIO, .-int_to_GPIO

.section .text.enable_GPIO
.type enable_GPIO, %function
enable_GPIO:
	mov r2, lr
	bl GPIO_to_int
	mov r1, #1
	lsl r0, r1, r0
	ldr ip, =RCC_BASE
	ldr r1, [ip, RCC_AHB1ENR]
	orr r1, r1, r0 
	str r1, [ip, RCC_AHB1ENR]
	bx r2
.size enable_GPIO, .-enable_GPIO

.section .text.disable_GPIO
.type disable_GPIO, %function
disable_GPIO:
	mov r2, lr
	bl GPIO_to_int
	mov r1, #1
	lsl r0, r1, r0
	ldr ip, =RCC_BASE
	ldr r1, [ip, RCC_AHB1ENR]
	bic r1, r1, r0
	str r1, [ip, RCC_AHB1ENR]
	bx r2
.size disable_GPIO, .-disable_GPIO

.section .text.reset_GPIO
.type reset_GPIO, %function
reset_GPIO:
	push {r4, r5}
	mov ip, r0
	ldmia ip, {r2-r5}
	mov r0, #0b11
	lsl r0, r0, r1
	bic r3, r3, r0
	lsl r0, r0, r1
	bic r2, r2, r0
	bic r4, r4, r0
	bic r5, r5, r0
	stm ip, {r2-r5}
	pop {r4, r5}
	bx lr
.size reset_GPIO, .-reset_GPIO

.section .text.fconfig_GPIO
.type fconfig_GPIO, %function
fconfig_GPIO:
	push {r0-r3, lr}		// save r0-r3 and lr temporarily
	bl enable_GPIO			// call enable_GPIO, r0-r2 destroyed
	ldmia sp, {r0, r1}		// restore r0 and r1 (port, pin)
	bl reset_GPIO			// call reset_GPIO, r0-r3 destoyed
	pop {r0-r3, lr}			// restore r0-r3 and lr
	mov ip, sp				// save argument stack to ip
	push {r4-r10}			// save non volatile registers
	ldmia ip, {r4-r6}		// load args (type, speed, AF)
	mov ip, r0				// load port ptr to free r0
	ldmia ip, {r7-r10}		// load registers
	lsl r4, r4, r1			// shift the 'type' setting according to the pin number
	orr r8, r8, r4			// write the 'type' setting to the register
	lsl r0, r1, #1			// r0 = r1(pin) * 2 for indexing in 2bit mapped registers
	lsl r2, r2, r0			// shift the 'mode' setting according to the pin number
	orr r7, r7, r2			// write the 'mode' setting to the register 
	lsl r5, r5, r0			// shift the 'speed' setting according to the pin number
	orr r9, r9, r5			// write the 'speed' setting to the register
	lsl r3, r3, r0			// shift the 'pull' setting according to the pin number
	orr r10, r10, r3		// write the 'pull' setting to the register
	stm ip, {r7-r10}		// store registers
	lsl r0, r0, #1			// r0 *= 2 for indexing in 4bit mapped registers
	cmp r1, #8				// check if AF setting for pin resides in AF_L or AF_H
	itee lo					// start IT block based on the carry flag
	addcc ip, #0x20			// set ip to AFR[0] if 'pin' < 8
	addcs ip, #0x24			// set ip to AFR[1] if 'pin' >= 8
	subcs r0, #0x20			// subtract 0x20 if 'pin' >= 8 to get the offset in AFR[1]
	ldr r2, [ip]			// load AF register
	lsl r6, r6, r0			// shift the 'AF' setting according to the pin number
	orr r2, r2, r6			// write the 'AF' setting to the register
	str r2, [ip]			// store AF register
	pop {r4-r10}			// restore non volatile registers
	bx lr
.size fconfig_GPIO, .-fconfig_GPIO

.section .text.config_GPIO
.type config_GPIO, %function
config_GPIO:
	eor ip, ip, ip
	push {ip, lr}
	push {ip}
	ldr ip, [sp, 0xC]
	push {ip}
	bl fconfig_GPIO
	pop {r0-r2, lr}
	bx lr
.size config_GPIO, .-config_GPIO

.section .text.GPIO_write
.type GPIO_write, %function
GPIO_write:
	ldr r3, [r0, GPIO_BSRR]
	cmp r2, #0
	it eq
	addeq r1, #0x10
	mov r2, #1
	lsl r2, r2, r1
	orr r3, r2
	str r3, [r0, GPIO_BSRR]
	bx lr
.size GPIO_write, .-GPIO_write

.section .text.GPIO_toggle
.type GPIO_toggle, %function
GPIO_toggle:
	ldr r2, [r0, GPIO_ODR]
	mov r3, #1
	lsl r3, r3, r1
	eor r2, r2, r3
	str r2, [r0, GPIO_ODR]
	bx lr
.size GPIO_toggle, .-GPIO_toggle

.section .text.GPIO_read
.type GPIO_read, %function
GPIO_read:
	ldr r2, [r0, GPIO_IDR]
	lsr r0, r0, r1
	bx lr
.size GPIO_read, .-GPIO_read
