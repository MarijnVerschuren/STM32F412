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
	//enable_GPIO
	//reset_GPIO
.size fconfig_GPIO, .-fconfig_GPIO

.section .text.config_GPIO
.type config_GPIO, %function
config_GPIO:
.size config_GPIO, .-config_GPIO

.section .text.GPIO_write
.type GPIO_write, %function
GPIO_write:
.size GPIO_write, .-GPIO_write

.section .text.GPIO_toggle
.type GPIO_toggle, %function
GPIO_toggle:
.size GPIO_toggle, .-GPIO_toggle

.section .text.GPIO_read
.type GPIO_read, %function
GPIO_read:
.size GPIO_read, .-GPIO_read
