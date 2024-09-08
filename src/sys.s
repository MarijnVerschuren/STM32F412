.syntax unified
.thumb

.include "periph.h"

.global sys_reset
.global sys_tick_handler
.global sys_init


// sys_reset
.section .text.sys_reset
.type sys_reset, %function
sys_reset:
	// TODO
	bx lr
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
