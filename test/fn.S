.syntax unified
.thumb


.global pipeline
.global fn_0
.global fn_1
.global fn_2
.global fn_3


.section .text.pipeline
.type pipeline, %function
pipeline:
	mov ip, r1
	ldr r1, [ip]
	lsl r2, r2, 2
	str lr, [ip, r2]
	add ip, ip, #4
	mov pc, r1
.size fn_0, .-fn_0


.section .text.fn_0
.type fn_0, %function
fn_0:
	ldr lr, [ip]
	add ip, ip, #4

	ldr r1, [r0]
	eor r1, r1, #1
	str r1, [r0]

	bx lr
.size fn_0, .-fn_0

.section .text.fn_1
.type fn_1, %function
fn_1:
	ldr lr, [ip]
	add ip, ip, #4

	ldr r1, [r0]
	eor r1, r1, #2
	str r1, [r0]

	bx lr
.size fn_1, .-fn_1

.section .text.fn_2
.type fn_2, %function
fn_2:
	ldr lr, [ip]
	add ip, ip, #4

	ldr r1, [r0]
	eor r1, r1, #4
	str r1, [r0]

	bx lr
.size fn_2, .-fn_2

.section .text.fn_3
.type fn_3, %function
fn_3:
	ldr lr, [ip]
	add ip, ip, #4

	ldr r1, [r0]
	eor r1, r1, #8
	str r1, [r0]

	bx lr
.size fn_3, .-fn_3