.syntax unified
.thumb


.global test_0
.global test_1


.section .text.test_0
.type test_0, %function
test_0:
	push {r0-r3, lr}		// save r0-r3 and lr temporarily
	bl enable_GPIO			// call enable_GPIO, r0-r2 destroyed
	ldmia sp, {r0, r1}		// restore r0 and r1 (port, pin)
	bl reset_GPIO			// call reset_GPIO, r0-r3 destoyed
	pop {r0-r3, lr}			// restore r0-r3 and lr
	mov ip, sp				// save argument stack to ip
	push {r4-r10}			// save non volatile registers
	ldmia ip, {r4-r6}		// load args (type, speed, AF)
	mov ip, r0				// load port ptr to free r0GPIO
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
.size test_0, .-test_0

.section .text.test_1
.type test_1, %function
test_1:
	push {r0-r3, lr}		// save R0-R3 and LR temporarily
	bl enable_GPIO			// call enable_GPIO, R0-R2 destroyed
	ldmia sp, {r0, r1}		// restore R0 and R1 (port, pin)
	bl reset_GPIO			// call reset_GPIO, R0-R3 destoyed
	pop {r0-r3, lr}			// restore R0-R3 and LR
	push {r4-r7}			// save non volatile registers
	mov ip, r0				// load port ptr to free R0
	ldmia ip, {r4-r7}		// load registers
	ubfx r0, r2, #6, #1		// load 'type' setting into R0
	lsl r0, r0, r1			// shift 'type' setting according to the pin number
	orr r5, r5, r0			// write 'type' setting
	lsl r1, r1, #1			// R1(pin) *= 2 to index 2bit mapped registers
	ubfx r0, r2, #0, #2		// load 'mode' setting
	lsl r0, r0, r1			// shift 'mode' setting according to the pin number
	orr r4, r4, r0			// write 'mode' setting
	ubfx r0, r2, #2, #2		// load 'pull' setting
	lsl r0, r0, r1			// shift 'pull' setting according to the pin number
	orr r7, r7, r0			// write 'pull' setting
	ubfx r0, r2, #4, #2		// load 'speed' setting
	lsl r0, r0, r1			// shift 'speed' setting according to the pin number
	orr r6, r6, r0			// write 'speed' setting
	stm ip, {r4-r7}			// store registers
	lsl r0, r1, #1			// R0 = R1(pin) * 2 to index 4bit mapped registers
	cmp r1, #0x10			// check if AF setting for pin resides in AF_L or AF_H
	itee lo					// start IT block based on the carry flag
	addcc ip, #0x20			// set ip to AFR[0] if 'pin' < 8
	addcs ip, #0x24			// set ip to AFR[1] if 'pin' >= 8
	subcs r0, #0x20			// subtract 0x20 if 'pin' >= 8 to get the offset in AFR[1]
	ldr r2, [ip]			// load AF register
	lsl r3, r3, r0			// shift the 'AF' setting according to the pin number
	orr r2, r2, r3			// write the 'AF' setting
	str r2, [ip]			// store AF register
	pop {r4-r7}				// restore non volatile registers
	bx lr
.size test_1, .-test_1
