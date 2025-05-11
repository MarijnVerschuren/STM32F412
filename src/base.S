.syntax unified
.thumb

// include memory map defines with the C preprocessor
#include "memory_map.h"	// define include

// functions
.global init_io_buffer
.global id_to_dev
.global dev_to_id
.global enable_id
.global disable_id
.global enable_dev
.global disable_dev
.global in_available
.global out_available


/*!<
 * functions
 * */
// io_buffer_t* init_io_buffer(uint32_t size, uint8_t flags);
.section .text.init_io_buffer
.type init_io_buffer, %function
init_io_buffer:
	push {r4-r6, lr}			// push non-volatile registers
	mov r4, r0					// r4: size
	mov r5, r1					// r5: flags
	bl calloc					// allocate buffer
	mov r6, r0					// r6: ptr
	mov r0, #16					// sizeof(io_buffer_t)
	bl calloc					// allocate buffer
	str r6, [r0]				// buffer->ptr
	lsl r1, r5, #31				// shift to ien flag
	lsl r2, r5, #30				// shift to oen flag
	lsl r3, r5, #29				// shift to fifo flag
	and r1, r1, #0x80000000		// ien flag
	and r2, r2, #0x80000000		// oen flag
	and r3, r3, #0x80000000		// fifo flag
	orr r3, r3, r4				// add size
	add r4, r0, #4				// add offset
	stm r4, {r1-r3}				// store data
	pop {r4-r6, lr}				// restore non-volatile registers
	bx lr						// return
.size init_io_buffer, .-init_io_buffer


// void* id_to_dev(uint32_t id);
.section .text.id_to_dev
.type id_to_dev, %function
id_to_dev:
	ubfx r1, r0, #0, #4			// clock
	ubfx r0, r0, #4, #8			// periph
	lsl r0, r0, #10				// periph <<= 10
	add r0, r0, #0x40000000 	// += APB1_PERIPH_BASE
	cmp r1, #5					// clock == DEV_CLOCK_APB2
	it eq
	addeq r0, r0, #0x00010000	// += APB2_PERIPH_BASE (offset)
	cmp r1, #0					// clock == DEV_CLOCK_AHB1
	it eq
	addeq r0, r0, #0x00020000	// += AHB1_PERIPH_BASE (offset)
	cmp r1, #1					// clock == DEV_CLOCK_AHB2
	it eq
	addeq r0, r0, #0x10000000	// += AHB2_PERIPH_BASE (offset)
	bx lr						// return
.size id_to_dev, .-id_to_dev


// uint32_t dev_to_id(void* dev);
.section .text.dev_to_id
.type dev_to_id, %function
dev_to_id:
	lsr r2, r0, #10				// periph number
	and r2, r2, #0x3F			// mask periph number
	sub r0, r0, #0x40000000		// subtract PERIPH_BASE
	mov r1, #4					// clock number (APB1)
	cmp r0, #0x00010000			// compare with APB2 offset
	bcc dev_to_id_end			// create ID if < APB2 offset
	mov r1, #5					// clock number (ABP2)
	cmp r0, #0x00020000			// compare with AHB1 offset
	bcc dev_to_id_end			// create ID if < AHB1 offset
	mov r1, #0					// clock number (AHB1)
	cmp r0, #0x10000000			// compare with ABH2 offset
	bcc dev_to_id_end			// create ID if < AHB2 offset
	mov r1, #1					// clock number (AHB2)
dev_to_id_end:
	mov r0, r1					// write clock number [0:3]
	lsl r1, r2, #4				// shift periph number
	orr r0, r0, r1				// write periph number [4:11]
	bx lr						// return
.size dev_to_id, .-dev_to_id


// void enable_id(uint32_t id);
.section .text.enable_id
.type enable_id, %function
enable_id:
	ubfx r1, r0, #0, #4			// clock
	ubfx r0, r0, #4, #8			// periph
	mov r2, #1					// mask
	lsl r0, r2, r0				// mask <<= periph
	ldr r2, =RCC_BASE			// load RCC periph pointer
	add r2, r2, #0x30			// RCC->AHB1ENR
	lsl r1, r1, #2				// clock *= 4
	add r2, r2, r1				// add clock register offset
	ldr r1, [r2]				// load register
	orr r1, r1, r0				// |= mask
	str r1, [r2]				// store register
	bx lr						// return
.size enable_id, .-enable_id


// void disable_id(uint32_t id);
.section .text.disable_id
.type disable_id, %function
disable_id:
	ubfx r1, r0, #0, #4			// clock
	ubfx r0, r0, #4, #8			// periph
	mov r2, #1					// mask
	lsl r0, r2, r0				// mask <<= periph
	ldr r2, =RCC_BASE			// load RCC periph pointer
	add r2, r2, #0x10			// RCC->AHB1RSTR
	lsl r1, r1, #2				// clock *= 4
	add r2, r2, r1				// add clock register offset
	ldr r1, [r2]				// load register
	orr r1, r1, r0				// |= mask
	str r1, [r2]				// store register
	bx lr						// return
.size disable_id, .-disable_id


// void enable_dev(void* dev);
.section .text.enable_dev
.type enable_dev, %function
enable_dev:
	mov ip, lr
	bl dev_to_id
	bl enable_id
	bx ip
.size enable_dev, .-enable_dev


// void disable_dev(void* dev);
.section .text.disable_dev
.type disable_dev, %function
disable_dev:
	mov ip, lr
	bl dev_to_id
	bl disable_id
	bx ip
.size disable_dev, .-disable_dev


// uint32_t in_available(io_buffer_t* buffer);
.section .text.in_available
.type in_available, %function
in_available:
	add r0, r0, #4		// skip buffer pointer
	ldmia r0, {r1, r2}	// load I and O offsets
	bfc r1, #31, #1		// remove mode bit
	bfc r2, #31, #1		// remove mode bit
	sub r0, r1, r2		// calculate I - O
	bx lr
.size in_available, .-in_available


// uint32_t out_available(io_buffer_t* buffer);
.section .text.out_available
.type out_available, %function
out_available:
	add r0, r0, #4		// skip buffer pointer
	ldmia r0, {r1, r2}	// load I and O offsets
	bfc r1, #31, #1		// remove mode bit
	bfc r2, #31, #1		// remove mode bit
	sub r0, r2, r1		// calculate O - I
	bx lr
.size out_available, .-out_available


