.syntax unified
.thumb


/*!<
 * globals
 * */
// LL
.global _sbrk
.global _write
.global _close
.global _lseek
.global _read
// SYS
.global malloc
.global calloc


/*!<
 * externals
 * */
.extern _heap_start


/*!<
 * variables
 * */
.section .data.heap_end
.type heap_end, %object				/* uint32_t heap_end; */
heap_end: .word 0x00000000
.size heap_end, .-heap_end


/*!<
 * LL functions
 * */
.section .text._sbrk
.type _sbrk, %function		/* void* _sbrk(uint32_t); */
_sbrk:
	mov r2, r0				// mov alloc size to R3
	ldr ip, =heap_end		// load heap_end ptr
	ldr r1, =_heap_start	// load _heap_start ptr
	ldr r0, [ip]			// load heap_end
	cmp r0, 0				// check if _heap_start uninitialized
	it eq					
	moveq r0, r1			// set heap_end to _heap_start if uninitialized
	add r2, r0, r2			// increment heap ptr
	str r2, [ip]			// store heap_end if uninitialized
	bx lr					// return
.size _sbrk, .-_sbrk

.section .text._write
.type _write, %function		/* uint32_t _write(uint32_t file, const void* ptr, uint32_t len); */
_write:
	mov r0, r2				// "written" byte count as return value
	bx lr					// return
.size _write, .-_write

.section .text._close
.type _close, %function		/* uint32_t _close(uint32_t); */
_close:
	mov r0, #0				// return code (success)
	bx lr					// return
.size _close, .-_close

.section .text._lseek
.type _lseek, %function		/* uint32_t _lseek(uint32_t file, int32_t offset, uint32_t whence); */
_lseek:
	mov r0, #0				// return code (success)
	bx lr					// return
.size _lseek, .-_lseek

.section .text._read
.type _read, %function		/* uint32_t _read(uint32_t file, const void* ptr, uint32_t len); */
_read:
	mov r0, r2				// "read" byte count as return value
	bx lr					// return
.size _read, .-_read


/*!<
 * sys functions
 * */
.section .text.calloc
.type calloc, %function		/*!< void* calloc(uint32_t size); */
calloc:
	push {r0}				// save size
	mov r3, lr				// save link address
	bl _sbrk				// alloc size
	mov ip, r0				// copy addr to ip
	pop {r1}				// restore size
	sub r1, r1, #1
	eor r2, r2, r2			// zero r2
calloc_loop:
	strb r2, [ip]			// store 0 to ip
	add ip, ip, #1			// increment ip
	subs r1, r1, #1			// decrement size
	bcs calloc_loop
	bx r3
.size calloc, .-calloc


/*!< weak definitions */
.weak malloc
.thumb_set malloc,			_sbrk
