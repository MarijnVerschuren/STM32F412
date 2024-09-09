.syntax unified
.thumb

.global IVT


.section .vector, "a", %progbits
.type IVT, %object
IVT:
.word _stack_end
.word reset_handler
.word NMI_handler
.word hard_fault_handler
.word mem_fault_handler
.word bus_fault_handler
.word exception_handler
.word 0
.word 0
.word 0
.word 0
.word SVC_handler
.word debug_handler
.word 0
.word pending_SV_handler
.word sys_tick_handler
// TODO: more interrupts


// default handler
.section .text.default_handler, "ax", %progbits
default_handler:
	bkpt #1
	b default_handler
.size default_handler, .-default_handler


// weak definitions
.weak NMI_handler
.thumb_set NMI_handler,			default_handler
.weak hard_fault_handler
.thumb_set hard_fault_handler,	default_handler
.weak mem_fault_handler
.thumb_set mem_fault_handler,	default_handler
.weak bus_fault_handler
.thumb_set bus_fault_handler,	default_handler
.weak exception_handler
.thumb_set exception_handler,	default_handler
.weak SVC_handler
.thumb_set SVC_handler,			default_handler
.weak debug_handler
.thumb_set debug_handler,		default_handler
.weak pending_SV_handler
.thumb_set pending_SV_handler,	default_handler
.weak sys_tick_handler
.thumb_set sys_tick_handler,	default_handler

