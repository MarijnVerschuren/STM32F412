//
// Created by marijn on 9/17/24.
//
#include "USART.h"


/*!<
 * IRQ variables
 * */  // TODO: linker alloc
io_buffer_t* USART_buffer[4];	// [3] -> USART6


/*!<
 * IRQ handler
 * */
static _FINLINE void USART_IRQ_handler(USART_t* usart, io_buffer_t* io) {
	if (!io) { return; }
	if (usart->SR & 0x00000020UL && io->ien && io->ptr) {
		((uint8_t*)io->ptr)[io->i++] = (_IO uint8_t)usart->DR;
		if (io->i == io->size) {					// this is assuming that 'i' is never changed otherwise
			if (io->fifo) { io->i = 0; return; }	// reset offset
			usart->CR1 &= ~0x00000020UL;			// turn off IRQ
		}
	}
	// TODO: other events

}

void USART1_handler() { USART_IRQ_handler(USART1, USART_buffer[0]); }
void USART2_handler() { USART_IRQ_handler(USART2, USART_buffer[1]); }
void USART3_handler() { USART_IRQ_handler(USART3, USART_buffer[2]); }
void USART6_handler() { USART_IRQ_handler(USART6, USART_buffer[3]); }


/*!<
 * irq
 * */
//void start_USART_read_irq(USART_TypeDef* usart, io_buffer_t* buffer) {
//	usart->CR1 |= USART_CR1_RXNEIE;
//	uint32_t irqn;
//	USART_IRQ_IO_t* usart_buf;
//	if (usart == USART1)		{ irqn = USART1_IRQn; usart_buf = &usart_buf_1; }
//	else if (usart == USART2)	{ irqn = USART2_IRQn; usart_buf = &usart_buf_2; }
//	else if (usart == USART6)	{ irqn = USART6_IRQn; usart_buf = &usart_buf_6; }
//	else { return; }  // error
//	usart_buf->rx_buf = buffer;	usart_buf->rx_fifo = fifo;
//	NVIC->ISER[((irqn) >> 5UL)] = (uint32_t)(1UL << ((irqn) & 0x1FUL));  // NVIC_EnableIRQ
//}
//void stop_USART_read_irq(USART_TypeDef* usart) { usart->CR1 &= ~USART_CR1_RXNEIE; }
//void disable_USART_irq(USART_TypeDef* usart) {
//	uint32_t irqn;
//	if (usart == USART1) { irqn = USART1_IRQn; }
//	else if (usart == USART2) { irqn = USART2_IRQn; }
//	else if (usart == USART6) { irqn = USART6_IRQn; }
//	else { return; }  // error
//	NVIC->ICER[((irqn) >> 5UL)] = (uint32_t)(1UL << ((irqn) & 0x1FUL));  // NVIC_DisableIRQ
//	__DSB(); __ISB();  // flush processor pipeline before fetching
//}