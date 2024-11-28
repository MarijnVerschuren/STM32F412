//
// Created by marijn on 11/26/24.
//
#include "base.h"


/*<!
 * io buffer
 * */
io_buffer_t* init_io_buffer(uint32_t size, uint8_t ien, uint8_t oen, uint8_t fifo) {
	io_buffer_t* buffer = malloc(16U);
	buffer->ptr = calloc(size);
	buffer->i = 0; buffer->ien = ien;
	buffer->o = 0; buffer->oen = oen;
	buffer->size = size;
	buffer->fifo = fifo;
	return buffer;
}