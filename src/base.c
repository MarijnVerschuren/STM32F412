//
// Created by marijn on 11/26/24.
//
#include "base.h"


/*<!
 * io buffer
 * */
uint32_t in_available(io_buffer_t* buffer) {
	return buffer->i - buffer->o;
}
uint32_t out_available(io_buffer_t* buffer) {
	return buffer->o - buffer->i;
}