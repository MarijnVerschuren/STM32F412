//
// Created by marijn on 9/4/24.
//

#ifndef STM32F412_BASE_H
#define STM32F412_BASE_H


typedef char				int8_t;
typedef short				int16_t;
typedef long				int32_t;
typedef long long			int64_t;

typedef unsigned char		uint8_t;
typedef unsigned short		uint16_t;
typedef unsigned long		uint32_t;
typedef unsigned long long	uint64_t;

#define _I	volatile const
#define _O	volatile
#define _IO	volatile

#define _FINLINE __attribute__((always_inline))

#define NULL ((void*)0x00000000UL)

typedef enum {
	DEV_CLOCK_AHB1 = 0,
	DEV_CLOCK_AHB2 = 1,
	DEV_CLOCK_AHB3 = 2,
	DEV_CLOCK_APB1 = 4,
	DEV_CLOCK_APB2 = 5
} dev_clock_id_t;

typedef struct {
	uint32_t	clk		: 4;		// clock bus
	uint32_t	periph	: 8;		// peripheral
	uint32_t	misc	: 8;		// misc info (tim channel number etc...)
	uint32_t	port	: 4;		// GPIO port
	uint32_t	pin		: 4;		// GPIO pin
	uint32_t	alt		: 4;		// alternate function
} dev_pin_t;  // 32 bit

typedef struct {
	_IO	void*		ptr;			// buffer
	_IO uint32_t	i	: 31;		// write
		uint32_t	ien	: 1;		// buffer input mode
	_IO uint32_t	o	: 31;		// read
		uint32_t	oen : 1;		// buffer output mode
		uint32_t	size: 31;		// buffer size
		uint32_t	fifo: 1;		// fifo mode (ien only)
} io_buffer_t;	// 128 bit


/*<! sys */
extern _IO uint32_t heap_end;
extern void* malloc(uint32_t);
extern void* calloc(uint32_t);
/*<! io buffer */
io_buffer_t* init_io_buffer(uint32_t size, uint8_t ien, uint8_t oen, uint8_t fifo);
/*<! dev id */
extern void* id_to_dev(uint32_t id);	// only uses: clk, periph
extern uint32_t dev_to_id(void* dev);	// only writes: clk, periph
extern void enable_id(uint32_t id);
extern void disable_id(uint32_t id);
extern void enable_dev(void* dev);
extern void disable_dev(void* dev);



#endif // STM32F412_BASE_H
