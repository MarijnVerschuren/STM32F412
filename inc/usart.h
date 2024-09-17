//
// Created by marijn on 9/16/24.
//

#ifndef STM32F412_USART_H
#define STM32F412_USART_H
#include "periph.h"
#include "gpio.h"
#include "sys.h"


/*!<
 * types
 * */
typedef enum {
	USART_PIN_DISABLE =	0x00000000,
	// USART1
	USART1_CK_A8 = 0x78000041,
	USART1_TX_A9 = 0x79000041,
	USART1_RX_A10 = 0x7A000041
} USART_GPIO_t;

typedef enum {
	USART_OVERSAMPLING_16 =	0,
	USART_OVERSAMPLING_8 =	1,
} USART_oversampling_t;


// TODO: ASM!!
/*!< init / enable / disable */
void enable_USART(USART_t* usart);
void disable_USART(USART_t* usart);
// UART mode TODO: improve fconfig asm
void fconfig_UART(USART_GPIO_t tx, USART_GPIO_t rx, uint32_t baud, USART_oversampling_t oversampling);
void config_UART(USART_GPIO_t tx, USART_GPIO_t rx, uint32_t baud);
// TODO: irq!
/*!< input / output */
uint32_t USART_write(USART_t* usart, const uint8_t* buffer, uint32_t size, uint32_t timeout);
uint32_t USART_read(USART_t* usart, uint8_t* buffer, uint32_t size, uint32_t timeout);
uint8_t USART_print(USART_t* usart, char* str, uint32_t timeout);


#endif //STM32F412_USART_H
