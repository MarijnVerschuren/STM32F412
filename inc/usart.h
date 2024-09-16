//
// Created by marijn on 9/16/24.
//

#ifndef STM32F412_USART_H
#define STM32F412_USART_H
#include "periph.h"


/*!<
 * types
 * */
typedef enum {
	USART_PIN_DISABLE =	0x00000000,
	// USART1
	USART_PIN_TEST_0 = 0xF4000FFF,
	USART_PIN_TEST_1 = 0xF5000FFF,
} USART_GPIO_t;

typedef enum {
	USART_OVERSAMPLING_16 =	0,
	USART_OVERSAMPLING_8 =	1,
} USART_oversampling_t;


/*!< misc */
uint8_t USART_to_int(USART_t* usart);
/*!< init / enable / disable */
void enable_USART(USART_t* usart);
void disable_USART(USART_t* usart);
void fconfig_USART(USART_GPIO_t tx, USART_GPIO_t rx, uint32_t baud, USART_oversampling_t oversampling);
void config_USART(USART_GPIO_t tx, USART_GPIO_t rx, uint32_t baud);


#endif //STM32F412_USART_H
