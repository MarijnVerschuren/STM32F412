//
// Created by marijn on 12/5/24.
//

#ifndef STM32F412_DW1000_H
#define STM32F412_DW1000_H
#include "SPI.h"


/*!<
 * defines
 * */
#define DW1000_TIMEOUT 10


/*!<
 * types
 * */
typedef struct {
	SPI_t* spi;
	GPIO_t* NSS_port;
	GPIO_t* NRST_port;
	uint8_t NSS_pin;
	uint8_t NRST_pin;
} DW1000_t;


/*!<
 * functions
 * */
void DW1000_init(DW1000_t* dw1000);


#endif //STM32F412_DW1000_H
