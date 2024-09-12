//
// Created by marijn on 9/11/24.
//

#ifndef STM32F412_MCO_H
#define STM32F412_MCO_H


typedef enum {
	MCO1_SRC_HSI =	0b00U,
	MCO1_SRC_LSE =	0b01U,
	MCO1_SRC_HSE =	0b10U,
	MCO1_SRC_PLL =	0b11U
} MCO1_SRC_t;

typedef enum {
	MCO2_SRC_HSI =	0b00U,
	MCO2_SRC_LSE =	0b01U,
	MCO2_SRC_HSE =	0b10U,
	MCO2_SRC_PLL =	0b11U
} MCO2_SRC_t;

typedef enum {
	MCOx_NO_DIV =	0b000U,
	MCOx_DIV2 =		0b100U,
	MCOx_DIV3 =		0b101U,
	MCOx_DIV4 =		0b110U,
	MCOx_DIV5 =		0b111U
} MCOx_DIV_t;

#endif //STM32F412_MCO_H
