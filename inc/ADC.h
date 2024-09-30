//
// Created by marijn on 9/25/24.
//

#ifndef STM32F412_ADC_H
#define STM32F412_ADC_H
#include "periph.h"
#include "GPIO.h"
#include "NVIC.h"


typedef enum {	// config_ADC
	ADC_CLK_DIV2 =			0b00U << 0,	// APB2 / 2
	ADC_CLK_DIV4 =			0b01U << 0,	// APB2 / 4
	ADC_CLK_DIV6 =			0b10U << 0,	// APB2 / 6
	ADC_CLK_DIV8 =			0b11U << 0	// APB2 / 8
} ADC_CLK_DIV_t;			// flags[:8] -> ADC_COMMON

typedef enum {	// config_ADC
	ADC_FEATURE_NONE =		0b00U << 6,
	ADC_FEATURE_VBAT =		0b01U << 6,	// enable dedicated Vbat channel
	ADC_FEATURE_TEMP_VREF =	0b10U << 6	// enable dedicated Vref and temp channel
} ADC_FEATURE_t;			// flags[:8] -> ADC_COMMON

typedef enum {	// config_ADC
	ADC_RES_12B =			0b00U << 8,	// 15 cycles for conversion
	ADC_RES_10B =			0b01U << 8,	// 13 cycles for conversion
	ADC_RES_8B =			0b10U << 8,	// 11 cycles for conversion
	ADC_RES_6B =			0b11U << 8	// 9 cycles for conversion
} ADC_RES_t;				// flags[8:] -> ADC1_CR1


typedef enum {	// config_ADC_watchdog
	ADC_WDG_CH0 =			0b00000U << 0,	ADC_WDG_CH1 =			0b00001U << 0,
	ADC_WDG_CH2 =			0b00010U << 0,	ADC_WDG_CH3 =			0b00011U << 0,
	ADC_WDG_CH4 =			0b00100U << 0,	ADC_WDG_CH5 =			0b00101U << 0,
	ADC_WDG_CH6 =			0b00110U << 0,	ADC_WDG_CH7 =			0b00111U << 0,
	ADC_WDG_CH8 =			0b01000U << 0,	ADC_WDG_CH9 =			0b01001U << 0,
	ADC_WDG_CH10 =			0b01010U << 0,	ADC_WDG_CH11 =			0b01011U << 0,
	ADC_WDG_CH12 =			0b01100U << 0,	ADC_WDG_CH13 =			0b01101U << 0,
	ADC_WDG_CH14 =			0b01110U << 0,	ADC_WDG_CH15 =			0b01111U << 0,
	ADC_WDG_CH16 =			0b10000U << 0,	ADC_WDG_CH17 =			0b10001U << 0,
	ADC_WDG_CH18 =			0b10010U << 0,
} ADC_WDG_CH_t;

typedef enum {  // config_ADC_watchdog
	ADC_WDG_IRQ_OFF =		0b0U << 6,
	ADC_WDG_IRQ_ON =		0b1U << 6
} ADC_WDG_IRQ_t;

typedef enum {  // config_ADC_watchdog
	ADC_WDG_SCAN_ALL =		0b0U << 9,
	ADC_WDG_SCAN_SINGLE =	0b1U << 9
} ADC_WDG_SCAN_t;

typedef enum {  // config_ADC_watchdog
	ADC_WDG_TYPE_DISABLE =	0b00U << 22,
	ADC_WDG_TYPE_INJECTED =	0b01U << 22,
	ADC_WDG_TYPE_REGULAR =	0b10U << 22,
	ADC_WDG_TYPE_BOTH =		0b11U << 22
} ADC_WDG_TYPE_t;


void config_ADC(uint16_t flags);
// TODO
void config_injected_ADC_GPIO(GPIO_t* port, uint8_t pin, uint8_t res, uint8_t EOC_int, uint8_t DISC, uint8_t CONT, uint8_t align, uint8_t trig, uint8_t trig_mode);
void config_ADC_watchdog(uint32_t flags);
void config_ADC_IRQ(uint8_t priority);


#endif //STM32F412_ADC_H
