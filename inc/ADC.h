//
// Created by marijn on 9/25/24.
//

#ifndef STM32F412_ADC_H
#define STM32F412_ADC_H
#include "periph.h"
#include "GPIO.h"
#include "NVIC.h"


typedef enum {	// config_ADC
	ADC_CLK_DIV2 =					0b00U << 0,	// APB2 / 2
	ADC_CLK_DIV4 =					0b01U << 0,	// APB2 / 4
	ADC_CLK_DIV6 =					0b10U << 0,	// APB2 / 6
	ADC_CLK_DIV8 =					0b11U << 0	// APB2 / 8
} ADC_CLK_DIV_t;					// flags[:8] -> ADC_COMMON

typedef enum {	// config_ADC
	ADC_FEATURE_NONE =				0b00U << 6,
	ADC_FEATURE_VBAT =				0b01U << 6,	// enable dedicated Vbat channel
	ADC_FEATURE_TEMP_VREF =			0b10U << 6	// enable dedicated Vref and temp channel
} ADC_FEATURE_t;					// flags[:8] -> ADC_COMMON

typedef enum {	// config_ADC
	ADC_MODE_DEFAULT =				0b0 << 8,
	ADC_MODE_SCAN =					0b1 << 8	// enable scan mode
} ADC_MODE_t;						// flags[8:32] -> ADC1_CR1

typedef enum {	// config_ADC
	ADC_DISC_DISABLE =				0b0U << 11,
	ADC_DISC_ENABLE =				0b1U << 11
} ADC_DISC_t;						// flags[8:32] -> ADC1_CR1

typedef enum {	// config_ADC
	ADC_INJECTED_DISC_DISABLE =		0b0U << 12,
	ADC_INJECTED_DISC_ENABLE =		0b1U << 12
} ADC_INJECTED_DISC_t;				// flags[8:32] -> ADC1_CR1

typedef enum {	// config_ADC
	ADC_DISC1 =						0b000U << 13,
	ADC_DISC2 =						0b001U << 13,
	ADC_DISC3 =						0b010U << 13,
	ADC_DISC4 =						0b011U << 13,
	ADC_DISC5 =						0b100U << 13,
	ADC_DISC6 =						0b101U << 13,
	ADC_DISC7 =						0b110U << 13,
	ADC_DISC8 =						0b111U << 13
} ADC_DISC_COUNT_t;					// flags[8:32] -> ADC1_CR1

typedef enum {	// config_ADC
	ADC_RES_12B =					0b00U << 24,	// 15 cycles for conversion
	ADC_RES_10B =					0b01U << 24,	// 13 cycles for conversion
	ADC_RES_8B =					0b10U << 24,	// 11 cycles for conversion
	ADC_RES_6B =					0b11U << 24		// 9 cycles for conversion
} ADC_RES_t;						// flags[8:32] -> ADC1_CR1

typedef enum {	// config_ADC
	ADC_CONT_DISABLE =				0b0U << 33,		// single conversion mode
	ADC_CONT_ENABLE =				0b1U << 33		// continuous conversion mode
} ADC_CONT_t;						// flags[32:] -> ADC1_CR2

typedef enum {	// config_ADC
	ADC_EOC_SEQUENCE =				0b0U << 42,		// EOC set after sequence conversion
	ADC_EOC_SINGLE =				0b1U << 42		// EOC set after single conversion
} ADC_EOC_t;						// flags[32:] -> ADC1_CR2

typedef enum {	// config_ADC
	ADC_ALIGN_RIGHT =				0b0U << 43,
	ADC_ALIGN_LEFT =				0b1U << 43
} ADC_ALIGN_t;						// flags[32:] -> ADC1_CR2

typedef enum {	// config_ADC
	ADC_INJECTED_TRIG_TIM1_CC4 =	0b0000U << 48,
	ADC_INJECTED_TRIG_TIM1_TRGO =	0b0001U << 48,
	ADC_INJECTED_TRIG_TIM2_CC1 =	0b0010U << 48,
	ADC_INJECTED_TRIG_TIM2_TRGO =	0b0011U << 48,
	ADC_INJECTED_TRIG_TIM3_CC2 =	0b0100U << 48,
	ADC_INJECTED_TRIG_TIM3_CC4 =	0b0101U << 48,
	ADC_INJECTED_TRIG_TIM4_CC1 =	0b0110U << 48,
	ADC_INJECTED_TRIG_TIM4_CC2 =	0b0111U << 48,
	ADC_INJECTED_TRIG_TIM4_CC3 =	0b1000U << 48,
	ADC_INJECTED_TRIG_TIM4_TRGO =	0b1001U << 48,
	ADC_INJECTED_TRIG_TIM5_CC4 =	0b1010U << 48,
	ADC_INJECTED_TRIG_TIM5_TRGO =	0b1011U << 48,
	ADC_INJECTED_TRIG_TIM8_CC2 =	0b1100U << 48,
	ADC_INJECTED_TRIG_TIM8_CC3 =	0b1101U << 48,
	ADC_INJECTED_TRIG_TIM8_CC4 =	0b1110U << 48,
	ADC_INJECTED_TRIG_EXTI_15 =		0b1111U << 48
} ADC_INJECTED_TRIG_t;				// flags[32:] -> ADC1_CR2

typedef enum {    // config_ADC
	ADC_INJECTED_TRIG_MODE_NONE =	0b00 << 52,
	ADC_INJECTED_TRIG_MODE_RISING =	0b01 << 52,
	ADC_INJECTED_TRIG_MODE_FALING =	0b10 << 52,
	ADC_INJECTED_TRIG_MODE_BOTH =	0b11 << 52
} ADC_INJECTED_TRIG_MODE_t;

typedef enum {	// config_ADC
	ADC_TRIG_TIM1_CC1 =				0b0000U << 56,
	ADC_TRIG_TIM1_CC2 =				0b0001U << 56,
	ADC_TRIG_TIM1_CC3 =				0b0010U << 56,
	ADC_TRIG_TIM2_CC2 =				0b0011U << 56,
	ADC_TRIG_TIM2_CC3 =				0b0100U << 56,
	ADC_TRIG_TIM2_CC4 =				0b0101U << 56,
	ADC_TRIG_TIM2_TRGO =			0b0110U << 56,
	ADC_TRIG_TIM3_CC1 =				0b0111U << 56,
	ADC_TRIG_TIM3_TRGO =			0b1000U << 56,
	ADC_TRIG_TIM4_CC4 =				0b1001U << 56,
	ADC_TRIG_TIM5_CC1 =				0b1010U << 56,
	ADC_TRIG_TIM5_CC2 =				0b1011U << 56,
	ADC_TRIG_TIM5_CC3 =				0b1100U << 56,
	ADC_TRIG_TIM8_CC1 =				0b1101U << 56,
	ADC_TRIG_TIM8_TRGO =			0b1110U << 56,
	ADC_TRIG_EXTI_11 =				0b1111U << 56
} ADC_TRIG_t;						// flags[32:] -> ADC1_CR2

typedef enum {    // config_ADC
	ADC_TRIG_MODE_NONE =			0b00 << 60,
	ADC_TRIG_MODE_RISING =			0b01 << 60,
	ADC_TRIG_MODE_FALING =			0b10 << 60,
	ADC_TRIG_MODE_BOTH =			0b11 << 60
} ADC_TRIG_MODE_t;


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
	ADC_WDG_SCAN_ALL =		0b0U << 9,
	ADC_WDG_SCAN_SINGLE =	0b1U << 9
} ADC_WDG_SCAN_t;

typedef enum {  // config_ADC_watchdog
	ADC_WDG_TYPE_DISABLE =	0b00U << 22,
	ADC_WDG_TYPE_INJECTED =	0b01U << 22,
	ADC_WDG_TYPE_REGULAR =	0b10U << 22,
	ADC_WDG_TYPE_BOTH =		0b11U << 22
} ADC_WDG_TYPE_t;


typedef enum {  // config_ADC_IRQ
	ADC_IRQ_DISABLE_EOC =	0b0U << 5,
	ADC_IRQ_ENABLE_EOC =	0b1U << 5
} ADC_IRQ_EOC_t;

typedef enum {  // config_ADC_IRQ
	ADC_IRQ_DISABLE_WDG =	0b0U << 6,
	ADC_IRQ_ENABLE_WDG =	0b1U << 6
} ADC_IRQ_WDG_t;

typedef enum {  // config_ADC_IRQ
	ADC_IRQ_DISABLE_JEOC =	0b0U << 7,
	ADC_IRQ_ENABLE_JEOC =	0b1U << 7
} ADC_IRQ_JEOC_t;

typedef enum {  // config_ADC_IRQ
	ADC_IRQ_DISABLE_OVR =	0b0U << 26,
	ADC_IRQ_ENABLE_OVR =	0b1U << 26
} ADC_IRQ_OVR_t;


typedef enum {
	ADC_SAMPLE_3_CYCLES =	0b000,
	ADC_SAMPLE_15_CYCLES =	0b001,
	ADC_SAMPLE_28_CYCLES =	0b010,
	ADC_SAMPLE_56_CYCLES =	0b011,
	ADC_SAMPLE_84_CYCLES =	0b100,
	ADC_SAMPLE_112_CYCLES =	0b101,
	ADC_SAMPLE_144_CYCLES =	0b110,
	ADC_SAMPLE_480_CYCLES =	0b111
} ADC_SAMPLE_TIME_t;


void config_ADC(uint64_t flags);
void config_ADC_watchdog(uint32_t flags, uint16_t high_threshold, uint16_t low_threshold);
void config_ADC_IRQ(uint8_t priority, uint32_t flags);

void config_ADC_GPIO_channel(GPIO_t* port, uint8_t pin, ADC_SAMPLE_TIME_t sample_time);

void start_ADC(uint8_t regular, uint8_t injected);


#endif //STM32F412_ADC_H
