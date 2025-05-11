//
// Created by marijn on 9/24/24.
//

#ifndef STM32F412_TIM_H
#define STM32F412_TIM_H
#include "periph.h"
#include "NVIC.h"


/*!<
 * types
 * */
typedef enum {
	TIM_PIN_DISABLE =	0x00000000UL,
	// TIM1
	TIM1_BKIN_A6 =		0x16000005UL,
	TIM1_CH1N_A7 =		0x17004005UL,
	TIM1_CH1_A8 =		0x18000005UL,
	TIM1_CH2_A9 =		0x19001005UL,
	TIM1_CH3_A10 =		0x1A002005UL,
	TIM1_CH4_A11 =		0x1B003005UL,
	TIM1_ETR_A12 =		0x1C000005UL,
	TIM1_CH2N_B0 =		0x10105005UL,
	TIM1_CH3N_B1 =		0x11106005UL,
	TIM1_BKIN_B12 =		0x1C100005UL,
	TIM1_CH1N_B13 =		0x1D104005UL,
	TIM1_CH2N_B14 =		0x1E105005UL,
	TIM1_CH3N_B15 =		0x1F106005UL,
	TIM1_ETR_E7 =		0x17400005UL,
	TIM1_CH1N_E8 =		0x18404005UL,
	TIM1_CH1_E9 =		0x19400005UL,
	TIM1_CH2N_E10 =		0x1A405005UL,
	TIM1_CH2_E11 =		0x1B401005UL,
	TIM1_CH3N_E12 =		0x1C406005UL,
	TIM1_CH3_E13 =		0x1D402005UL,
	TIM1_CH4_E14 =		0x1E403005UL,
	TIM1_BKIN_E15 =		0x1F400005UL,
	TIM1_ETR_F10 =		0x1A500005UL,
	// TIM2
	TIM2_CH1_A0	=		0x10000004UL,
	TIM2_ETR_A0	=		0x10000004UL,
	TIM2_CH2_A1	=		0x11001004UL,
	TIM2_CH3_A2	=		0x12002004UL,
	TIM2_CH4_A3	=		0x13003004UL,
	TIM2_CH1_A5	=		0x15000004UL,
	TIM2_ETR_A5	=		0x15000004UL,
	TIM2_CH1_A15 =		0x1F000004UL,
	TIM2_ETR_A15 =		0x1F000004UL,
	TIM2_CH2_B3 =		0x13101004UL,
	TIM2_CH3_B10 =		0x1A102004UL,
	TIM2_CH4_B11 =		0x1B103004UL,

	// TODO (AF1 done)
} TIM_GPIO_t;

typedef enum {
	TIM_TRGO_RESET =	0b000 << 4,
	TIM_TRGO_ENABLE =	0b001 << 4,
	TIM_TRGO_UPDATE =	0b010 << 4,
	TIM_TRGO_CC1IF =	0b011 << 4,
	TIM_TRGO_OC1REF =	0b100 << 4,
	TIM_TRGO_OC2REF =	0b101 << 4,
	TIM_TRGO_OC3REF =	0b110 << 4,
	TIM_TRGO_OC4REF =	0b111 << 4
} TIM_TRGO_t;

//typedef enum {
//	TIM_TRGO_RESET =	0b0 << 7,
//	TIM_TRGO_RESET =	0b0 << 7
//} TIM_TI1_t;



/*!< init / enable / disable */
void config_TIM(TIM_t* tim, uint32_t prescaler, uint32_t limit);
void config_TIM_master(TIM_t* tim, uint32_t prescaler, uint32_t limit, uint32_t flags);	// TODO: config channels
void config_TIM_slave(TIM_t* tim, uint32_t prescaler, uint32_t limit, uint32_t flags);	// TODO: config trigger
void disable_TIM(TIM_t* tim);
/*!< actions */
void start_TIM(TIM_t* tim);
void stop_TIM(TIM_t* tim);
void delay_TIM(TIM_t* tim, uint32_t count);
/*!< irq */
void start_TIM_update_irq(TIM_t* tim);
void stop_TIM_update_irq(TIM_t* tim);


#endif //STM32F412_TIM_H
