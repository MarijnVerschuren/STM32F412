//
// Created by marijn on 9/10/24.
//
#include "sys.h"



volatile extern struct {
	// RCC_CR			config
	uint32_t HSI_enable					: 1;
	uint32_t HSI_trim					: 5;
	uint32_t HSE_enable					: 1;
	uint32_t HSE_bypass_enable			: 1;
	uint32_t CSS_enable					: 1;
	uint32_t PLL_enable					: 1;
	uint32_t PLL_I2S_enable				: 1;
	// RCC_PLL_CFGR		config TODO
	uint32_t PLL_M						: 6;
	uint32_t PLL_N						: 9;
	uint32_t PLL_P						: 2;  // PLL_P_t
	uint32_t PLL_Q						: 4;  // (0, 1 are invalid)
	uint32_t PLL_R						: 3;  // (0, 1 are invalid)
	uint32_t PLL_src					: 1;  // PLL_source_t
	// RCC_CFGR			config TODO
	uint32_t sys_clk_src				: 2;  // SYS_clock_source_t
	uint32_t AHB_div					: 4;  // AHB_prescaler_t
	uint32_t APB1_div					: 3;  // APB_prescaler_t
	uint32_t APB2_div					: 3;  // APB_prescaler_t
	uint32_t RTC_div					: 5;  // RTC_prescaler_t
	uint32_t MCO1_src					: 2;  // MCO1_source_t
	uint32_t MCO1_div					: 3;  // MCO_prescaler_t
	uint32_t MCO2_src					: 2;  // MCO2_source_t
	uint32_t MCO2_div					: 3;  // MCO_prescaler_t
	// RCC_BDCR			config TODO
	uint32_t LSE_enable					: 1;
	uint32_t LSE_bypass_enable			: 1;
	uint32_t LSE_low_power_disable		: 1;
	uint32_t RTC_enable					: 1;
	uint32_t RTC_src					: 2;  // RTC_source_t
	// RCC_CSR			config
	uint32_t LSI_enable					: 1;
	// RCC_PLL_I2S_CFGR	config
	uint32_t PLL_I2S_M					: 6;
	uint32_t PLL_I2S_N					: 9;
	uint32_t PLL_I2S_Q					: 4;  // (0, 1 are invalid)
	uint32_t PLL_I2S_R					: 3;  // (0, 1 are invalid)
	uint32_t PLL_I2S_src				: 1;  // PLL_I2S_source_t
	// RCC_DCK_CFGR		config
	uint32_t DFSDM1_src					: 1;  // DFSDM_source_t
	uint32_t DFSDM1A_src				: 1;  // DFSDM_A_source_t
	uint32_t TIM_prescaler				: 1;  // TIM_prescaler_t
	uint32_t I2S1_src					: 2;  // I2S_source_t
	uint32_t I2S2_src					: 2;  // I2S_source_t
	// FLASH_ TODO
	uint32_t FLASH_prefetch				: 1;
	uint32_t FLASH_instruction_cache	: 1;
	uint32_t FLASH_data_cache			: 1;
	// hardware info
	uint32_t PWR_level					: 2;  // PWR_level_t
	uint32_t HSE_frequency				: 23;
} sys_config;



void set_SYS_hardware_config(PWR_level_t power_level, uint32_t HSE_frequency) {
	sys_config.PWR_level =				power_level;
	sys_config.HSE_frequency =			HSE_frequency;
}
void set_SYS_oscilator_config(
	uint8_t HSI_enable, uint8_t HSE_enable, uint8_t LSI_enable, uint8_t LSE_enable, uint8_t CSS_enable,
	uint8_t HSE_bypass_enable, uint8_t LSE_bypass_enable, uint8_t LSE_low_power_disable, uint8_t HSI_trim
) {
	sys_config.HSI_enable =				HSI_enable;
	sys_config.HSE_enable =				HSE_enable;
	sys_config.LSI_enable =				LSI_enable;
	sys_config.LSE_enable =				LSE_enable;
	sys_config.CSS_enable =				CSS_enable;
	sys_config.HSE_bypass_enable =		HSE_bypass_enable;
	sys_config.LSE_bypass_enable =		LSE_bypass_enable;
	sys_config.LSE_low_power_disable =	LSE_low_power_disable;
	sys_config.HSI_trim = HSI_trim;
}
void set_SYS_PLL_config(PLL_CLK_SRC_t src, uint8_t PLL_M, uint8_t PLL_N, PLL_P_t PLL_P, uint8_t PLL_Q, uint8_t PLL_R) {
	sys_config.PLL_enable =				0b1U;
	sys_config.PLL_src =				src;
	sys_config.PLL_M =					PLL_M;
	sys_config.PLL_N =					PLL_N;
	sys_config.PLL_P =					PLL_P;
	sys_config.PLL_Q =					PLL_Q;
	sys_config.PLL_R =					PLL_R;
}
void set_SYS_clock_config(SYS_CLK_SRC_t src, AHB_DIV_t AHB_div, APBx_DIV_t APB1_div, APBx_DIV_t APB2_div) {
	sys_config.sys_clk_src =			src;
	sys_config.AHB_div =				AHB_div;
	sys_config.APB1_div =				APB1_div;
	sys_config.APB2_div =				APB2_div;
}
void set_SYS_MCO_config(MCO1_SRC_t MCO1_src, MCOx_DIV_t MCO1_div, MCO2_SRC_t MCO2_src, MCOx_DIV_t MCO2_div) {
	sys_config.MCO1_src =				MCO1_src;
	sys_config.MCO1_div =				MCO1_div;
	sys_config.MCO2_src =				MCO2_src;
	sys_config.MCO2_div =				MCO2_div;
}
void set_SYS_RTC_config(RTC_SRC_t RTC_src, uint8_t RTC_div) {
	sys_config.RTC_enable =				0b1U;
	sys_config.RTC_src =				RTC_src;
	sys_config.RTC_div =				RTC_div;
}
void set_SYS_PLL_I2S_config(PLL_I2S_CLK_SRC_t src, uint8_t PLL_M, uint8_t PLL_N, uint8_t PLL_Q, uint8_t PLL_R) {
	sys_config.PLL_I2S_enable =			0b1U;
	sys_config.PLL_I2S_src =			src;
	sys_config.PLL_I2S_M =				PLL_M;
	sys_config.PLL_I2S_N =				PLL_N;
	sys_config.PLL_I2S_Q =				PLL_Q;
	sys_config.PLL_I2S_R =				PLL_R;
}
void set_SYS_periph_clocks_config(TIM_MUL_t tim_prescaler, I2S_SRC_t I2S1_src, I2S_SRC_t I2S2_src, DFSDM_A_CLK_SRC_t DFSDM_A_src, DFSDM_CLK_SRC_t DFSDM_src) {

}


void sys_init(void) {
	// FLASH!!
	// PWR!!
}
