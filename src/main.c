#include "base.h"
#include "periph.h"
#include "sys.h"
#include "gpio.h"
#include "usart.h"
#include "RTC.h"


void RTC_tamper_stamp_handler(void) {
	for(;;);
}


// application
void main(void) {
	set_SYS_hardware_config(PWR_LVL_NOM, 25000000);						// 3v3, HSE@25MHz
	set_SYS_oscilator_config(0, 1, 0, 1, 1, 0, 0, 0, 0);				// enable HSE, LSE, CSS and the backup-domain
	set_SYS_PLL_config(PLL_CLK_SRC_HSE, 12U, 96U, PLL_P_DIV_2, 0, 0);	// enable PLL @ 25MHz / 12 * 96 / 2 -> 100MHz
	set_SYS_backup_domain_config();										// enable backup domain
	set_SYS_RTC_config(RTC_SRC_LSE, 0U);								// enable RTC
	set_SYS_clock_config(SYS_CLK_SRC_PLL_P, AHB_CLK_NO_DIV, APBx_CLK_DIV2, APBx_CLK_NO_DIV);
	set_SYS_tick_config(1);												// enable SYS_tick with interrupt
	sys_init();															// write settings

	config_GPIO(GPIOC, 13, GPIO_output, GPIO_pull_up, GPIO_open_drain);
	config_UART(USART1_TX_A9, USART1_RX_A10, 115200);
	uint32_t ts = 1726653600;
	uconfig_RTC(ts, RTC_WAKEUP_DISABLE, RTC_WAKEUP_DIV16, 0x0000U);
	//config_RTC_ext_ts(1U, RTC_TS_POLARITY_FALLING);  // TODO: enable int

	while (1) {
		GPIO_toggle(GPIOC, 13);
		ts = RTC_unix();
		USART_write(USART1, &ts, 4, 100);
		delay_ms(500);
		//*((uint32_t*)&tr) = RTC->TR;
		//*((uint32_t*)&dr) = RTC->DR;
		//(void)tr;
	}
}
