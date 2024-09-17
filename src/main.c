#include "base.h"
#include "periph.h"
#include "sys.h"
#include "gpio.h"
#include "usart.h"
#include "RTC.h"


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


	//fconfig_GPIO(GPIOA, 8, GPIO_output, GPIO_pull_up, GPIO_open_drain, GPIO_high_speed, 10);
	config_GPIO(GPIOC, 13, GPIO_output, GPIO_pull_up, GPIO_open_drain);

	volatile RCC_t* rcc = RCC;
	volatile USART_t* usart = USART1;
	volatile GPIO_t* porta = GPIOA;
	volatile RTC_t* rtc = RTC;
	config_UART(USART1_TX_A9, USART1_RX_A10, 115200);

	RTC_timestamp_t time;
	time.year = 24;
	time.month = 9;
	time.day = 17;
	time.hour = 18;
	time.min = 7;
	time.sec = 10;
	sconfig_RTC(time);

	volatile struct {
		uint32_t su	: 4;
		uint32_t st	: 3;
		uint32_t _0	: 1;
		uint32_t mu	: 4;
		uint32_t mt	: 3;
		uint32_t _1 : 1;
		uint32_t hu	: 4;
		uint32_t ht	: 2;
		uint32_t _2 : 10
	} tr;
	volatile struct {
		uint32_t du	: 4;
		uint32_t dt	: 2;
		uint32_t _0	: 2;
		uint32_t mu	: 4;
		uint32_t mt	: 1;
		uint32_t wd : 3;
		uint32_t yu	: 4;
		uint32_t yt	: 4;
		uint32_t _2 : 8;
	} dr;

	while (1) {
		//GPIO_toggle(GPIOC, 13);
		//USART_print(USART1, "Hello world!", 200);

		//*((uint32_t*)&tr) = RTC->TR;
		//*((uint32_t*)&dr) = RTC->DR;
		//(void)tr;

		uint32_t ep = RTC_epoch();
	}
}
