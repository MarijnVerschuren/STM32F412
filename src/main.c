#include "types.h"
#include "periph.h"
#include "sys.h"
#include "gpio.h"

#define PIN 8


// application
void main(void) {
	set_SYS_hardware_config(PWR_LVL_NOM, 25000000);						// 3v3, HSE@25MHz
	set_SYS_oscilator_config(0, 1, 0, 0, 1, 0, 0, 0, 0);				// enable HSE and CSS
	set_SYS_PLL_config(PLL_CLK_SRC_HSE, 12U, 96U, PLL_P_DIV_2, 0, 0);	// enable PLL @ 25MHz / 12 * 96 / 2 -> 100MHz
	set_SYS_clock_config(SYS_CLK_SRC_PLL_P, AHB_CLK_NO_DIV, APBx_CLK_DIV2, APBx_CLK_NO_DIV);
	set_SYS_tick_config(1);												// enable SYS_tick with interrupt
	sys_init();															// write settings

	enable_GPIO(GPIOA);
	GPIOA->MODER |= 0b01 << (2*PIN);

	while (1) {
		GPIOA->BSRR = 0x1 << PIN;
		delay_ms(50);
		GPIOA->BSRR = 0x10000 << PIN;
		delay_ms(50);
	}
}
